/**
 * Copyright (c) 2026 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of the Parrot Company nor the names
 *   of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written
 *   permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * PARROT COMPANY BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Image classifier example in C++: consumes camera frames from the SDK input
 * source, preprocesses them (NV12 -> normalized RGB 224x224) and runs a
 * non-blocking infer(). A worker thread pulls the freshest frame and does the
 * (CPU-heavy) preprocessing off the loop, so frame reception is never blocked;
 * the top-5 result comes back on the loop in onInferResult(). Results are
 * logged.
 */

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <fstream>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <libpomp.hpp>
#include <opencv2/opencv.hpp>
#include <ulog.hpp>

#include <airsdk/neural_network.hpp>
#include <airsdk/neural_network/blob.hpp>
#include <airsdk/neural_network/input.hpp>

#define ULOG_TAG image_classifier_cpp
ULOG_DECLARE_TAG(ULOG_TAG);

namespace nn = airsdk::neural_network;
namespace nninput = airsdk::neural_network::input;

#define UID "com.parrot.missions.samples.image_classifier_cpp"
#define BASE_PATH "/mnt/missions/" UID "/payload/etc/services/classification"
#define STREAM_NAME "fcam_airsdk"
/* Steady-state timeout: infer()'s first call is floored to 30s internally by
 * the SDK to cover the model load, so this only needs to size normal
 * inference latency. */
#define INFERENCE_TIMEOUT_MS 5000
#define DEFAULT_NUM_CLASSES 1000

static volatile sig_atomic_t s_run = 1;
static void sig_handler(int)
{
	s_run = 0;
}

namespace {
cv::Mat preProcessFrame(const nninput::Sample &sample);
std::vector<float> softmax(const std::vector<float> &logits);
std::vector<size_t> argSort(const std::vector<float> &v);
} // namespace

/**
 * @brief Image-classification pipeline.
 *
 * Owns the network and the camera source and implements the network listener.
 * A worker thread pulls frames (waitSample), preprocesses off the loop and
 * fires a non-blocking infer(); onInferResult() (on the loop) logs the top-5.
 */
class Classifier : public nn::NeuralNetworkListener {
public:
	explicit Classifier(pomp::Loop &loop) : mLoop(loop) {}

	~Classifier() override
	{
		mRunning = false;
		if (mWorker.joinable())
			mWorker.join();
		mSource.reset();
		delete mNeuralNetwork;
	}

	int init()
	{
		loadLabels();

		nn::Settings settings;
		settings.netId = "airsdk@ResNet50";
		settings.modelFile = std::string(BASE_PATH) + "/ResNet50_quantized.dlc";
		settings.outputLayers = {"/fc/Gemm"};
		// settings.diagLogFolder = BASE_PATH; // debug only: adds per-inference I/O overhead
		mNeuralNetwork = new nn::NeuralNetwork(&settings, this, mLoop,
						       INFERENCE_TIMEOUT_MS);

		nninput::VipcSettings vs;
		vs.stream = STREAM_NAME;
		mSource = nninput::makeVipcSource(mLoop, vs);
		if (!mSource)
			return -ENOMEM;

		mRunning = true;
		mWorker = std::thread(&Classifier::worker, this);
		return 0;
	}

	/* Neural-network events (on the mission loop). */
	void onStatus(nn::NeuralNetwork *, bool isReady) override
	{
		mReady = isReady;
		ULOGI("network %s", isReady ? "ready" : "not ready");
		/* Some missions want the Parrot on-board networks off the NPU
		 * they use, for performance. */
		if (isReady)
			mNeuralNetwork->deactivateNativeNns(true);
	}

	/* Re-enable native NNs on shutdown, or the drone stays degraded. */
	void deactivateNativeNns(bool deactivate)
	{
		if (mNeuralNetwork)
			mNeuralNetwork->deactivateNativeNns(deactivate);
	}

	void onError(nn::NeuralNetwork *, std::string reason) override
	{
		ULOGW("network error: %s", reason.c_str());
	}

	void onInferResult(nn::NeuralNetwork *,
			   std::vector<nn::Blob> &&outputs,
			   uint64_t /*timestamp_ns*/) override
	{
		if (outputs.empty())
			return;
		const float *p = reinterpret_cast<const float *>(outputs[0].ptr());
		std::vector<float> scores(p, p + outputs[0].attributes().channel);

		auto prob = softmax(scores);
		auto idxs = argSort(prob);
		for (size_t i = 0; i < 5 && i < idxs.size(); ++i) {
			const char *name = idxs[i] < mLabels.size()
						   ? mLabels[idxs[i]].c_str()
						   : "unknown";
			ULOGI("Top%zu: %s (%.1f%%)", i + 1, name,
			      prob[idxs[i]] * 100.f);
		}
	}

	void onInferError(nn::NeuralNetwork *, int error,
			  uint64_t /*timestamp_ns*/) override
	{
		ULOGW("inference failed: %d", error);
	}

private:
	/* Worker thread: pull the freshest frame, preprocess it off the loop
	 * and fire a non-blocking inference. */
	void worker()
	{
		while (mRunning) {
			std::shared_ptr<const nninput::Sample> sample =
				mSource->waitSample(1000);
			if (!sample)
				continue; /* timeout: the source reconnects itself */
			if (!mReady)
				continue; /* network not ready yet */

			cv::Mat tensor = preProcessFrame(*sample);
			if (tensor.empty()) {
				ULOGE("preprocessing failed");
				continue;
			}
			if (!tensor.isContinuous()) {
				/* infer()'s input must be one packed block; a
				 * non-continuous Mat (e.g. an ROI) would
				 * silently feed the wrong bytes. */
				ULOGE("preprocessing failed: tensor is not contiguous");
				continue;
			}

			size_t size = tensor.rows * tensor.cols *
				      tensor.channels() * sizeof(float);
			nn::InferInput input{
				nn::BlobAttributes("input", "input", 1,
						   tensor.rows, tensor.cols,
						   tensor.channels()),
				tensor.data, size};

			size_t nClasses = mLabels.empty() ? DEFAULT_NUM_CLASSES
							  : mLabels.size();
			/* Non-blocking: infer copies the input and returns
			 * immediately; -EBUSY means a previous inference is
			 * still running, so we drop this frame (one at a time). */
			int rc = mNeuralNetwork->infer(
				{input}, nClasses * sizeof(float),
				sample->timestampNs);
			if (rc < 0 && rc != -EBUSY)
				ULOGW("infer failed: %d", rc);
		}
	}

	void loadLabels()
	{
		std::ifstream f(std::string(BASE_PATH) + "/classes_names.csv");
		if (!f.is_open()) {
			ULOGW("cannot open labels file");
			return;
		}
		std::string line;
		while (std::getline(f, line))
			mLabels.push_back(line);
		ULOGI("loaded %zu class names", mLabels.size());
	}

	pomp::Loop &mLoop;
	nn::NeuralNetwork *mNeuralNetwork = nullptr;
	std::unique_ptr<nninput::Source> mSource;
	std::vector<std::string> mLabels;
	std::atomic<bool> mReady{false};   /* set on the loop by onStatus */
	std::atomic<bool> mRunning{false}; /* worker run flag */
	std::thread mWorker;
};

int main()
{
	signal(SIGTERM, sig_handler);
	signal(SIGINT, sig_handler);
	ULOGI("Starting image_classifier_cpp mission");

	pomp::Loop loop;
	Classifier classifier(loop);
	if (classifier.init() != 0) {
		ULOGE("failed to initialize classifier");
		return -1;
	}

	/* Pump the loop: this is where onStatus / onInferResult run. The finite
	 * timeout lets us notice SIGTERM promptly. */
	while (s_run)
		loop.waitAndProcess(500);

	classifier.deactivateNativeNns(false);

	ULOGI("Stopping image_classifier_cpp mission");
	return 0;
}

/* ------------------------------------------------------------ */
/* --- Local processing functions ----------------------------- */
/* ------------------------------------------------------------ */

namespace {

std::vector<float> softmax(const std::vector<float> &logits)
{
	std::vector<float> exps(logits.size());
	float maxLogit = *std::max_element(logits.begin(), logits.end());

	float sum = 0.f;
	for (size_t i = 0; i < logits.size(); ++i) {
		exps[i] = std::exp(logits[i] - maxLogit);
		sum += exps[i];
	}
	for (float &val : exps)
		val /= sum;
	return exps;
}

std::vector<size_t> argSort(const std::vector<float> &v)
{
	std::vector<size_t> idx(v.size());
	std::iota(idx.begin(), idx.end(), 0);
	std::sort(idx.begin(), idx.end(),
		  [&v](size_t i1, size_t i2) { return v[i1] > v[i2]; });
	return idx;
}

/* Converts a raw NV12 sample into a normalized tensor (RGB, float32, 224x224).
 * Assumes NV12: guaranteed by the AirSDK camera stream (vipc fcam_airsdk) this
 * mission reads from. sample.format carries the actual vacq pixel format if
 * you read from a different source. */
cv::Mat preProcessFrame(const nninput::Sample &sample)
{
	if (sample.planes.size() < 2 || sample.width == 0 ||
	    sample.height == 0) {
		ULOGE("preProcessFrame: unexpected sample (planes=%zu %ux%u)",
		      sample.planes.size(), sample.width, sample.height);
		return cv::Mat();
	}

	cv::Mat y_plane(sample.height, sample.width, CV_8UC1,
			(void *)sample.planeData(0), sample.planes[0].stride);

	cv::Mat uv_plane(sample.height / 2, sample.width, CV_8UC1,
			 (void *)sample.planeData(1), sample.planes[1].stride);

	/* Center crop the Y/UV planes before color conversion, not after: on
	 * a wide source (e.g. 1280x720) this halves the pixels cvtColor has
	 * to touch instead of converting margins we're about to discard.
	 * Offsets are rounded to even so the crop stays chroma-aligned. */
	int crop_size = std::min(sample.width, sample.height);
	crop_size = crop_size / 2 * 2;
	int x_off = (sample.width - crop_size) / 2 / 2 * 2;
	int y_off = (sample.height - crop_size) / 2 / 2 * 2;

	cv::Mat nv12_image(crop_size + crop_size / 2, crop_size, CV_8UC1);
	y_plane(cv::Rect(x_off, y_off, crop_size, crop_size))
		.copyTo(nv12_image(cv::Rect(0, 0, crop_size, crop_size)));
	uv_plane(cv::Rect(x_off, y_off / 2, crop_size, crop_size / 2))
		.copyTo(nv12_image(cv::Rect(0, crop_size, crop_size,
					    crop_size / 2)));

	cv::Mat cropped;
	cv::cvtColor(nv12_image, cropped, cv::COLOR_YUV2RGB_NV12);

	cv::Mat float_image;
	cropped.convertTo(float_image, CV_32FC3, 1.f / 255.f);
	cv::resize(float_image, float_image, cv::Size(224, 224));

	cv::Scalar mean(0.485, 0.456, 0.406);
	cv::Scalar stddev(0.229, 0.224, 0.225);

	cv::Mat normalized;
	cv::subtract(float_image, mean, normalized);
	cv::divide(normalized, stddev, normalized);

	return normalized;
}

} // namespace
