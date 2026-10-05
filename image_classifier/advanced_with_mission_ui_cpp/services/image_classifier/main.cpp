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
 * Same manual pipeline as image_classifier/advanced_cpp, but each top-5
 * result is also broadcast to a Mission UI side panel (appserver.hpp/.cpp,
 * shared with mission_ui_cpp).
 */

#include "appserver.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <json-c/json.h>
#include <libpomp.hpp>
#include <opencv2/opencv.hpp>
#include <ulog.hpp>

#include <airsdk/neural_network.hpp>
#include <airsdk/neural_network/blob.hpp>
#include <airsdk/neural_network/input.hpp>
#include <airsdk/publisher.hpp>

#define ULOG_TAG image_classifier_ui_cpp
ULOG_DECLARE_TAG(ULOG_TAG);

namespace nn = airsdk::neural_network;
namespace nninput = airsdk::neural_network::input;

#define UID "com.parrot.missions.samples.image_classifier_ui_cpp"
#define BASE_PATH "/mnt/missions/" UID "/payload/etc/services/classification"
#define STREAM_NAME "fcam_airsdk"
/* 30s: first run includes SNPE model loading on the NPU (~10-30s). */
#define INFERENCE_TIMEOUT_MS 30000
#define DEFAULT_NUM_CLASSES 1000

static airsdk::publisher::Publisher *g_publisher = nullptr;

namespace {
cv::Mat preProcessFrame(const nninput::Sample &sample);
std::vector<float> softmax(const std::vector<float> &logits);
std::vector<size_t> argSort(const std::vector<float> &v);
} // namespace

/* Manual inference path: a worker thread drives allocate/load/run/get and
 * reports the top-5. The listener only handles readiness/errors. */
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

	/* No auto-deactivate here: only the UI button calls deactivateNativeNns(). */
	void onStatus(nn::NeuralNetwork *, bool isReady) override
	{
		mReady = isReady;
		ULOGI("network %s", isReady ? "ready" : "not ready");
		publishStatus(); /* tell the panel even if isReady didn't change it */
	}

	/* Tracked so the panel can be told the real state, not just what was asked. */
	void deactivateNativeNns(bool deactivate)
	{
		mNativeNnsDeactivated = deactivate;
		if (mNeuralNetwork)
			mNeuralNetwork->deactivateNativeNns(deactivate);
	}

	/* Gates the worker's inference calls; the UI starts it explicitly. */
	void setInferEnabled(bool enabled)
	{
		mInferEnabled = enabled;
	}

	/* Broadcasts the current state; called after every change. */
	void publishStatus() const
	{
		struct json_object *root = json_object_new_object();
		struct json_object *status = json_object_new_object();
		json_object_object_add(
			status,
			"native_nns_deactivated",
			json_object_new_boolean(mNativeNnsDeactivated));
		json_object_object_add(
			status,
			"inference_enabled",
			json_object_new_boolean(mInferEnabled));
		json_object_object_add(root, "status", status);
		broadcast(json_object_to_json_string(root));
		json_object_put(root);
	}

	void onError(nn::NeuralNetwork *, std::string reason) override
	{
		ULOGW("network error: %s", reason.c_str());
	}

private:
	/* Preprocessing and the manual inference sequence are blocking, hence the thread. */
	void worker()
	{
		while (mRunning) {
			std::shared_ptr<const nninput::Sample> sample =
				mSource->waitSample(1000);
			if (!sample)
				continue; /* timeout: the source reconnects itself */
			if (!mReady)
				continue; /* network not ready yet */
			if (!mInferEnabled)
				continue; /* paused until the UI starts it */

			cv::Mat tensor = preProcessFrame(*sample);
			if (tensor.empty()) {
				ULOGE("preprocessing failed");
				continue;
			}
			if (!tensor.isContinuous()) {
				/* A non-continuous Mat would feed the wrong bytes. */
				ULOGE("preprocessing failed: tensor is not contiguous");
				continue;
			}
			inferManual(tensor, sample->timestampNs);
		}
	}

	/* One manual inference: allocate -> load -> run -> fetch -> log +
	 * broadcast. */
	void inferManual(const cv::Mat &tensor, uint64_t timestamp_ns)
	{
		size_t inSize = tensor.rows * tensor.cols * tensor.channels() *
				sizeof(float);
		int err = 0;

		std::unique_ptr<nn::NNBuffer> inBuf =
			mNeuralNetwork->allocateNNBufSync(inSize, &err);
		if (!inBuf) {
			ULOGW("allocate input buffer failed: %d", err);
			return;
		}

		std::vector<nn::Blob> inputs;
		inputs.emplace_back(
			std::move(inBuf),
			nn::BlobAttributes("input", "input", 1, tensor.rows,
					   tensor.cols, tensor.channels()),
			tensor.data, inSize);

		if (mNeuralNetwork->loadInputBlobs(inputs, timestamp_ns) < 0) {
			ULOGW("loadInputBlobs failed");
			return;
		}
		if (mNeuralNetwork->run() < 0) {
			ULOGW("run failed");
			return;
		}

		size_t nClasses = mLabels.empty() ? DEFAULT_NUM_CLASSES
						  : mLabels.size();
		std::unique_ptr<nn::NNBuffer> outBuf =
			mNeuralNetwork->allocateNNBufSync(
				nClasses * sizeof(float), &err);
		if (!outBuf) {
			ULOGW("allocate output buffer failed: %d", err);
			return;
		}

		std::vector<nn::Blob> outputs;
		if (mNeuralNetwork->getOutputBlobs(outputs, std::move(outBuf)) <
		    0) {
			ULOGW("getOutputBlobs failed");
			return;
		}
		reportTop5(outputs);
	}

	/* Runs on the worker thread; broadcast() is thread-safe. */
	void reportTop5(const std::vector<nn::Blob> &outputs)
	{
		if (outputs.empty())
			return;
		const float *p = reinterpret_cast<const float *>(outputs[0].ptr());
		std::vector<float> scores(p, p + outputs[0].attributes().channel);

		auto prob = softmax(scores);
		auto idxs = argSort(prob);

		struct json_object *root = json_object_new_object();
		struct json_object *classification = json_object_new_object();
		struct json_object *predictions = json_object_new_array();

		for (size_t i = 0; i < 5 && i < idxs.size(); ++i) {
			const char *name = idxs[i] < mLabels.size()
						   ? mLabels[idxs[i]].c_str()
						   : "unknown";
			float score = prob[idxs[i]];
			ULOGI("Top%zu: %s (%.1f%%)", i + 1, name, score * 100.f);

			struct json_object *pred = json_object_new_object();
			json_object_object_add(
				pred, "label", json_object_new_string(name));
			json_object_object_add(
				pred, "score", json_object_new_double(score));
			json_object_array_add(predictions, pred);
		}

		json_object_object_add(classification, "predictions", predictions);
		json_object_object_add(
			classification, "fps", json_object_new_double(inferFps()));
		json_object_object_add(root, "classification", classification);
		broadcast(json_object_to_json_string(root));
		json_object_put(root);
	}

	/* Counts completed inferences per second, refreshed once a second;
	 * returns the last computed value in between. */
	double inferFps()
	{
		auto now = std::chrono::steady_clock::now();
		++mFpsCount;
		double elapsed =
			std::chrono::duration<double>(now - mFpsWindowStart).count();
		if (elapsed >= 1.0) {
			mFpsValue = mFpsCount / elapsed;
			mFpsCount = 0;
			mFpsWindowStart = now;
		}
		return mFpsValue;
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
	std::atomic<bool> mNativeNnsDeactivated{false}; /* UI-controlled, see deactivateNativeNns() */
	std::atomic<bool> mInferEnabled{false}; /* UI-controlled, starts paused */
	int mFpsCount = 0;
	double mFpsValue = 0;
	std::chrono::steady_clock::time_point mFpsWindowStart =
		std::chrono::steady_clock::now();
	std::thread mWorker;
};

/* Passed to start_webserver so it can publish the chosen port. Static: only
 * used within this file. extern "C" for the plain-C callback signature
 * start_webserver() expects. */
extern "C" {
static void publish_port_cb_c(const char *process_name, int port)
{
	if (!g_publisher) {
		ULOGW("publish_port_cb: publisher=null");
		return;
	}
	std::string svc = process_name ? process_name : "unknown";
	std::thread([svc, port]() {
		for (int i = 0; i < 50 && g_publisher; ++i) {
			int rc = g_publisher->publishService(svc, (uint32_t)port);
			if (rc == 0) {
				ULOGI("Published %s on port %d", svc.c_str(),
				      port);
				return;
			}
			std::this_thread::sleep_for(
				std::chrono::milliseconds(100));
		}
		ULOGE("Failed to publish %s", svc.c_str());
	}).detach();
}
} // extern "C"

int main()
{
	signal(SIGTERM, sig_handler);
	signal(SIGINT, sig_handler);
	ULOGI("Starting image_classifier_ui_cpp mission");

	pomp::Loop loop;

	/* Own loop/thread: keeps publisher activity off the loop the NN client
	 * uses to unblock the worker thread's synchronous calls. */
	airsdk::publisher::Publisher publisher(UID);
	g_publisher = &publisher;

	Classifier classifier(loop);
	if (classifier.init() != 0) {
		ULOGE("failed to initialize classifier");
		return -1;
	}

	/* Registered before start_webserver(), so no client can race an empty command table. */
	register_command("set_native_nns_deactivated",
			 [&classifier](struct lws *, struct json_object *jobj) {
				 struct json_object *jval = nullptr;
				 if (!json_object_object_get_ex(
					     jobj, "value", &jval))
					 return -1;
				 classifier.deactivateNativeNns(
					 json_object_get_boolean(jval));
				 classifier.publishStatus();
				 return 0;
			 });
	register_command("set_inference_enabled",
			 [&classifier](struct lws *, struct json_object *jobj) {
				 struct json_object *jval = nullptr;
				 if (!json_object_object_get_ex(
					     jobj, "value", &jval))
					 return -1;
				 classifier.setInferEnabled(
					 json_object_get_boolean(jval));
				 classifier.publishStatus();
				 return 0;
			 });

	start_webserver("image_classifier_ui_cpp", publish_port_cb_c, "0.0.0.0", 0);

	/* Finite timeout so SIGTERM is noticed promptly. */
	while (run)
		loop.waitAndProcess(500);

	classifier.deactivateNativeNns(false);

	ULOGI("Stopping image_classifier_ui_cpp mission");
	stop_webserver();
	publisher.stopPublisher();
	g_publisher = nullptr;
	return 0;
}

/* Local processing functions */

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

/* NV12 sample -> normalized RGB float32 tensor, 224x224. Assumes NV12: the
 * fcam_airsdk stream's format; check sample.format if you use another source. */
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
