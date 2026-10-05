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
 */
#include "processing.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <fstream>

#include "mission_config.hpp"

#include <object_detector/detection.pb.h>

#define ULOG_TAG object_detector
#include <ulog.hpp>
ULOG_DECLARE_TAG(ULOG_TAG);

static const std::string STREAM_NAME = "fcam_airsdk";

namespace nninput = airsdk::neural_network::input;

// SSD model geometry: 8732 priors, 91 classes, per-box feature = 4 bbox +
// logits.
static constexpr int kNumBoxes = 8732;
static constexpr int kNumClasses = 91;
static constexpr int kFeatureSize = 4 + kNumClasses;
static constexpr int kMaxDetections = 10;
// Max serialized size of the per-frame detections metadata (bytes).
static constexpr uint16_t kMetaMaxSize = 2048;
// Stream user-metadata UID. Must NOT contain '.': telemetry section names with
// dots are rejected by libtelemetry consumers, so the daemon never embeds the
// metadata into the video stream. Hence a dotless UID, not MISSION_UID. The
// viewer hashes this same string (Jenkins) to match the vmeta user entry.
#define DETECTIONS_METADATA_UID "object_detector"
// Video-stream resource carrying the metadata. Use the AirSDK stream
// ("front-airsdk", vipc fcam_airsdk), the very stream inference runs on, so
// the boxes align with what the viewer displays. (See stream-server.cfg.)
#define STREAM_RESOURCE "front-airsdk"

void MetadataListener::onStreamUserMetadataList(
	std::string resource_path,
	std::vector<airsdk::metadata_publisher::StreamUserMetadata>
		user_metadata)
{
}

void MetadataListener::onRecordUserMetadataList(
	uint64_t camera_id,
	std::string track_name,
	std::vector<airsdk::metadata_publisher::RecordUserMetadata>
		user_metadata)
{
}

// Simple per-class NMS.
std::vector<Detection> Processing::nms(const std::vector<Box> &boxes,
				       const std::vector<float> &scores,
				       int class_id,
				       float iou_thresh,
				       int top_k)
{
	std::vector<int> indices(boxes.size());
	for (int i = 0; i < (int)boxes.size(); i++)
		indices[i] = i;

	// Sort by descending score.
	std::sort(indices.begin(), indices.end(), [&](int i1, int i2) {
		return scores[i1] > scores[i2];
	});

	std::vector<Detection> keep;

	std::vector<bool> suppressed(boxes.size(), false);
	for (size_t _i = 0; _i < indices.size(); ++_i) {
		int i = indices[_i];
		if (suppressed[i])
			continue;

		keep.push_back(Detection{class_id, scores[i], boxes[i]});

		if ((int)keep.size() >= top_k)
			break;

		for (size_t _j = _i + 1; _j < indices.size(); ++_j) {
			int j = indices[_j];
			if (suppressed[j])
				continue;
			float iou = computeIou(boxes[i], boxes[j]);
			if (iou > iou_thresh) {
				suppressed[j] = true;
			}
		}
	}

	return keep;
}

// Generate the fixed SSD priors, matching ssd300_vgg16's anchor generator.
// Cell centers use image_size/step, not 1/feature_size (the two differ
// slightly, e.g. step=8 gives 300/8=37.5 vs feature map size 38).
std::vector<Box> Processing::generate_ssd_priors()
{
	std::vector<Box> priors;
	const std::vector<int> feature_map_sizes = {38, 19, 10, 5, 3, 1};
	const std::vector<int> steps = {8, 16, 32, 64, 100, 300};
	const std::vector<std::vector<float>> aspect_ratios = {
		{2}, {2, 3}, {2, 3}, {2, 3}, {2}, {2}};
	const std::vector<float> scales = {
		0.07f, 0.15f, 0.33f, 0.51f, 0.69f, 0.87f, 1.05f};
	constexpr float kImageSize = 300.0f;

	auto clamp01 = [](float v) {
		return std::min(1.0f, std::max(0.0f, v));
	};

	for (size_t k = 0; k < feature_map_sizes.size(); ++k) {
		int f = feature_map_sizes[k];
		float f_k = kImageSize / steps[k];
		float scale = scales[k];
		float scale_next = scales[k + 1];
		float s_prime = clamp01(std::sqrt(scale * scale_next));
		float scale_c = clamp01(scale);

		for (int i = 0; i < f; ++i) {
			for (int j = 0; j < f; ++j) {
				float cx = (j + 0.5f) / f_k;
				float cy = (i + 0.5f) / f_k;

				// Square box.
				priors.push_back(Box{cx, cy, scale_c, scale_c});

				// Next scale box.
				priors.push_back(Box{cx, cy, s_prime, s_prime});

				// Aspect ratio boxes.
				for (float ar : aspect_ratios[k]) {
					float w =
						clamp01(scale * std::sqrt(ar));
					float h =
						clamp01(scale / std::sqrt(ar));
					priors.push_back(Box{cx, cy, w, h});
					priors.push_back(Box{cx, cy, h, w});
				}
			}
		}
	}

	return priors;
}

// Decode SSD boxes from location predictions and priors.
std::vector<Box> Processing::decode_boxes(const float *loc_data,
					  int num_boxes,
					  const std::vector<Box> &priors,
					  const std::vector<float> &variances)
{
	std::vector<Box> decoded_boxes(num_boxes);

	for (int i = 0; i < num_boxes; ++i) {
		float cx = priors[i].x1 +
			   loc_data[i * 4 + 0] * variances[0] * priors[i].x2;
		float cy = priors[i].y1 +
			   loc_data[i * 4 + 1] * variances[0] * priors[i].y2;
		float w = priors[i].x2 *
			  std::exp(loc_data[i * 4 + 2] * variances[1]);
		float h = priors[i].y2 *
			  std::exp(loc_data[i * 4 + 3] * variances[1]);

		decoded_boxes[i].x1 = cx - w / 2.f;
		decoded_boxes[i].y1 = cy - h / 2.f;
		decoded_boxes[i].x2 = cx + w / 2.f;
		decoded_boxes[i].y2 = cy + h / 2.f;
	}

	return decoded_boxes;
}

std::vector<Detection> Processing::ssd_postprocess(const float *loc_data,
						   const float *conf_data,
						   int num_boxes,
						   int num_classes,
						   float conf_thresh,
						   float iou_thresh,
						   int top_k)
{
	// Priors are fixed: compute them once and reuse.
	if (mPriors.empty())
		mPriors = generate_ssd_priors();
	const std::vector<float> variances = {0.1f, 0.2f};

	auto decoded_boxes =
		decode_boxes(loc_data, num_boxes, mPriors, variances);

	// A class can exceed conf_thresh only if it is the argmax, so find the
	// argmax logit first (no exp) and compute the softmax score only for
	// the few boxes whose argmax is not the background class.
	std::vector<std::vector<Box>> class_boxes(num_classes);
	std::vector<std::vector<float>> class_scores(num_classes);
	for (int i = 0; i < num_boxes; ++i) {
		const float *logits = conf_data + i * num_classes;
		int amax = 0;
		float mmax = logits[0];
		for (int c = 1; c < num_classes; ++c) {
			if (logits[c] > mmax) {
				mmax = logits[c];
				amax = c;
			}
		}
		if (amax == 0)
			continue; // background dominates: no class can exceed
				  // the threshold

		float sum = 0.f;
		for (int c = 0; c < num_classes; ++c)
			sum += std::exp(logits[c] - mmax);
		float score = 1.f / sum; // softmax prob of the argmax class

		if (score > conf_thresh) {
			class_boxes[amax].push_back(decoded_boxes[i]);
			class_scores[amax].push_back(score);
		}
	}

	std::vector<Detection> results;
	for (int c = 1; c < num_classes; ++c) {
		if (class_boxes[c].empty())
			continue;
		auto kept = nms(
			class_boxes[c], class_scores[c], c, iou_thresh, top_k);
		results.insert(results.end(), kept.begin(), kept.end());
	}

	return results;
}

Processing::~Processing()
{
	mSource.reset(); // stop frames before tearing down the network
	delete mNeuralNetwork;
	delete mMetadataPublisher;
	delete mMetadataListener;
	delete mLoop;
}

int Processing::init()
{
	mLoop = new pomp::Loop();

	// Class names are fixed: load them once, up front.
	loadClassNames();

	// Create the network on the mission loop; readiness, errors and
	// inference results are delivered to this object (as the listener) on
	// that loop.
	airsdk::neural_network::Settings settings;
	settings.netId = "airsdk@object_detector";
	settings.modelFile = MISSION_CFG_DIR "/ssd_quantized.dlc";
	// SNPE *layer* names (snpe-dlc-info's "Name" column), one per output
	// tensor: bbox regression, then class logits.
	settings.outputLayers = {"/Unsqueeze", "/Unsqueeze_1"};
	// settings.diagLogFolder = MISSION_CFG_DIR; // debug only: adds
	// per-inference I/O overhead
	ULOGI("initializing neural network (model: %s)",
	      settings.modelFile.c_str());
	mNeuralNetwork = new airsdk::neural_network::NeuralNetwork(
		&settings, this, *mLoop, 30000);

	mMetadataListener = new MetadataListener();
	mMetadataPublisher = new airsdk::metadata_publisher::MetadataPublisher(
		MISSION_UID, *mLoop, mMetadataListener);

	return 0;
}

int Processing::start()
{
	// Loop-native: the SDK input source delivers each fresh frame to
	// onSample() on mLoop (with reconnection + stall recovery handled by
	// the SDK). Inference is fired there and completes in onInferResult().
	nninput::VipcSettings vs;
	vs.stream = STREAM_NAME;
	mSource = nninput::makeVipcSource(*mLoop, vs);
	if (!mSource) {
		ULOGE("failed to create vipc input source");
		return -ENOMEM;
	}
	mSource->setSampleCallback(
		[this](std::shared_ptr<const nninput::Sample> s) {
			onSample(std::move(s));
		});
	return 0;
}

void Processing::stop()
{
	/* Drop the source: no more frames means no more inference is fired. */
	mSource.reset();
	// Re-enable native NNs on shutdown, or the drone stays degraded.
	if (mNeuralNetwork)
		mNeuralNetwork->deactivateNativeNns(false);
}

void Processing::run()
{
	/* Run loop until asked to exit */
	while (!mExitRequested) {
		mLoop->waitAndProcess(-1);
	}
}

void Processing::requestExit()
{
	/* Set the exit flag and wake up the loop (safe to call from a signal).
	 */
	mExitRequested = true;
	if (mLoop)
		mLoop->wakeup();
}

void Processing::onSample(std::shared_ptr<const nninput::Sample> sample)
{
	// Drop the frame if the network is not ready, or if a previous
	// inference is still in flight (infer returns -EBUSY, one inference at
	// a time).
	if (!mNnReady)
		return;

	cv::Mat preprocessedFrame = preProcessFrame(*sample);
	if (preprocessedFrame.empty())
		return;

	airsdk::neural_network::InferInput input{
		airsdk::neural_network::BlobAttributes(
			"input",
			"input",
			1,
			preprocessedFrame.rows,
			preprocessedFrame.cols,
			preprocessedFrame.channels()),
		preprocessedFrame.data,
		(size_t)preprocessedFrame.rows * preprocessedFrame.cols *
			preprocessedFrame.elemSize()};

	// infer copies the input synchronously, so the local Mat can be freed
	// right after; the result comes back on the loop in onInferResult().
	int ret = mNeuralNetwork->infer(
		{input},
		(uint32_t)(kNumBoxes * kFeatureSize * sizeof(float)),
		sample->timestampNs);
	if (ret < 0 && ret != -EBUSY)
		ULOGW("infer failed: %d", ret);
}

static cv::Mat matFromPlane(const nninput::Sample &sample,
			    int planeIndex,
			    int height,
			    int width)
{
	return cv::Mat(height,
		       width,
		       CV_8UC1,
		       (void *)sample.planeData(planeIndex),
		       sample.planes[planeIndex].stride);
}

// Convert an NV12 sample to a 300x300 RGB float32 image (no normalization).
cv::Mat Processing::preProcessFrame(const nninput::Sample &sample)
{
	if (sample.planes.size() < 2 || sample.width == 0 ||
	    sample.height == 0) {
		ULOGE("preProcessFrame: unexpected sample (planes=%zu %ux%u)",
		      sample.planes.size(),
		      sample.width,
		      sample.height);
		return cv::Mat();
	}

	const uint32_t w = sample.width, h = sample.height;

	// NV12 = Y (h x stride) then interleaved UV (h/2 x stride). If the
	// planes are tightly packed (stride == width) the sample bytes are
	// already a valid NV12 buffer we can wrap directly; otherwise repack to
	// drop the row padding.
	cv::Mat nv12;
	if (sample.planes[0].stride == w && sample.planes[1].stride == w &&
	    sample.planes[1].offset == (size_t)w * h) {
		nv12 = cv::Mat(h + h / 2, w, CV_8UC1, (void *)sample.data());
	} else {
		nv12.create(h + h / 2, w, CV_8UC1);
		matFromPlane(sample, 0, h, w)
			.copyTo(nv12(cv::Rect(0, 0, w, h)));
		matFromPlane(sample, 1, h / 2, w)
			.copyTo(nv12(cv::Rect(0, h, w, h / 2)));
	}

	// NV12 -> RGB in one pass, resize to 300x300, to float32 (no
	// normalization).
	cv::Mat rgb;
	cv::cvtColor(nv12, rgb, cv::COLOR_YUV2RGB_NV12);
	cv::Mat resized;
	cv::resize(rgb, resized, cv::Size(300, 300));
	cv::Mat floatImage;
	resized.convertTo(floatImage, CV_32FC3);

	return floatImage;
}

void Processing::loadClassNames()
{
	std::string classesNamesFilePath(MISSION_CFG_DIR "/classes_names.csv");
	std::ifstream classesNamesFile(classesNamesFilePath);
	if (!classesNamesFile.is_open()) {
		ULOGW("failed to read classes names file %s",
		      classesNamesFilePath.c_str());
		return;
	}
	std::string className;
	while (std::getline(classesNamesFile, className))
		mClassesNames.push_back(className);
	ULOGI("loaded %zu class names", mClassesNames.size());
}

void Processing::onStatus(airsdk::neural_network::NeuralNetwork *neuralNetwork,
			  bool isReady)
{
	mNnReady = isReady;
	ULOGI("neural network %s", isReady ? "ready" : "not ready");
	// Take the NPU: deactivate the drone's native networks (visual
	// tracking, hand-land, precise home) so our SSD runs at full rate.
	if (isReady)
		neuralNetwork->deactivateNativeNns(true);
}

void Processing::onError(airsdk::neural_network::NeuralNetwork *neuralNetwork,
			 std::string reason)
{
	ULOGE("neural network error: %s", reason.c_str());
}

void Processing::onInferError(
	airsdk::neural_network::NeuralNetwork *neuralNetwork,
	int error,
	uint64_t timestamp_ns)
{
	ULOGW("inference failed: %d", error);
}

void Processing::onInferResult(
	airsdk::neural_network::NeuralNetwork *neuralNetwork,
	std::vector<airsdk::neural_network::Blob> &&outputs,
	uint64_t timestamp_ns)
{
	if (outputs.size() != 2)
		return;

	// bbox_regression: [1, kNumBoxes, 4, 1]. cls_logits: [1, kNumBoxes,
	// kNumClasses, 1].
	const airsdk::neural_network::Blob *loc_blob = nullptr;
	const airsdk::neural_network::Blob *conf_blob = nullptr;
	for (const auto &blob : outputs) {
		if (blob.attributes().dataName == "bbox_regression")
			loc_blob = &blob;
		else if (blob.attributes().dataName == "cls_logits")
			conf_blob = &blob;
	}
	if (!loc_blob || !conf_blob) {
		ULOGW("unexpected output tensors, skipping frame");
		return;
	}

	const auto &loc_attrs = loc_blob->attributes();
	const auto &conf_attrs = conf_blob->attributes();
	if (loc_attrs.num != 1 || loc_attrs.height != kNumBoxes ||
	    loc_attrs.width != 4 || loc_attrs.channel != 1 ||
	    conf_attrs.num != 1 || conf_attrs.height != kNumBoxes ||
	    conf_attrs.width != kNumClasses || conf_attrs.channel != 1) {
		ULOGW("unexpected output tensor shape, skipping frame");
		return;
	}

	const float *loc_data = static_cast<const float *>(loc_blob->ptr());
	const float *conf_data = static_cast<const float *>(conf_blob->ptr());
	std::vector<Detection> detections =
		ssd_postprocess(loc_data, conf_data, kNumBoxes, kNumClasses);

	publishDetections(detections);
}

void Processing::publishDetections(const std::vector<Detection> &detections)
{
	// Register our single stream-metadata entry once, on the first
	// detection: the publisher is not yet connected to the daemon at
	// init().
	if (!mMetaRegistered) {
		mMetadataPublisher->addStreamUserMetadata(
			DETECTIONS_METADATA_UID, kMetaMaxSize, STREAM_RESOURCE);
		mMetaRegistered = true;
		ULOGI("registered stream user metadata '%s'",
		      DETECTIONS_METADATA_UID);
	}

	// Serialize every detection into one protobuf message, keyed by the
	// mission UID. The pdraw viewer reads it back from the frame's vmeta
	// user metadata.
	ObjectDetector::Metadata::Detections proto;
	std::string summary;
	int n = 0;
	for (size_t i = 0; i < detections.size(); ++i) {
		if (n >= kMaxDetections)
			break;
		const Detection &det = detections[i];

		const std::string label =
			(det.class_id >= 0 &&
			 (size_t)det.class_id < mClassesNames.size())
				? mClassesNames[det.class_id]
				: std::to_string(det.class_id);
		summary += label + "(" +
			   std::to_string((int)(det.score * 100.f)) + "%) ";

		auto *d = proto.add_detections();
		d->set_class_id((uint32_t)det.class_id);
		d->set_score(det.score);
		d->set_label(label);
		auto *b = d->mutable_box();
		b->set_left(det.box.x1);
		b->set_top(det.box.y1);
		b->set_right(det.box.x2);
		b->set_bottom(det.box.y2);
		++n;
	}

	// One push per frame (an empty message when nothing is detected, so the
	// viewer clears its boxes).
	std::string bytes;
	proto.SerializeToString(&bytes);
	int res = mMetadataPublisher->push(
		DETECTIONS_METADATA_UID,
		reinterpret_cast<const uint8_t *>(bytes.data()),
		(uint32_t)bytes.size());
	if (res < 0)
		ULOGW("metadata push failed: %d", res);

	if (n > 0)
		ULOGI("detected %d object(s): %s", n, summary.c_str());
}
