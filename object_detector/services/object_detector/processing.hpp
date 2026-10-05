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
#pragma once

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <airsdk/airsdk.hpp>
#include <airsdk/neural_network.hpp>
#include <airsdk/neural_network/input.hpp>
#include <libpomp.hpp>

#include "types.hpp"

// Metadata publisher listener (no-op callbacks, required by the SDK).
class MetadataListener
		: public airsdk::metadata_publisher::MetadataPublisherListener {
private:
	virtual void onStreamUserMetadataList(
		std::string resource_path,
		std::vector<airsdk::metadata_publisher::StreamUserMetadata>
			user_metadata) override;

	virtual void onRecordUserMetadataList(
		uint64_t camera_id,
		std::string track_name,
		std::vector<airsdk::metadata_publisher::RecordUserMetadata>
			user_metadata) override;
};

// Core class: everything runs on a single pomp loop (no worker thread).
// onSample preprocesses and fires a non-blocking inference; the result comes
// back on the same loop via onInferResult, where it is post-processed.
class Processing : public airsdk::neural_network::NeuralNetworkListener {
public:
	~Processing();

	int init();

	int start();
	void stop();

	void run();
	void requestExit();

	// Fresh camera frame, delivered on mLoop by the input source.
	void
	onSample(std::shared_ptr<const airsdk::neural_network::input::Sample>
			 sample);

	// Neural-network events (on mLoop).
	void onStatus(airsdk::neural_network::NeuralNetwork *neuralNetwork,
		      bool isReady) override;
	void onError(airsdk::neural_network::NeuralNetwork *neuralNetwork,
		     std::string reason) override;
	void onInferResult(airsdk::neural_network::NeuralNetwork *neuralNetwork,
			   std::vector<airsdk::neural_network::Blob> &&outputs,
			   uint64_t timestamp_ns) override;
	void onInferError(airsdk::neural_network::NeuralNetwork *neuralNetwork,
			  int error,
			  uint64_t timestamp_ns) override;

private:
	cv::Mat
	preProcessFrame(const airsdk::neural_network::input::Sample &sample);
	void loadClassNames();
	void publishDetections(const std::vector<Detection> &detections);

	// loc_data: num_boxes * 4 bbox regressions. conf_data: num_boxes *
	// num_classes logits.
	std::vector<Detection> ssd_postprocess(const float *loc_data,
					       const float *conf_data,
					       int num_boxes,
					       int num_classes,
					       float conf_thresh = 0.5f,
					       float iou_thresh = 0.45f,
					       int top_k = 200);
	std::vector<Detection> nms(const std::vector<Box> &boxes,
				   const std::vector<float> &scores,
				   int class_id,
				   float iou_thresh,
				   int top_k);
	std::vector<Box> generate_ssd_priors();

	std::vector<Box>
	decode_boxes(const float *loc_data,
		     int num_boxes,
		     const std::vector<Box> &priors,
		     const std::vector<float> &variances = {0.1f, 0.2f});

private:
	volatile bool mExitRequested = false;
	pomp::Loop *mLoop = nullptr;
	std::unique_ptr<airsdk::neural_network::input::Source> mSource;
	airsdk::neural_network::NeuralNetwork *mNeuralNetwork = nullptr;
	bool mNnReady = false; // set on the loop by onStatus
	std::vector<Box> mPriors; // SSD priors, computed once and reused
	std::vector<std::string> mClassesNames; // loaded once at init
	// Stream metadata entry registered lazily on the first detection (the
	// publisher is not yet connected at init).
	bool mMetaRegistered = false;

	MetadataListener *mMetadataListener = nullptr;
	airsdk::metadata_publisher::MetadataPublisher *mMetadataPublisher =
		nullptr;
};
