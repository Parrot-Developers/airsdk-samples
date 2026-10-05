/**
 * @file recorder.hpp
 * @brief Audio recorder using PDrAW pipeline.
 *
 * Implements an audio capture and encoding pipeline that reads PCM audio from
 * an ALSA source, encodes it via a PDrAW audio encoder, and writes the result
 * to a file through a PDrAW muxer.
 * The lifecycle follows an asynchronous step-by-step state machine driven by
 * pomp idle handlers; the shared PDrAW plumbing lives in PdrawPipeline.
 */

#pragma once

#include "pdraw_pipeline.hpp"

/**
 * @class Recorder
 * @brief Asynchronous audio recorder backed by PDrAW.
 *
 * Inherits privately from the ALSA source, audio encoder and muxer listener
 * interfaces so that internal callbacks are not part of the public API. The
 * PDrAW instance lifecycle and step scheduling come from PdrawPipeline.
 *
 * Typical usage:
 * @code
 *   Recorder recorder(loop);
 *   recorder.start("/path/to/output.mp4");
 *   // ...
 *   recorder.stop();
 * @endcode
 */
class Recorder : public PdrawPipeline,
		 private Pdraw::IPdraw::IAlsaSource::Listener,
		 private Pdraw::IPdraw::IAudioEncoder::Listener,
		 private Pdraw::IPdraw::IMuxer::Listener {
public:
	/**
	 * @brief Construct a Recorder attached to the given event loop.
	 * @param loop pomp event loop used to schedule asynchronous steps.
	 */
	Recorder(pomp::Loop &loop) : PdrawPipeline(loop) {}

	/**
	 * @brief Start recording to an audio file.
	 *
	 * Initiates the asynchronous startup sequence:
	 * create PDrAW → create ALSA source → start source →
	 * create encoder → create muxer.
	 *
	 * @param file_path Absolute or relative path to the output file.
	 * @return 0 on success, negative errno on failure.
	 */
	int start(const std::string &file_path);

	/**
	 * @brief Stop recording and release all resources.
	 *
	 * Initiates the asynchronous teardown sequence:
	 * pause source → delete source → delete encoder →
	 * close muxer → delete muxer → stop PDrAW.
	 *
	 * @return 0 on success, negative errno on failure.
	 */
	int stop();

private:
	// -------------------------------------------------------------------------
	// Internal state machine
	// -------------------------------------------------------------------------

	/**
	 * @brief Ordered steps executed during the startup sequence.
	 */
	enum START_STEP {
		STEP_CREATE_PDRAW,   /**< Create the PDrAW instance. */
		STEP_CREATE_SOURCE,  /**< Create the ALSA audio source. */
		STEP_PLAY_SOURCE,    /**< Start the ALSA source capture. */
		STEP_CREATE_ENCODER, /**< Create the audio encoder. */
		STEP_CREATE_MUXER,   /**< Create the file muxer. */
	};

	/**
	 * @brief Ordered steps executed during the teardown sequence.
	 */
	enum STOP_STEP {
		STEP_PAUSE_SOURCE,   /**< Pause the ALSA source capture. */
		STEP_DELETE_SOURCE,  /**< Destroy the ALSA source object. */
		STEP_DELETE_ENCODER, /**< Destroy the encoder object. */
		STEP_CLOSE_MUXER,    /**< Flush and close the muxer. */
		STEP_DELETE_MUXER,   /**< Destroy the muxer object. */
		STEP_STOP_PDRAW,     /**< Stop and delete the PDrAW instance. */
	};

	/**
	 * @brief Dispatch a scheduled step to the matching sequence.
	 *
	 * The current state disambiguates whether @p step belongs to the
	 * startup or teardown enum.
	 *
	 * @param step Raw step value previously passed to scheduleStep().
	 */
	void runStep(int step) override
	{
		if (m_state == State::Stopping)
			doStopStep(static_cast<STOP_STEP>(step));
		else
			doStartStep(static_cast<START_STEP>(step));
	}

	/**
	 * @brief Execute a startup step synchronously.
	 * @param step Step to execute.
	 */
	void doStartStep(enum START_STEP step);

	/**
	 * @brief Execute a teardown step synchronously.
	 * @param step Step to execute.
	 */
	void doStopStep(enum STOP_STEP step);

	/**
	 * @brief Advance the startup sequence as audio medias appear.
	 *
	 * PCM media triggers encoder creation; AAC media triggers muxer
	 * creation (its id is retained for the muxer track).
	 *
	 * @param encoding Encoding of the added audio media.
	 * @param media_id Pipeline id of the added media.
	 */
	void onAudioMediaAdded(enum adef_encoding encoding,
			       unsigned int media_id) override;

	// -------------------------------------------------------------------------
	// IAlsaSource::Listener callbacks
	// -------------------------------------------------------------------------

	/**
	 * @brief Called when the ALSA source ready-to-play state changes;
	 * starts capture once ready.
	 */
	void alsaSourceReadyToPlay(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IAlsaSource *source,
		bool ready,
		enum pdraw_alsa_source_eos_reason eosReason) override;

	/**
	 * @brief Called when the ALSA source play operation has completed.
	 */
	void alsaSourcePlayResponse(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IAlsaSource *source) override;

	/**
	 * @brief Called when the ALSA source pause operation has completed;
	 * advances the teardown sequence.
	 */
	void alsaSourcePauseResponse(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IAlsaSource *source) override;

	void alsaSourceFrameReady(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IAlsaSource *source,
				  struct mbuf_audio_frame *frame) override
	{
	}

	// -------------------------------------------------------------------------
	// IAudioEncoder::Listener callbacks
	// -------------------------------------------------------------------------

	void audioEncoderFrameOutput(Pdraw::IPdraw *pdraw,
				     Pdraw::IPdraw::IAudioEncoder *encoder,
				     struct mbuf_audio_frame *frame) override
	{
	}

	void audioEncoderFramePreRelease(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IAudioEncoder *encoder,
		struct mbuf_audio_frame *frame) override
	{
	}

	// -------------------------------------------------------------------------
	// IMuxer::Listener callbacks
	// -------------------------------------------------------------------------

	void onMuxerConnectionStateChanged(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IMuxer *muxer,
		enum pdraw_muxer_connection_state connectionState,
		enum pdraw_muxer_disconnection_reason disconnectionReason)
		override
	{
	}

	void onMuxerMediaReady(Pdraw::IPdraw *pdraw,
			       Pdraw::IPdraw::IMuxer *muxer,
			       const char *mediaPath,
			       const struct iovec *iov,
			       int iovcnt) override
	{
	}

	void onMuxerMediaSaved(Pdraw::IPdraw *pdraw,
			       Pdraw::IPdraw::IMuxer *muxer,
			       const char *mediaPath) override
	{
	}

	void onMuxerUnrecoverableError(Pdraw::IPdraw *pdraw,
				       Pdraw::IPdraw::IMuxer *muxer,
				       int status) override
	{
	}

	/**
	 * @brief Called when the muxer close operation has completed; advances
	 * the teardown sequence.
	 */
	void muxerCloseResponse(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IMuxer *muxer,
				int status) override;

	// -------------------------------------------------------------------------
	// Recorder-specific members
	// -------------------------------------------------------------------------

	std::string m_file_path; /**< Path to the output audio file. */

	unsigned int m_aac_media_id = 0; /**< Id of the AAC audio media. */

	std::unique_ptr<Pdraw::IPdraw::IAlsaSource>
		m_source; /**< ALSA audio capture source. */

	std::unique_ptr<Pdraw::IPdraw::IAudioEncoder>
		m_encoder; /**< Audio encoder. */

	std::unique_ptr<Pdraw::IPdraw::IMuxer> m_muxer; /**< File muxer. */

	static const struct adef_format
		source_format; /**< PCM format used by the ALSA source. */

	static const struct pdraw_alsa_source_params
		source_params; /**< Static ALSA source configuration. */

	static const struct aenc_config
		encoder_config; /**< Static encoder configuration. */

	static const struct pdraw_muxer_params
		muxer_params; /**< Static muxer configuration. */

	static const struct pdraw_muxer_media_params
		muxer_media_params; /**< Static muxer media-track configuration.
				     */
};
