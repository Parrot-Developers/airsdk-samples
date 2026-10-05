/**
 * @file player.hpp
 * @brief Audio file player using PDrAW pipeline.
 *
 * Implements an audio playback pipeline that opens a local audio file via a
 * PDrAW demuxer and renders the decoded audio through a PDrAW audio renderer.
 * The lifecycle follows an asynchronous step-by-step state machine driven by
 * pomp idle handlers; the shared PDrAW plumbing lives in PdrawPipeline.
 */

#pragma once

#include "pdraw_pipeline.hpp"

/**
 * @class Player
 * @brief Asynchronous audio file player backed by PDrAW.
 *
 * Inherits privately from the demuxer and audio-renderer listener interfaces
 * so that internal callbacks are not part of the public API. The PDrAW
 * instance lifecycle and step scheduling come from PdrawPipeline.
 *
 * Typical usage:
 * @code
 *   Player player(loop);
 *   player.start("/path/to/audio.mp4");
 *   // ...
 *   player.stop();
 * @endcode
 */
class Player : public PdrawPipeline,
	       private Pdraw::IPdraw::IDemuxer::Listener,
	       private Pdraw::IPdraw::IAudioRenderer::Listener {
public:
	/**
	 * @brief Construct a Player attached to the given event loop.
	 * @param loop pomp event loop used to schedule asynchronous steps.
	 */
	Player(pomp::Loop &loop) : PdrawPipeline(loop) {}

	/**
	 * @brief Start playback of an audio file.
	 *
	 * Initiates the asynchronous startup sequence:
	 * create PDrAW → open demuxer → play → create renderer → seek to t=0.
	 *
	 * The seek ensures the renderer, created after the demuxer starts
	 * playing, receives all frames from the beginning of the file.
	 *
	 * @param file_path Absolute or relative path to the audio file to play.
	 * @return 0 on success, negative errno on failure.
	 */
	int start(const std::string &file_path);

	/**
	 * @brief Stop playback and release all resources.
	 *
	 * Initiates the asynchronous teardown sequence:
	 * delete renderer → close demuxer → delete demuxer → stop PDrAW.
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
		STEP_CREATE_PDRAW,    /**< Create the PDrAW instance. */
		STEP_CREATE_DEMUXER,  /**< Open the file demuxer. */
		STEP_PLAY_DEMUXER,    /**< Start demuxer playback. */
		STEP_CREATE_RENDERER, /**< Create the audio renderer. */
		STEP_SEEK_START, /**< Seek to t=0 so renderer receives all frames. */
	};

	/**
	 * @brief Ordered steps executed during the teardown sequence.
	 */
	enum STOP_STEP {
		STEP_DELETE_RENDERER, /**< Destroy the audio renderer. */
		STEP_CLOSE_DEMUXER,   /**< Close the demuxer. */
		STEP_DELETE_DEMUXER,  /**< Destroy the demuxer object. */
		STEP_STOP_PDRAW, /**< Stop and delete the PDrAW instance. */
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
	 * @brief Schedule renderer creation once the PCM media is available.
	 * @param encoding Encoding of the added audio media.
	 * @param media_id Pipeline id of the added media.
	 */
	void onAudioMediaAdded(enum adef_encoding encoding,
			       unsigned int media_id) override;

	// -------------------------------------------------------------------------
	// IDemuxer::Listener callbacks
	// -------------------------------------------------------------------------

	void demuxerOpenResponse(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 int status) override
	{
	}

	/**
	 * @brief Called when the demuxer close operation has completed;
	 * advances the teardown sequence.
	 */
	void demuxerCloseResponse(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IDemuxer *demuxer,
				  int status) override;

	void onDemuxerUnrecoverableError(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IDemuxer *demuxer) override
	{
	}

	/**
	 * @brief Called to let the application select which media tracks to
	 * demux.
	 * @return Bitmask of selected media indices (0 = default selection).
	 */
	int demuxerSelectMedia(Pdraw::IPdraw *pdraw,
			       Pdraw::IPdraw::IDemuxer *demuxer,
			       const struct pdraw_demuxer_media *medias,
			       size_t count,
			       uint32_t selectedMedias) override;

	/**
	 * @brief Called when the demuxer ready-to-play state changes; starts
	 * playback once ready.
	 */
	void demuxerReadyToPlay(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IDemuxer *demuxer,
				bool ready) override;

	/**
	 * @brief Called when the demuxer has reached the end of the media
	 * range.
	 */
	void onDemuxerEndOfRange(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 uint64_t timestamp) override;

	/**
	 * @brief Called when the demuxer play operation has completed.
	 */
	void demuxerPlayResponse(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed) override;

	void demuxerPauseResponse(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IDemuxer *demuxer,
				  int status,
				  uint64_t timestamp) override
	{
	}

	void demuxerSeekResponse(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed) override
	{
	}

	// -------------------------------------------------------------------------
	// IAudioRenderer::Listener callbacks
	// -------------------------------------------------------------------------

	void onAudioRendererMediaAdded(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IAudioRenderer *renderer,
		const struct pdraw_media_info *info) override
	{
	}

	void onAudioRendererMediaRemoved(
		Pdraw::IPdraw *pdraw,
		Pdraw::IPdraw::IAudioRenderer *renderer,
		const struct pdraw_media_info *info) override
	{
	}

	// -------------------------------------------------------------------------
	// Player-specific members
	// -------------------------------------------------------------------------

	std::string m_file_path; /**< Path to the audio file being played. */

	std::unique_ptr<Pdraw::IPdraw::IDemuxer>
		m_demuxer; /**< File demuxer. */

	std::unique_ptr<Pdraw::IPdraw::IAudioRenderer>
		m_renderer; /**< Audio renderer. */

	static const struct pdraw_demuxer_params
		demuxer_params; /**< Static demuxer configuration. */

	static const struct pdraw_audio_renderer_params
		renderer_params; /**< Static renderer configuration. */
};
