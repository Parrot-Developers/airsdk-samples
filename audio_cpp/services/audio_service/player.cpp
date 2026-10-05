#include "player.hpp"

#define ULOG_TAG player
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include <cinttypes>
#include <chrono>

const struct pdraw_demuxer_params Player::demuxer_params = {
	.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL,
	.playback_mode = PDRAW_PLAYBACK_MODE_REALTIME,
};
const struct pdraw_audio_renderer_params Player::renderer_params = {
	.address = "plughw:1", // using plughw for maximum compatibility
};

int Player::start(const std::string &file_path)
{
	m_file_path = file_path;
	m_error = 0;
	m_state = State::Starting;
	int ret = scheduleStep(STEP_CREATE_PDRAW);
	if (ret < 0)
		failStep("scheduleStep", ret);
	return ret;
}

void Player::doStartStep(enum START_STEP step)
{
	switch (step) {
	case STEP_CREATE_PDRAW: {
		int ret = createPdrawInstance();
		if (ret < 0)
			return failStep("createPdrawInstance", ret);

		ret = scheduleStep(STEP_CREATE_DEMUXER);
		if (ret < 0)
			return failStep("scheduleStep", ret);
		break;
	}

	case STEP_CREATE_DEMUXER: {
		// Creating demuxer
		Pdraw::IPdraw::IDemuxer *demuxer;
		int ret = m_pdraw->createDemuxer(
			m_file_path, &demuxer_params, this, &demuxer);
		if (ret < 0)
			return failStep("IPdraw::createDemuxer", ret);
		ULOGI("IDemuxer created for file: %s", m_file_path.c_str());
		m_demuxer.reset(demuxer);

		// next step called by demuxerReadyToPlay
		break;
	}

	case STEP_PLAY_DEMUXER: {
		// Starting demuxer
		ULOGI("IDemuxer PLAY");
		int ret = m_demuxer->play();
		if (ret < 0)
			return failStep("IDemuxer::play", ret);

		// next step called by onMediaAdded
		break;
	}

	case STEP_CREATE_RENDERER: {
		// Creating Renderer
		Pdraw::IPdraw::IAudioRenderer *renderer;
		int ret = m_pdraw->createAudioRenderer(
			m_pcm_media_id, &renderer_params, this, &renderer);
		if (ret < 0)
			return failStep("IPdraw::createAudioRenderer", ret);
		ULOGI("IAudioRenderer created for media %d on %s",
		      m_pcm_media_id,
		      renderer_params.address);
		m_renderer.reset(renderer);

		// Seek to t=0 before declaring Running: the renderer was created
		// after play() started, so some frames were decoded without a
		// consumer. Seeking back ensures the renderer receives the full
		// recording from the beginning.
		ret = scheduleStep(STEP_SEEK_START);
		if (ret < 0)
			return failStep("scheduleStep", ret);
		break;
	}

	case STEP_SEEK_START: {
		ULOGI("IDemuxer seeking to start");
		int ret = m_demuxer->seekTo(0, true);
		if (ret < 0)
			return failStep("IDemuxer::seekTo", ret);

		// end of start procedure
		m_state = State::Running;
		break;
	}
	}
}

int Player::stop()
{
	m_state = State::Stopping;
	return scheduleStopStep(STEP_DELETE_RENDERER);
}

void Player::doStopStep(enum STOP_STEP step)
{
	switch (step) {
	case STEP_DELETE_RENDERER: {
		if (m_renderer) {
			// Deleting renderer
			m_renderer.reset();
			ULOGI("IAudioRenderer removed");
		}

		scheduleStopStep(STEP_CLOSE_DEMUXER);
		break;
	}
	case STEP_CLOSE_DEMUXER: {
		if (m_demuxer) {
			// Closing demuxer
			ULOGI("IDemuxer CLOSE");
			int ret = m_demuxer->close();
			if (ret == 0) {
				// next step called by demuxerCloseResponse
				break;
			}
			// Keep tearing down: the sequence must reach its end
			failStep("IDemuxer::close", ret);
		}

		scheduleStopStep(STEP_DELETE_DEMUXER);
		break;
	}
	case STEP_DELETE_DEMUXER: {
		if (m_demuxer) {
			// Deleting demuxer
			m_demuxer.reset();
			ULOGI("IDemuxer removed");
		}

		scheduleStopStep(STEP_STOP_PDRAW);
		break;
	}
	case STEP_STOP_PDRAW:
		// Stop and delete the PDrAW instance (handled by the base);
		// end of stop procedure
		requestPdrawStop();
		break;
	}
}

void Player::onAudioMediaAdded(enum adef_encoding encoding,
			       unsigned int media_id)
{
	// Waiting for the PCM audio media to create the renderer
	if (encoding == ADEF_ENCODING_PCM) {
		int ret = scheduleStep(STEP_CREATE_RENDERER);
		if (ret < 0)
			return failStep("scheduleStep", ret);
	}
}

// IDemuxer::Listener callbacks
void Player::demuxerCloseResponse(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IDemuxer *demuxer,
				  int status)
{
	if (status != 0) {
		// Keep tearing down, the demuxer still has to be deleted
		failStep("IDemuxer close", status);
	} else {
		ULOGI("IDemuxer closed");
	}

	scheduleStopStep(STEP_DELETE_DEMUXER);
}

int Player::demuxerSelectMedia(Pdraw::IPdraw *pdraw,
			       Pdraw::IPdraw::IDemuxer *demuxer,
			       const struct pdraw_demuxer_media *medias,
			       size_t count,
			       uint32_t selectedMedias)
{
	return 0; // Default media
}

void Player::demuxerReadyToPlay(Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IDemuxer *demuxer,
				bool ready)
{
	if (ready) {
		ULOGI("IDemuxer ready to play");
		int ret = scheduleStep(STEP_PLAY_DEMUXER);
		if (ret < 0)
			return failStep("scheduleStep", ret);
	}
}

void Player::onDemuxerEndOfRange(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 uint64_t timestamp)
{
	ULOGI("Playback ended after %" PRId64 "ms",
	      std::chrono::duration_cast<std::chrono::milliseconds>(
		      std::chrono::microseconds(timestamp))
		      .count());
}

void Player::demuxerPlayResponse(Pdraw::IPdraw *pdraw,
				 Pdraw::IPdraw::IDemuxer *demuxer,
				 int status,
				 uint64_t timestamp,
				 float speed)
{
	ULOGI("IDemuxer playing");
}
