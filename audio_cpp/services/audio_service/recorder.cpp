#include "recorder.hpp"

#define ULOG_TAG recorder
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

const struct adef_format Recorder::source_format = {
	.encoding = ADEF_ENCODING_PCM,
	.channel_count = 2,
	.bit_depth = 16,
	.sample_rate = 44100,
	.pcm =
		{
			.interleaved = true,
			.signed_val = true,
			.little_endian = true,
		},
	.aac =
		{
			.data_format = ADEF_AAC_DATA_FORMAT_UNKNOWN,
		},
};

const struct pdraw_alsa_source_params Recorder::source_params = {
	.address = "hw:1",
	.sample_count = 1024,
	.audio =
		{
			.format = source_format,
			.aac_lc =
				{
					.asc = {0},
					.asclen = 0,
				},
		},
};

const struct aenc_config Recorder::encoder_config = {
	.name = NULL,
	.device = NULL,
	.implem = AENC_ENCODER_IMPLEM_FDK_AAC,
	.preferred_thread_count = 0,
	.preferred_max_frames_in_encoder = 0,
	.preferred_frame_length = 1024,
	.encoding = ADEF_ENCODING_AAC_LC,
	.input =
		{
			.preferred_min_buf_count = 0,
			.format = source_format,
		},
	.output =
		{
			.preferred_min_buf_count = 0,
			.preferred_format = ADEF_AAC_DATA_FORMAT_RAW,
		},
	.aac_lc =
		{
			.rate_control = AENC_RATE_CONTROL_CBR,
			.max_bitrate = 320000,
			.target_bitrate = 0,
		},
	.implem_cfg = NULL,
};

const struct pdraw_muxer_params Recorder::muxer_params = {
	.filemode = 0,
	.free_space_limit = 0,
	.tables_size_mb = 0,
	.recovery =
		{
			.tables_file = "",
			.sync_period_ms = 0,
			.check_storage_uuid = false,
			.allocate_space_for_tables_file = false,
		},
	.tables_sync_period_ms = 0,
	.initial_file_index = 0,
	.rtsp_transport = PDRAW_MUXER_RTSP_TRANSPORT_UDP,
};

const struct pdraw_muxer_media_params Recorder::muxer_media_params = {
	.track_name = NULL,
	.timescale = 0,
	.is_default = true,
};

int Recorder::start(const std::string &file_path)
{
	m_file_path = file_path;
	m_error = 0;
	m_state = State::Starting;
	int ret = scheduleStep(STEP_CREATE_PDRAW);
	if (ret < 0)
		failStep("scheduleStep", ret);
	return ret;
}

void Recorder::doStartStep(enum START_STEP step)
{
	switch (step) {
	case STEP_CREATE_PDRAW: {
		int ret = createPdrawInstance();
		if (ret < 0)
			return failStep("createPdrawInstance", ret);

		ret = scheduleStep(STEP_CREATE_SOURCE);
		if (ret < 0)
			return failStep("scheduleStep", ret);
		break;
	}
	case STEP_CREATE_SOURCE: {
		// Creating source
		ULOGI("Creating IAlsaSource");
		Pdraw::IPdraw::IAlsaSource *source;
		int ret = m_pdraw->createAlsaSource(
			&source_params, this, &source);
		if (ret < 0)
			return failStep("IPdraw::createAlsaSource", ret);
		ULOGI("IAlsaSource created on %s", source_params.address);
		m_source.reset(source);

		// next step called by alsaSourceReadyToPlay
		break;
	}
	case STEP_PLAY_SOURCE: {
		// Starting source
		ULOGI("IAlsaSource PLAY");
		int ret = m_source->play();
		if (ret < 0)
			return failStep("IAlsaSource::play", ret);

		// next step called by onMediaAdded
		break;
	}
	case STEP_CREATE_ENCODER: {
		// Creating encoder
		ULOGI("Creating IAudioEncoder");
		Pdraw::IPdraw::IAudioEncoder *encoder;
		int ret = m_pdraw->createAudioEncoder(
			m_pcm_media_id, &encoder_config, this, &encoder);
		if (ret < 0)
			return failStep("IPdraw::createAudioEncoder", ret);
		ULOGI("IAudioEncoder created for media %d", m_pcm_media_id);
		m_encoder.reset(encoder);

		// next step called by onMediaAdded
		break;
	}
	case STEP_CREATE_MUXER: {
		// Creating muxer
		ULOGI("Creating IMuxer");
		Pdraw::IPdraw::IMuxer *muxer;
		int ret = m_pdraw->createMuxer(
			m_file_path, &muxer_params, this, &muxer);
		if (ret < 0)
			return failStep("IPdraw::createMuxer", ret);
		// Own the muxer before addMedia(), so a failure releases it
		m_muxer.reset(muxer);
		ret = muxer->addMedia(m_aac_media_id, &muxer_media_params);
		if (ret < 0)
			return failStep("IMuxer::addMedia", ret);
		ULOGI("IMuxer created with media %d to file %s",
		      m_aac_media_id,
		      m_file_path.c_str());

		// end of start procedure
		m_state = State::Running;
		break;
	}
	}
}

int Recorder::stop()
{
	m_state = State::Stopping;
	return scheduleStopStep(STEP_PAUSE_SOURCE);
}

void Recorder::doStopStep(enum STOP_STEP step)
{
	switch (step) {
	case STEP_PAUSE_SOURCE: {
		if (m_source && !m_source->isPaused()) {
			// Pausing source if not
			ULOGI("IAlsaSource PAUSE");
			int ret = m_source->pause();
			if (ret == 0) {
				// next step called by alsaSourcePauseResponse
				break;
			}
			// Keep tearing down: the sequence must reach its end
			failStep("IAlsaSource::pause", ret);
		}

		scheduleStopStep(STEP_DELETE_SOURCE);
		break;
	}
	case STEP_DELETE_SOURCE: {
		if (m_source) {
			// Deleting source
			ULOGI("Removing IAlsaSource");
			m_source.reset();
			ULOGI("IAlsaSource removed");
		}

		scheduleStopStep(STEP_DELETE_ENCODER);
		break;
	}
	case STEP_DELETE_ENCODER: {
		if (m_encoder) {
			// Deleting encoder
			ULOGI("Removing IAudioEncoder");
			m_encoder.reset();
			ULOGI("IAudioEncoder removed");
		}

		scheduleStopStep(STEP_CLOSE_MUXER);
		break;
	}
	case STEP_CLOSE_MUXER: {
		if (m_muxer) {
			// Closing muxer
			ULOGI("IMuxer CLOSE");
			int ret = m_muxer->close();
			if (ret == 0) {
				// next step called by muxerCloseResponse
				break;
			}
			// Keep tearing down: the sequence must reach its end
			failStep("IMuxer::close", ret);
		}

		scheduleStopStep(STEP_DELETE_MUXER);
		break;
	}
	case STEP_DELETE_MUXER: {
		if (m_muxer) {
			// Deleting muxer
			ULOGI("Removing IMuxer");
			m_muxer.reset();
			ULOGI("IMuxer removed");
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

void Recorder::onAudioMediaAdded(enum adef_encoding encoding,
				 unsigned int media_id)
{
	// Waiting for the PCM audio media to create the encoder
	if (encoding == ADEF_ENCODING_PCM) {
		int ret = scheduleStep(STEP_CREATE_ENCODER);
		if (ret < 0)
			return failStep("scheduleStep", ret);
	}
	// Waiting for the AAC audio media to create the muxer
	else if (encoding == ADEF_ENCODING_AAC_LC) {
		m_aac_media_id = media_id;
		int ret = scheduleStep(STEP_CREATE_MUXER);
		if (ret < 0)
			return failStep("scheduleStep", ret);
	}
}

// IAlsaSource::Listener callbacks
void Recorder::alsaSourceReadyToPlay(
	Pdraw::IPdraw *pdraw,
	Pdraw::IPdraw::IAlsaSource *source,
	bool ready,
	enum pdraw_alsa_source_eos_reason eosReason)
{
	if (ready) {
		ULOGI("IAlsaSource ready to play");
		int ret = scheduleStep(STEP_PLAY_SOURCE);
		if (ret < 0)
			return failStep("scheduleStep", ret);
	}
}

void Recorder::alsaSourcePlayResponse(Pdraw::IPdraw *pdraw,
				      Pdraw::IPdraw::IAlsaSource *source)
{
	ULOGI("IAlsaSource playing");
}

void Recorder::alsaSourcePauseResponse(Pdraw::IPdraw *pdraw,
				       Pdraw::IPdraw::IAlsaSource *source)
{
	ULOGI("IAlsaSource paused");
	scheduleStopStep(STEP_DELETE_SOURCE);
}

// IMuxer::Listener callbacks
void Recorder::muxerCloseResponse(Pdraw::IPdraw *pdraw,
				  Pdraw::IPdraw::IMuxer *muxer,
				  int status)
{
	if (status != 0) {
		// Keep tearing down, the muxer still has to be deleted
		failStep("IMuxer close", status);
	} else {
		ULOGI("IMuxer closed");
	}

	scheduleStopStep(STEP_DELETE_MUXER);
}
