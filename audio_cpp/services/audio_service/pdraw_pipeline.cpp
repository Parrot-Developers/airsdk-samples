#include "pdraw_pipeline.hpp"

#define ULOG_TAG pdraw_pipeline
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

int PdrawPipeline::scheduleStep(int step)
{
	m_handlers[step].set([this, step] { runStep(step); });
	return m_loop.idleAdd(&m_handlers[step]);
}

void PdrawPipeline::failStep(const char *what, int err)
{
	if (err > 0)
		err = -err;
	ULOG_ERRNO("%s", -err, what);
	/* Keep the first error, the one which aborted the sequence. */
	if (m_error == 0)
		m_error = err;
}

int PdrawPipeline::scheduleStopStep(int step)
{
	int ret = scheduleStep(step);
	if (ret < 0) {
		failStep("scheduleStep", ret);
		// The teardown must complete, run the step inline
		runStep(step);
	}
	return ret;
}

int PdrawPipeline::createPdrawInstance()
{
	ULOGI("Creating IPdraw");
	Pdraw::IPdraw *pdraw = nullptr;
	int ret = Pdraw::createPdraw(m_loop.get(), this, &pdraw);
	ULOG_ERRNO_RETURN_ERR_IF(ret < 0, -ret);
	m_pdraw.reset(pdraw);
	ULOGI("IPdraw created");
	return 0;
}

int PdrawPipeline::requestPdrawStop()
{
	if (!m_pdraw) {
		// Nothing to stop, complete the teardown immediately
		finishStop();
		return 0;
	}

	ULOGI("IPdraw STOP");
	int ret = m_pdraw->stop();
	if (ret < 0) {
		failStep("IPdraw::stop", ret);
		// The teardown must complete even on error
		finishStop();
		return ret;
	}

	// Teardown completes asynchronously in stopResponse()
	return 0;
}

void PdrawPipeline::finishStop()
{
	if (m_pdraw) {
		ULOGI("Removing IPdraw");
		m_pdraw.reset();
		ULOGI("IPdraw removed");
	}
	m_state = State::Idle;
}

// IPdraw::Listener callbacks
void PdrawPipeline::stopResponse(Pdraw::IPdraw *pdraw, int status)
{
	if (status != 0)
		failStep("IPdraw stop", status);
	else
		ULOGI("IPdraw stopped");

	// Complete the teardown whatever the status: stopped() must become true
	finishStop();
}

void PdrawPipeline::onMediaAdded(Pdraw::IPdraw *pdraw,
				 const struct pdraw_media_info *info,
				 void *elementUserData)
{
	ULOGI("Media %d added: %s",
	      info->id,
	      Pdraw::pdrawMediaTypeStr(info->type));

	if (info->type != PDRAW_MEDIA_TYPE_AUDIO)
		return;

	const enum adef_encoding encoding = info->audio.format.encoding;
	if (encoding == ADEF_ENCODING_PCM)
		m_pcm_media_id = info->id;

	onAudioMediaAdded(encoding, info->id);
}
