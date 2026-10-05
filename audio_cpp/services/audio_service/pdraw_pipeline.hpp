/**
 * @file pdraw_pipeline.hpp
 * @brief Base class factoring the common PDrAW pipeline lifecycle.
 *
 * Both Recorder and Player drive a PDrAW pipeline through an asynchronous,
 * step-by-step state machine posted on a pomp idle handler. This base captures
 * everything they share: the event loop, the PDrAW instance, the step
 * scheduling plumbing, the IPdraw::Listener boilerplate, and the stop/delete
 * teardown of the PDrAW instance itself. Derived classes only implement the
 * steps specific to their elements (source+encoder+muxer vs demuxer+renderer).
 */

#pragma once

#include <map>
#include <memory>
#include <string>
#include <libpomp.hpp>
#include <pdraw/pdraw.hpp>

/**
 * @class PdrawPipeline
 * @brief Common base for the asynchronous PDrAW audio pipelines.
 *
 * Inherits the PDrAW listener interface as @c protected so that derived
 * classes remain valid @c IPdraw listeners while keeping the callbacks out of
 * their public API.
 */
class PdrawPipeline : protected Pdraw::IPdraw::Listener {
public:
	/**
	 * @brief Construct a pipeline attached to the given event loop.
	 * @param loop pomp event loop used to schedule asynchronous steps.
	 */
	PdrawPipeline(pomp::Loop &loop) : m_loop(loop) {}

	virtual ~PdrawPipeline() = default;

	/**
	 * @brief Check whether the pipeline is fully torn down.
	 * @return @c true once teardown has completed (state is Idle).
	 */
	bool stopped() const
	{
		return m_state == State::Idle;
	}

	/**
	 * @brief Check whether the pipeline is fully started and running.
	 * @return @c true once the startup sequence has completed (state is
	 *         Running).
	 */
	bool running() const
	{
		return m_state == State::Running;
	}

	/**
	 * @brief Get the error which aborted the pipeline, if any.
	 *
	 * A failed step stops the sequence, so the owner has to tear the
	 * pipeline down instead of waiting for running() forever.
	 *
	 * @return negative errno of the first failed step, 0 if none failed.
	 */
	int error() const
	{
		return m_error;
	}

protected:
	// -------------------------------------------------------------------------
	// Lifecycle state (replaces the ambiguous m_running flag)
	// -------------------------------------------------------------------------

	/**
	 * @brief Explicit pipeline lifecycle state.
	 */
	enum class State {
		Idle,     /**< Nothing created. */
		Starting, /**< Startup sequence in flight. */
		Running,  /**< Fully started. */
		Stopping, /**< Teardown sequence in flight. */
	};

	// -------------------------------------------------------------------------
	// Step scheduling
	// -------------------------------------------------------------------------

	/**
	 * @brief Post a step to run on the next loop iteration.
	 *
	 * Each step value owns a distinct handler slot, created on demand and
	 * kept at a stable address, so several different steps may be pending
	 * at once — e.g. creating the ALSA source synchronously triggers both
	 * alsaSourceReadyToPlay and onMediaAdded, queueing two follow-up steps.
	 * Keying by value means there is no fixed bound to overflow.
	 *
	 * Accepts any START_STEP / STOP_STEP enumerator (unscoped enums convert
	 * to @c int implicitly). The two enums number from 0 and therefore
	 * share slots, which is safe because the start and stop sequences never
	 * run concurrently.
	 *
	 * @param step Step to schedule.
	 * @return 0 on success, negative errno on failure.
	 */
	int scheduleStep(int step);

	/**
	 * @brief Abort the current sequence on a step failure.
	 *
	 * Logs @p what and records @p err for error(). Use as
	 * @c return failStep(...) so that no failure is only logged.
	 *
	 * @param what Name of the operation which failed, for the log.
	 * @param err Error code of the failure (either sign).
	 */
	void failStep(const char *what, int err);

	/**
	 * @brief Schedule a teardown step, running it inline as a fallback.
	 *
	 * The teardown must complete, so a step the loop refuses to take is run
	 * inline. The stop sequence is short, so the recursion is bounded.
	 *
	 * @param step Teardown step to schedule.
	 * @return 0 on success, negative errno if the step was run inline.
	 */
	int scheduleStopStep(int step);

	/**
	 * @brief Dispatch a scheduled step.
	 *
	 * Derived classes cast @p step back to their START_STEP / STOP_STEP
	 * enum (disambiguated by the current state) and run the matching
	 * action.
	 *
	 * @param step Raw step value previously passed to scheduleStep().
	 */
	virtual void runStep(int step) = 0;

	// -------------------------------------------------------------------------
	// Shared PDrAW lifecycle helpers (used by the derived start/stop steps)
	// -------------------------------------------------------------------------

	/**
	 * @brief Create the PDrAW instance into m_pdraw.
	 * @return 0 on success, negative errno on failure.
	 */
	int createPdrawInstance();

	/**
	 * @brief Tear down the PDrAW instance and return to the Idle state.
	 *
	 * Call this as the last teardown step. If a PDrAW instance exists it is
	 * asked to stop and the teardown completes asynchronously in
	 * stopResponse(); otherwise it completes immediately.
	 *
	 * @return 0 on success, negative errno on failure.
	 */
	int requestPdrawStop();

	/**
	 * @brief Hook called for every audio media added to the pipeline.
	 *
	 * The PCM media id is already recorded in m_pcm_media_id before this is
	 * called. Derived classes map an encoding to the next step to schedule
	 * (e.g. PCM → create encoder/renderer, AAC → create muxer).
	 *
	 * @param encoding Encoding of the added audio media.
	 * @param media_id Pipeline id of the added media.
	 */
	virtual void onAudioMediaAdded(enum adef_encoding encoding,
				       unsigned int media_id) = 0;

	// -------------------------------------------------------------------------
	// IPdraw::Listener — shared implementations
	// -------------------------------------------------------------------------

	/** @brief Common teardown tail: drop the instance and return to Idle.
	 */
	void stopResponse(Pdraw::IPdraw *pdraw, int status) override;

	/** @brief Records the media id, then forwards to onAudioMediaAdded().
	 */
	void onMediaAdded(Pdraw::IPdraw *pdraw,
			  const struct pdraw_media_info *info,
			  void *elementUserData) override;

	// Boilerplate the samples never use — defaulted once, here.
	void onMediaRemoved(Pdraw::IPdraw *pdraw,
			    const struct pdraw_media_info *info,
			    void *elementUserData) override
	{
	}
	void onSocketCreated(Pdraw::IPdraw *pdraw, int fd) override {}

	// -------------------------------------------------------------------------
	// Shared members
	// -------------------------------------------------------------------------

	pomp::Loop &m_loop; /**< Event loop used to post idle handlers. */

	std::unique_ptr<Pdraw::IPdraw> m_pdraw; /**< PDrAW pipeline instance. */

	/** @brief Lifecycle state, only used from the event loop thread. */
	State m_state = State::Idle;

	int m_error = 0; /**< Error of the first failed step, 0 if none. */

	unsigned int m_pcm_media_id = 0; /**< Id of the PCM audio media. */

	/** @brief Idle-handler slots, keyed by step value (one per step). */
	std::map<int, pomp::Loop::IdleHandlerFunc> m_handlers;

private:
	/** @brief Drop the PDrAW instance and mark the pipeline Idle. */
	void finishStop();
};
