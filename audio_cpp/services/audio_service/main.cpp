#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <unistd.h>
#include <libpomp.hpp>

#include "player.hpp"
#include "recorder.hpp"

#define ULOG_TAG audio_service
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#define RECORDING_TIME 5000

/****************************************************************/

/* Stop flag, set to 1 by signal handler to exit cleanly */
static volatile sig_atomic_t stop;

/****************************************************************/

class Context {
	/**
	 * @brief Phases of the record-then-play sequence, advanced from the
	 * main event loop as PDrAW signals each step's completion.
	 */
	enum class Phase {
		StartingRecorder, /**< Waiting for the recorder to be running.
				   */
		Recording, /**< Capturing until the recording timer fires. */
		StoppingRecorder, /**< Waiting for the recorder to be stopped.
				   */
		Playing,          /**< Playing the recording back until stop. */
	};

	/* Main loop of the program */
	pomp::Loop mLoop;

	Recorder mRecorder{this->mLoop};
	Player mPlayer{this->mLoop};

	/* Recording-duration timer and the flag its handler raises */
	bool mTimerExpired = false;
	pomp::Timer::HandlerFunc mTimerHandler{
		[this] { this->mTimerExpired = true; }};
	pomp::Timer mTimer{&this->mLoop, &this->mTimerHandler};

	Phase mPhase{Phase::StartingRecorder};
	std::string mRecordPath;

public:
	inline void wakeup()
	{
		this->mLoop.wakeup();
	}
	/** @brief Error which aborted a pipeline, 0 if none failed. */
	inline int error() const
	{
		int res = this->mRecorder.error();
		return res != 0 ? res : this->mPlayer.error();
	}
	inline int start()
	{
		const char *mission_data = getenv("MISSION_DATA");
		if (mission_data == nullptr) {
			ULOGE("MISSION_DATA environment variable is not set");
			return -EINVAL;
		}
		this->mRecordPath = std::string(mission_data) + "/record.m4a";

		ULOGI("Starting recording");
		int res = this->mRecorder.start(this->mRecordPath);
		if (res != 0) {
			ULOG_ERRNO("Recorder::start", -res);
			return res;
		}
		this->mPhase = Phase::StartingRecorder;
		return 0;
	}
	inline void waitAndProcess(int timeout)
	{
		this->mLoop.waitAndProcess(timeout);
		/* Stop advancing the sequence once a stop is requested, so
		 * playback never starts while we are shutting down. */
		if (!stop)
			switch (this->mPhase) {
			case Phase::StartingRecorder:
				/* Arm the timer only once the pipeline is fully
				 * up so the recorded duration matches
				 * RECORDING_TIME. */
				if (this->mRecorder.running()) {
					this->mTimer.set(RECORDING_TIME);
					this->mPhase = Phase::Recording;
				}
				break;
			case Phase::Recording:
				if (this->mTimerExpired) {
					this->mTimer.clear();
					ULOGI("Stopping recording");
					this->mRecorder.stop();
					this->mPhase = Phase::StoppingRecorder;
				}
				break;
			case Phase::StoppingRecorder:
				if (this->mRecorder.stopped()) {
					ULOGI("Starting playing");
					/* Failures surface through error() */
					this->mPlayer.start(this->mRecordPath);
					this->mPhase = Phase::Playing;
				}
				break;
			case Phase::Playing:
				/* Nothing to do: play back until a stop is
				 * requested. */
				break;
			}
	}
	inline void cleanup()
	{
		/* SIGTERM received: tear down whichever pipeline is active.
		 * Before playback has started, stop the recorder and never
		 * play; once playing, stop the player. */
		this->mTimer.clear();
		switch (this->mPhase) {
		case Phase::StartingRecorder:
		case Phase::Recording:
			this->mRecorder.stop();
			break;
		case Phase::StoppingRecorder:
			/* Recorder teardown is already in flight. */
			break;
		case Phase::Playing:
			this->mPlayer.stop();
			break;
		}

		/* Pump the loop until both pipelines have fully torn down. */
		while (!this->mRecorder.stopped() || !this->mPlayer.stopped())
			this->mLoop.waitAndProcess(-1);
	}
};

/*
 * Global context of the mission. It retrieves all the objects and
 * controllers the mission needs to work up
 */
static Context s_ctx;

/****************************************************************/

static void sighandler(int signum)
{
	/* Set stop flag and wakeup loop */
	ULOGI("Signal %d (%s) received", signum, strsignal(signum));
	stop = 1;
	s_ctx.wakeup();
}

/****************************************************************/

int main(int argc, char *argv[])
{
	int res;

	/* Initialisation code
	 *
	 * The service is automatically started by the drone when the mission is
	 * loaded.
	 */
	ULOGI("Hello from audio_service");
	/* Setup signal handler */
	signal(SIGINT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGPIPE, SIG_IGN);

	/* Initialize and start context */
	res = s_ctx.start();
	if (res != 0) {
		ULOGE("Error while starting audio_service");
		ULOG_ERRNO("Context::start", -res);
		return res;
	}

	/* Loop code
	 *
	 * The service runs the record-then-play sequence until a termination
	 * request is received via a SIGTERM signal, or until a pipeline step
	 * fails: such a failure is not recoverable, so the service tears down
	 * and exits with an error instead of waiting forever.
	 */
	while (!stop && s_ctx.error() == 0)
		s_ctx.waitAndProcess(-1);

	/* Cleanup code
	 *
	 * On SIGTERM or on a pipeline failure, gracefully tear down the active
	 * pipeline so the recorded file is coherent before the service exits.
	 */
	ULOGI("Cleaning up from audio_service");
	s_ctx.cleanup();

	res = s_ctx.error();
	if (res != 0)
		ULOG_ERRNO("audio_service", -res);

	return res == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
