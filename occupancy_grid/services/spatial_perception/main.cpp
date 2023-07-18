/**
 * Copyright (C) 2023 parrot
 */

#include <csignal>
#include <string>

#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include <libpomp.hpp>
#define ULOG_TAG spatial_perception
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "spatial_perception.hpp"

/* The grid provider name. */
static const std::string GRID_PROVIDER_NAME = "default.grid";
/* The grid server address */
static const std::string GRID_SERVER_ADDR = "unix:@/tmp/mapping-grid";

sig_atomic_t run = 1;

/*
 * Context inherits from SpatialPerceptionClient so it can get informed
 * when the occupation grid is available.
 */
class Context : public SpatialPerception::Client, public pomp::Timer::Handler {
private:
  /* Main loop of the program. */
  pomp::Loop mLoop;
  /* Timer to periodically query grid density. */
  pomp::Timer mTimer;
  /* The spatial perception client */
  SpatialPerception mSpatialPerception;

  static constexpr uint32_t TIMER_INITAL_DELAY = 1000; /* ms */
  static constexpr uint32_t TIMER_PERIOD = 1000;       /* ms */

  static constexpr int8_t GRID_OCCUPATION_LOGODD = 1;
  static constexpr float GRID_STEP_DISTANCE = 1.2; /* meters */

public:
  Context()
      : mTimer(&this->mLoop, this),
        mSpatialPerception(&this->mLoop, this, GRID_SERVER_ADDR,
                           GRID_PROVIDER_NAME) {}

  inline void wakeup() { this->mLoop.wakeup(); }
  inline void waitAndProcess(int timeout) {
    this->mLoop.waitAndProcess(timeout);
  }

  inline int start() { return this->mSpatialPerception.start(); }

  /*
   * SpatialPerceptionClient override
   */
  virtual void onSpatialPerceptionReady() override {
    ULOGI("onSpatialPerceptionReady");
    /* once at least a grid is available schedule a periodic timer to
     * query the occupation grid density */
    this->mTimer.setPeriodic(TIMER_INITAL_DELAY, TIMER_PERIOD);
  }

  virtual void processTimer() override {
    if (this->mSpatialPerception.isReady() == false)
      return;
    const moser::IGrid &grid = this->mSpatialPerception.getGrid();
    const float density = grid.getObstacleDensityRatio(GRID_OCCUPATION_LOGODD,
                                                       GRID_STEP_DISTANCE);
    ULOGI("occupation density %f", density);
  }
};
/*
 * Global context of the mission. It retrieves all the objects the mission needs
 * to work with.
 */
static Context s_ctx;

extern "C" void sig_handler(int signum) {
  run = 0;
  /* Set stopped flag and wakeup loop */
  ULOGI("Signal %d (%s) received", signum, strsignal(signum));
  s_ctx.wakeup();
}

int main(int argc, char *argv[]) {
  int res;
  /* Initialisation code
   *
   * The service is automatically started by the drone when the mission is
   * loaded.
   */
  ULOGI("Hello from spatial_perception");
  signal(SIGTERM, sig_handler);

  /* Initialize and start context */
  res = s_ctx.start();
  if (res != 0) {
    ULOGE("Error while starting spatial perception client");
    return res;
  }
  ULOGI("SpatialPerceptionClient has started successfully");

  /* Loop code
   *
   * The service is assumed to run an infinite loop, and termination
   * requests are handled via a SIGTERM signal.
   * If your serivce exists before this SIGTERM is sent, it will be
   * considered as a crash, and the system will relaunch the service.
   * If this happens too many times, the system will no longer start the
   * service.
   */
  while (run) {
    s_ctx.waitAndProcess(-1);
  }

  /* Cleanup code
   *
   * When stopped by a SIGTERM, a service can use a short amount of time
   * for cleanup (typically closing opened files and ensuring that the
   * written data is coherent).
   */
  ULOGI("Cleaning up from spatial_perception");
  return res == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
