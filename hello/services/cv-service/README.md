
# Sample of native service

## Introduction

This sample code demonstrates how to use the different libraries to communicate
with other modules on the system. The sample implements a daemon that runs
indefinitely using a single event loop (single thread program). It is also
interrupt friendly (it exits cleanly when it receives SIGINT/SIGTERM signals).

## Dependencies

* `libulog`: Library to log debugging information as text. It will log in a
  kernel ring buffer that will timestamp it. Log can de displayed by executing
  the command `ulogcat -C -v long`.<br>
  If the process that logs is launched with the environment variable
  `ULOG_STDERR` set to `y`, logs will also be written to `stderr`.

* `libpomp`: Library that provides 2 main features:
  * poll-based event loop to monitor a set of file descriptors (sockets,
    timers).
  * small communication framework using a printf-like approach to ease exchange
    of data between a client and a server.

* `libtelemetry`: Library to read and write telemetry data.

## Ulog usage

`Ulog` is used to log text information. Several severity levels are available.
By default everything except the `Debug` level is displayed. All macros require,
the `ULOG_TAG` to be defined prior to include `ulog.h`. Also, for each tag used,
a single `ULOG_DECLARE_TAG` is required. The following environment variables can
be used to modify the behavior:

* `ULOG_LEVEL`: Set to `C`, `E`, `W`, `N`, `I`, `D` to change the global
  verbosity of the process.
* `ULOG_LEVEL_<tag>`: Similar to `ULOG_LEVEL` but only affects the given
  `<tag>`.
* `ULOG_STDERR`: If set to `y`, it will also log on `stderr`.

## Telemetry usage

The telemetry library is used to exchange data between processes via a shared
memory and a lock-free circular buffer. A producer writes data as timestamped
samples. A sample is a set of values whose description (name, type...) is
indicated in the shared memory header.

Both producer and consumer need to register the variables they wish to exchange
by giving the name, type, as well as a source/destination pointer. This pointer
needs to be valid during the whole life-cycle of the producer/consumer.

Video presentation: https://www.youtube.com/watch?v=i3S7Wa3MuUY
