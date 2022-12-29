# Copyright (c) 2023 Parrot Drones SAS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
# * Neither the name of the Parrot Company nor the names
#   of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written
#   permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# PARROT COMPANY BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.

import asyncio
import logging
import signal
import ulog
import telemetry_binding

# This is limited to 15 charecters
PROCESS_NAME = b"ex_tlm_py"


async def service_main():
    # Initialisation code
    #
    # The service is automatically started by the drone when the mission is
    # loaded.
    ulog.setup_logging(PROCESS_NAME)
    logger = logging.getLogger("main")
    logger.info("Hello from example telemetry")
    run = True
    def sig_handler(*_):
        nonlocal run
        run = False

    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGTERM, sig_handler)

    single_sample_consumer = telemetry_binding.Consumer("/dev/shm")
    pressure = telemetry_binding.types.float32_t(0.0)
    temperature = telemetry_binding.types.array_float32_t(0.0, 10)
    single_sample_consumer.reg(pressure, "sensors_barometer.pressure")
    single_sample_consumer.reg(temperature, "sensors_imu.temperature")
    single_sample_consumer.regComplete()

    my_producer = telemetry_binding.Producer("new_section", 100, 1000)
    new_value_source = telemetry_binding.types.uint64_t(0)
    my_producer.reg(new_value_source, "new_value")
    my_producer.regComplete()

    my_consumer = telemetry_binding.Consumer()
    new_value_destination = telemetry_binding.types.uint64_t(0)
    my_consumer.reg(new_value_destination, "new_section.new_value")
    my_consumer.regComplete()

    # Loop code
    #
    # The service is assumed to run an infinite loop, and termination
    # requests are handled via a SIGTERM signal.
    # If your service exits before this SIGTERM is sent, it will be
    # considered as a crash, and the system will relaunch the service.
    # If this happens too many times, the system will no longer start the
    # service.
    while run:
        if int(new_value_source) > 1000:
            new_value_source.write(0)
        else:
            new_value_source.write(int(new_value_source) + 1)
        my_producer.putSample()

        await asyncio.sleep(5)

        single_sample_consumer.getSample(
            telemetry_binding.types.timespec(0, 0),
            telemetry_binding.Method.Latest,
        )
        my_consumer.getSample(
            telemetry_binding.types.timespec(0, 0),
            telemetry_binding.Method.Latest,
        )

        logger.info("##################################################")
        logger.info("#### TELEMETRY VALUE:")
        logger.info("####")
        logger.info("#### single_sample_consumer")
        logger.info("#### > barometer")
        logger.info(f"####     pressure {pressure}")
        logger.info("####")
        logger.info("#### > imu")
        for i in range(10):
            logger.info(f"####     temperature[{i}]: {temperature[i]}")
        logger.info("####")
        logger.info("#### my_consumer")
        logger.info("#### > new_section")
        logger.info(f"####     new_value {new_value_destination}")

    # Cleanup code
    #
    # When stopped by a SIGTERM, a service can use a short amount of time
    # for cleanup (typically closing opened files and ensuring that the
    # written data is coherent).
    logger.info("Cleaning up from toto")
    single_sample_consumer = None
    my_producer = None
    my_consumer = None

    return 0


def main():
    asyncio.run(service_main())
