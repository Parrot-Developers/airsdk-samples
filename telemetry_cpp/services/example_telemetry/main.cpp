/**
 * Copyright (c) 2023 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of the Parrot Company nor the names
 *   of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written
 *   permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * PARROT COMPANY BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <csignal>
#include <unistd.h>

#include <libtelemetry.hpp>

#define ULOG_TAG ex_tlm_cpp
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#define MAX_SAMPLE 10

sig_atomic_t run = 1;

extern "C" void sig_handler(int sig)
{
	run = 0;
}

struct MyStruct {
	int myIntergerValue;
	float myFloatValue;
};

namespace telemetry {
template <> int ClassRegistrator<MyStruct>::reg(ClassDesc &d, Type &v)
{
	d.reg(v.myIntergerValue, "my_integer_value");
	d.reg(v.myFloatValue, "my_float_value");
	return 0;
};
} // namespace telemetry

int main(int argc, char *argv[])
{
	/* Initialisation code
	 *
	 * The service is automatically started by the drone when the mission is
	 * loaded.
	 */
	ULOGI("Hello from example_telemetry");
	signal(SIGTERM, sig_handler);

	int res;
	int tmp_integer;
	float tmp_float;

	res = 0;

	/**
	 * Consume telemetry about the drone.
	 *
	 * In this example:
	 *    - Get a single sample of pressure from the barometer sensor.
	 *    - Get an array of samples of altitude_agl (altitude above ground
	 *          level) from the drone controller.
	 */
	telemetry::Consumer *single_sample_consumer;
	float pressure;

	telemetry::Consumer *array_sample_consumer;
	float altitude_agl[MAX_SAMPLE];
	size_t nb_array_sample;
	size_t ref_array_sample;

	/**
	 * Produce and consume your own telemetry.
	 *
	 * In this example:
	 *     - Produce and consume a single integer sample.
	 *     - Produce and consume a single struct sample.
	 */
	telemetry::Producer *my_producer;
	int new_value_source;

	telemetry::Consumer *my_consumer;
	int new_value_destination;

	telemetry::Producer *my_struct_producer;
	MyStruct new_struct_source;

	telemetry::Consumer *my_struct_consumer;
	MyStruct new_struct_destination;

	single_sample_consumer = telemetry::Consumer::create();

	array_sample_consumer = telemetry::Consumer::create();

	my_producer = telemetry::Producer::create(
		"new_section", /* Name of the producer (section).generally
						  snake_case */
		"/dev/shm", /* Directory where to create the shared memory.
					   default = "/dev/shm". */
		10, /* Maximum number of samples */
		5000000 /* Approximative rate of samples (in us). [5 sec for
				   this exemple] */
	);
	my_consumer = telemetry::Consumer::create();

	my_struct_producer = telemetry::Producer::create(
		"new_struct_section", /* Name of the producer (section).generally
								snake_case */
		"/dev/shm", /* Directory where to create the shared memory.
					   default = "/dev/shm". */
		10, /* Maximum number of samples */
		5000000 /* Approximative rate of samples (in us). [5 sec for
				   this exemple] */
	);

	my_struct_consumer = telemetry::Consumer::create();

	/**
	 * Register the Pressure value of barometer sensor (Pa) "pressure" from
	 * the sensors_battery telemetry section. This telemetry field is a
	 * scalar float. Only 1 sample is consumed.
	 */
	single_sample_consumer->reg(pressure, "sensors_barometer.pressure");
	res = single_sample_consumer->regComplete();
	if (res < 0) {
		ULOG_ERRNO("single_sample_consumer regComplete", -res);
		goto out;
	}

	/**
	 * Register the altitude above ground level "altitude_agl" from the
	 * drone_controller telemetry section. This telemetry field is a scalar
	 * float and we want to consume the last MAX_SAMPLE (10) samples. The
	 * type of the registered variable is float[MAX_SAMPLE].
	 */
	array_sample_consumer->regSamplesArray(altitude_agl,
					       "drone_controller.altitude_agl");
	res = array_sample_consumer->regComplete();
	if (res < 0) {
		ULOG_ERRNO("array_sample_consumer regComplete", -res);
		goto out;
	}

	/**
	 * New value producer registration.
	 *
	 * Register the new_value value from the new_section telemetry section.
	 * This telemetry field is a scalar int. Only 1 sample is produced.
	 */
	new_value_source = 0;
	my_producer->reg(new_value_source, "new_value");
	res = my_producer->regComplete();
	if (res < 0) {
		ULOG_ERRNO("my_producer regComplete", -res);
		goto out;
	}

	/**
	 * New value consumer registration.
	 *
	 * Register the new_value value from the new_section telemetry section.
	 * This telemetry field is a scalar int. Only 1 sample is consumed.
	 */
	new_value_destination = 0;
	my_consumer->reg(new_value_destination, "new_section.new_value");
	res = my_consumer->regComplete();
	if (res < 0) {
		ULOG_ERRNO("my_consumer regComplete", -res);
		goto out;
	}

	/**
	 * New struct producer registration.
	 *
	 * Register the new_struct from the new_struct_section telemetry section.
	 * This telemetry field is a MyStruct. Only 1 sample is produced.
	 */
	new_struct_source.myIntergerValue = 0;
	new_struct_source.myFloatValue = 0.0;
	my_struct_producer->reg(new_struct_source, "new_struct");
	res = my_struct_producer->regComplete();
	if (res < 0) {
		ULOG_ERRNO("my_struct_producer regComplete", -res);
		goto out;
	}

	/**
	 * New struct Consumer registration.
	 *
	 * Register the new_struct from the new_struct_section telemetry section.
	 * This telemetry field is a MyStruct. Only 1 sample is produced.
	 */
	new_struct_destination.myIntergerValue = 0;
	new_struct_destination.myFloatValue = 0.0;
	my_struct_consumer->reg(new_struct_destination,
			       "new_struct_section.new_struct");
	res = my_struct_consumer->regComplete();
	if (res < 0) {
		ULOG_ERRNO("my_struct_consumer regComplete", -res);
		goto out;
	}

	/* Loop code
	 *
	 * The service is assumed to run an infinite loop, and termination
	 * requests are handled via a SIGTERM signal.
	 * If your service exits before this SIGTERM is sent, it will be
	 * considered as a crash, and the system will relaunch the service.
	 * If this happens too many times, the system will no longer start the
	 * service.
	 */
	while (run) {

		if (new_value_source > 5) {
			new_value_source = 0;

			new_struct_source.myIntergerValue = 0;
			new_struct_source.myFloatValue= 0.0;
		} else {
			new_value_source++;

			tmp_integer = new_struct_source.myIntergerValue;
			tmp_float = new_struct_source.myFloatValue;

			tmp_integer++;
			tmp_float += 0.6;

			new_struct_source.myIntergerValue = tmp_integer;
			new_struct_source.myFloatValue = tmp_float;
		}

		/* Put only one sample */
		res = my_producer->putSample(
			/* timestamp of sample (can be nullptr to use current
			   time). */
			nullptr);
		if (res < 0) {
			ULOGW("Can't put my_producer sample %s",
			      strerror(-res));
		}

		/* Put only one sample */
		res = my_struct_producer->putSample(
			/* timestamp of sample (can be nullptr to use current
			   time). */
			nullptr);
		if (res < 0) {
			ULOGW("Can't put my_struct_producer sample %s",
			      strerror(-res));
		}

		sleep(5);

		/* Get only one sample */
		res = single_sample_consumer->getSample(
			/* timestamp of query (can be nullptr for LATEST
			   method). */
			nullptr,
			/* method of query. */
			telemetry::Method::TLM_LATEST);
		if (!res) {
			ULOGW("Can't read single_sample_consumer sample %s",
			      strerror(-res));
		}

		/* Get several samples at once */
		res = array_sample_consumer->getSamples(
			/* timestamp of query (can be nullptr for LATEST
			   method). */
			nullptr,
			/* method of query. */
			telemetry::Method::TLM_LATEST,
			/* number of requested samples before the timestamp */
			MAX_SAMPLE - 1,
			/* number of requested samples after the timestamp.
			nb_samples_before + nb_samples_after +1 <= nb_samples */
			0,
			/* number of actual returned samples */
			&nb_array_sample,
			/* index of the reference sample in the table. It is
			related to the reference timestamp and depends on the
			method. */
			&ref_array_sample);
		if (!res) {
			ULOGW("Can't read array_sample_consumer sample %s",
			      strerror(-res));
		}

		/* Get only one sample */
		res = my_consumer->getSample(
			/* timestamp of query (can be nullptr for LATEST
			   method). */
			nullptr,
			/* method of query. */
			telemetry::Method::TLM_LATEST);
		if (!res) {
			ULOGW("Can't read my_consumer sample %s",
			      strerror(-res));
		}

		/* Get only one sample */
		res = my_struct_consumer->getSample(
			/* timestamp of query (can be nullptr for LATEST
			   method). */
			nullptr,
			/* method of query. */
			telemetry::Method::TLM_LATEST);
		if (!res) {
			ULOGW("Can't read my_struct_consumer sample %s",
			      strerror(-res));
		}

		ULOGI("##################################################");
		ULOGI("#### TELEMETRY VALUES:");
		ULOGI("####");
		ULOGI("#### simple_sample_consumer");
		ULOGI("#### > pressure");
		ULOGI("####     pressure %f", pressure);
		ULOGI("####");
		ULOGI("#### array_sample_consumer:");
		for (int i = 0; i < MAX_SAMPLE; i++)
			ULOGI("####     altitude_agl[%d]: %f",
			      i,
			      altitude_agl[i]);
		ULOGI("####  nb_array_sample : %ld", nb_array_sample);
		ULOGI("####  ref_array_sample: %ld", ref_array_sample);
		ULOGI("####");
		ULOGI("#### my_consumer");
		ULOGI("#### > new_section");
		ULOGI("####     new_value %d", new_value_destination);
		ULOGI("####");
		ULOGI("#### my_struct_consumer");
		ULOGI("#### > new_struct_section");
		ULOGI("####     my_integer_value %d",
		      new_struct_destination.myIntergerValue);
		ULOGI("####     my_float_value   %f",
		      new_struct_destination.myFloatValue);
	}

	/* Cleanup code
	 *
	 * When stopped by a SIGTERM, a service can use a short amount of time
	 * for cleanup (typically closing opened files and ensuring that the
	 * written data is coherent).
	 */
	ULOGI("Cleaning up from example_telemetry");

out:
	telemetry::Consumer::release(single_sample_consumer);
	telemetry::Consumer::release(array_sample_consumer);
	telemetry::Producer::release(my_producer);
	telemetry::Consumer::release(my_consumer);

	if (res < 0)
		return EXIT_FAILURE;
	else
		return EXIT_SUCCESS;
}