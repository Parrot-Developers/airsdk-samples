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

#ifndef EXAMPLE_TELEMETRY_H
#define EXAMPLE_TELEMETRY_H

/* example_telemetry context */
struct example_telemetry_context;

/**
 * @brief Create a new example_telemetry object.
 *
 * @return example_telemetry structure or 'NULL' in case of error
 */
struct example_telemetry_context *example_telemetry_new(void);

/**
 * @brief Destroy a example_telemetry object.
 *
 * @param ctx example_telemetry structure.
 */
void example_telemetry_destroy(struct example_telemetry_context *ctx);

/**
 * @brief Configure the example_telemetry object.
 *
 * Register all telemetry values.
 *
 * @param ctx example_telemetry structure.
 * @return 0 in case of success, negative errno value in case of error.
 */
int example_telemetry_init(struct example_telemetry_context *ctx);

/**
 * @brief Put samples of all registered producers.
 *
 * @param ctx example_telemetry structure.
 */
void example_telemetry_put_samples(struct example_telemetry_context *ctx);

/**
 * @brief Get samples of all registered consumers.
 *
 * @param ctx example_telemetry structure.
 */
void example_telemetry_get_samples(struct example_telemetry_context *ctx);

/**
 * @brief Log the telemetry values for debug.
 *
 * @param ctx example_telemetry structure.
 */
void example_telemetry_log_values(struct example_telemetry_context *ctx);

#endif // EXAMPLE_TELEMETRY_H