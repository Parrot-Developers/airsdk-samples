/*
 * Copyright (c) 2026 Parrot Drones SAS
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
 *
 * appserver.hpp
 *
 * WebSocket JSON dispatch server using libwebsockets
 *
 */

#pragma once

#include <csignal>
#include <functional>
#include <libwebsockets.h>
#include <string>

/* Forward declaration json-c */
struct json_object;

#define DEFAULT_BIND_ADDRESS "0.0.0.0"

/**
 * Type for command handler. The handler receives:
 *   - struct lws *wsi : the client socket
 *   - struct json_object *jobj : parsed JSON (json-c)
 * Returns 0 on success, !=0 on error.
 */
using ws_cmd_handler_t =
	std::function<int(struct lws *wsi, struct json_object *jobj)>;

/* Server lifecycle */

/**
 * Start the WebSocket server.
 * - process_name: name used for service publication (can be nullptr)
 * - publish_port: optional callback to publish chosen port (can be nullptr)
 * - bind_address: interface to bind (default 0.0.0.0)
 * - port: 0 = auto-bind
 */
void start_webserver(const char *process_name,
		     void (*publish_port)(const char *, int),
		     const char *bind_address = DEFAULT_BIND_ADDRESS,
		     int port = 0);

/**
 * Stop the WebSocket server gracefully.
 */
void stop_webserver();

/* signals / run flag */

/**
 * C linkage signal handler you can use to stop the server.
 */
extern "C" void sig_handler(int sig);

/**
 * Global flag indicating if the server should keep running.
 */
extern volatile sig_atomic_t run;

/* Messaging API */

/**
 * Broadcast a JSON message to all connected clients.
 */
void broadcast(const std::string &json_msg);

/**
 * Send a JSON message to a specific WebSocket client.
 * Returns 0 on success, -1 on error (e.g. invalid wsi).
 */
int send_message(struct lws *wsi, const std::string &json_msg);

/* Command registration */

/**
 * Register a command handler.
 * - cmd: command name (value of JSON field "cmd")
 * - handler: function called with (wsi, parsed_json_object)
 * Returns 0 on success.
 */
int register_command(const std::string &cmd, ws_cmd_handler_t handler);
