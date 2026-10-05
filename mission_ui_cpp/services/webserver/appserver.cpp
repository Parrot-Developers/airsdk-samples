/**
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
 * Lightweight WebSocket JSON dispatch server using libwebsockets (LWS).
 */

#include "appserver.hpp"

#include <json-c/json.h>
#include <libwebsockets.h>

#include <algorithm>
#include <arpa/inet.h>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <functional>
#include <mutex>
#include <netinet/in.h>
#include <new>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <vector>

#define ULOG_TAG appserver
#include <ulog.hpp>
ULOG_DECLARE_TAG(ULOG_TAG);

/* Configuration */
static constexpr size_t MAX_OUTQ_PER_CLIENT = 256; /* per-client queue limit */

/* Globals */
/* run flag (declared extern in header) */
volatile sig_atomic_t run = 1;

static std::thread server_thread;
static struct lws_context *g_context = nullptr;

/* Active clients and mutex */
static std::vector<struct lws *> clients;
static std::mutex clients_mutex;

/* Command table and mutex */
static std::unordered_map<std::string, ws_cmd_handler_t> cmd_table;
static std::mutex cmd_table_mutex;

/* Optional publish callback and process name */
static void (*publish_port_cb)(const char *process_name, int port) = nullptr;
static std::string g_process_name;

/* Signal handler stops the LWS service loop */
extern "C" void sig_handler(int sig)
{
	(void)sig;
	run = 0;
	if (g_context)
		lws_cancel_service(g_context);
}

/* Per-session data: outgoing message queue + mutex, allocated by LWS per
 * connection ("per_session" size in the protocols array). */
struct per_session_data {
	std::deque<std::string> outq;
	std::mutex mtx;

	per_session_data() = default;
	~per_session_data() = default;
};

/* Internal helpers */

/* Remove client from global list */
static void remove_client(struct lws *wsi)
{
	std::lock_guard<std::mutex> lock(clients_mutex);
	auto it = std::remove(clients.begin(), clients.end(), wsi);
	if (it != clients.end())
		clients.erase(it, clients.end());
}

/* Add client to global list */
static void add_client(struct lws *wsi)
{
	std::lock_guard<std::mutex> lock(clients_mutex);
	clients.push_back(wsi);
}

/* lws_callback_on_writable() is the only LWS api safe to call off-thread. */
static bool queue_message_for_wsi(struct lws *wsi, const std::string &json_msg)
{
	if (!wsi) {
		ULOGW("queue_message_for_wsi: null wsi");
		return false;
	}

	per_session_data *ps =
		reinterpret_cast<per_session_data *>(lws_wsi_user(wsi));
	if (!ps) {
		ULOGW("queue_message_for_wsi: per-session data null");
		return false;
	}

	{
		std::lock_guard<std::mutex> lk(ps->mtx);
		if (ps->outq.size() >= MAX_OUTQ_PER_CLIENT) {
			ULOGW("client queue full (size=%zu), dropping oldest message",
			      ps->outq.size());
			ps->outq.pop_front();
		}
		ps->outq.push_back(json_msg);
	}

	lws_callback_on_writable(wsi);
	return true;
}

/* Public messaging API */

/* Send to a specific client. Returns 0 on success. */
int send_message(struct lws *wsi, const std::string &json_msg)
{
	if (!queue_message_for_wsi(wsi, json_msg))
		return -1;
	return 0;
}

/* Send json_msg to every connected client (best-effort). */
void broadcast(const std::string &json_msg)
{
	std::vector<struct lws *> snapshot;
	{
		std::lock_guard<std::mutex> lock(clients_mutex);
		snapshot = clients;
	}

	for (auto wsi : snapshot) {
		(void)queue_message_for_wsi(wsi, json_msg);
	}
}

/* Command registration */

int register_command(const std::string &cmd, ws_cmd_handler_t handler)
{
	std::lock_guard<std::mutex> lock(cmd_table_mutex);
	cmd_table[cmd] = handler;
	return 0;
}

/* Error helper */
static void send_error(struct lws *wsi, const char *err_str)
{
	struct json_object *j = json_object_new_object();
	json_object_object_add(j, "error", json_object_new_string(err_str));
	const char *s = json_object_to_json_string(j);
	send_message(wsi, std::string(s));
	json_object_put(j);
}

/* LWS callback */

static int callback_ws(struct lws *wsi,
		       enum lws_callback_reasons reason,
		       void *user,
		       void *in,
		       size_t len)
{
	switch (reason) {
	case LWS_CALLBACK_ESTABLISHED: {
		ULOGI("WebSocket client connected");
		per_session_data *ps =
			reinterpret_cast<per_session_data *>(lws_wsi_user(wsi));
		if (ps) {
			new (ps) per_session_data();
		} else {
			ULOGW("LWS did not provide per-session storage");
		}
		add_client(wsi);
		break;
	}

	case LWS_CALLBACK_CLOSED: {
		ULOGI("WebSocket client disconnected");
		per_session_data *ps =
			reinterpret_cast<per_session_data *>(lws_wsi_user(wsi));
		if (ps) {
			ps->~per_session_data();
		}
		remove_client(wsi);
		break;
	}

	case LWS_CALLBACK_RECEIVE: {
		/* Received raw frame (not null-terminated) */
		std::string msg(reinterpret_cast<char *>(in), len);
		ULOGI("Received message: %s", msg.c_str());

		struct json_object *jobj = json_tokener_parse(msg.c_str());
		if (!jobj) {
			ULOGW("Invalid JSON received");
			send_error(wsi, "Invalid JSON format");
			break;
		}

		struct json_object *jcmd = nullptr;
		if (!json_object_object_get_ex(jobj, "cmd", &jcmd)) {
			send_error(wsi, "Missing command");
			json_object_put(jobj);
			break;
		}

		const char *cmd_str = json_object_get_string(jcmd);
		if (!cmd_str) {
			send_error(wsi, "Invalid cmd value");
			json_object_put(jobj);
			break;
		}

		ULOGI("Command: %s", cmd_str);

		/* Dispatch to registered handler */
		{
			std::lock_guard<std::mutex> lock(cmd_table_mutex);
			auto it = cmd_table.find(cmd_str);
			if (it != cmd_table.end()) {
				int rc = 0;
				try {
					rc = it->second(wsi, jobj);
				} catch (...) {
					ULOGE("Exception in command handler");
					send_error(wsi, "Handler error");
				}
				if (rc != 0) {
					send_error(wsi,
						   "Handler returned error");
				}
			} else {
				send_error(wsi, "Unknown command");
			}
		}

		json_object_put(jobj);
		break;
	}

	case LWS_CALLBACK_SERVER_WRITEABLE: {
		/* LWS service thread: pop one queued message and write it */
		per_session_data *ps =
			reinterpret_cast<per_session_data *>(lws_wsi_user(wsi));
		if (!ps)
			break;

		std::string out;
		{
			std::lock_guard<std::mutex> lk(ps->mtx);
			if (ps->outq.empty())
				break;
			out = std::move(ps->outq.front());
			ps->outq.pop_front();
		}

		if (!out.empty()) {
			size_t msg_len = out.size();
			std::vector<unsigned char> buf(LWS_PRE + msg_len);
			memcpy(buf.data() + LWS_PRE, out.data(), msg_len);
			int n = lws_write(wsi,
					  buf.data() + LWS_PRE,
					  static_cast<int>(msg_len),
					  LWS_WRITE_TEXT);
			if (n < 0) {
				ULOGW("lws_write failed: %d", n);
			}
		}

		/* If more messages remain, request another writable callback */
		{
			std::lock_guard<std::mutex> lk(ps->mtx);
			if (!ps->outq.empty())
				lws_callback_on_writable(wsi);
		}
		break;
	}

	default:
		break;
	}

	return 0;
}

/* Protocols array */
static struct lws_protocols protocols[] = {
	{"websocket-protocol",
	 callback_ws,
	 sizeof(per_session_data),
	 65536,
	 0,
	 nullptr,
	 0},
	{nullptr, nullptr, 0, 0, 0, nullptr, 0}};

/* Server loop: create context and run lws_service */
static void webserver_loop(int port, const char *bind_address)
{
	struct lws_context_creation_info info;
	memset(&info, 0, sizeof(info));

	info.protocols = protocols;
	info.options = LWS_SERVER_OPTION_VALIDATE_UTF8;

	int listen_fd = -1;
	int detected_port = port;

	if (port == 0) {
		/* Pre-bind socket to detect chosen port */
		listen_fd = socket(AF_INET, SOCK_STREAM, 0);
		if (listen_fd < 0) {
			ULOGE("socket() failed");
			return;
		}
		int on = 1;
		setsockopt(
			listen_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

		struct sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));
		addr.sin_family = AF_INET;
		if (bind_address && strcmp(bind_address, "0.0.0.0") != 0) {
			if (inet_aton(bind_address, &addr.sin_addr) == 0) {
				ULOGW("invalid bind_address '%s', falling back to INADDR_ANY",
				      bind_address);
				addr.sin_addr.s_addr = htonl(INADDR_ANY);
			}
		} else {
			addr.sin_addr.s_addr = htonl(INADDR_ANY);
		}
		addr.sin_port = htons(0);

		if (bind(listen_fd,
			 reinterpret_cast<struct sockaddr *>(&addr),
			 sizeof(addr)) < 0) {
			ULOGE("bind() failed");
			close(listen_fd);
			return;
		}

		if (listen(listen_fd, 16) < 0) {
			ULOGE("listen() failed");
			close(listen_fd);
			return;
		}

		struct sockaddr_in sa;
		socklen_t sa_len = sizeof(sa);
		if (getsockname(listen_fd,
				reinterpret_cast<struct sockaddr *>(&sa),
				&sa_len) == 0) {
			detected_port = ntohs(sa.sin_port);
			ULOGI("Auto-bound listen socket on port %d (fd=%d)",
			      detected_port,
			      listen_fd);
		} else {
			ULOGW("getsockname() failed after bind");
		}

		info.vh_listen_sockfd =
			reinterpret_cast<lws_sockfd_type>(listen_fd);
	} else {
		info.port = port;
		info.iface = bind_address;
	}

	g_context = lws_create_context(&info);
	if (!g_context) {
		ULOGE("Failed to create LWS context");
		if (listen_fd >= 0)
			close(listen_fd);
		return;
	}

	/* publish chosen port if requested */
	if (publish_port_cb) {
		if (detected_port != 0)
			publish_port_cb(g_process_name.c_str(), detected_port);
		else if (port != 0)
			publish_port_cb(g_process_name.c_str(), port);
		else
			ULOGW("Auto-bind but port not detected; cannot publish");
	}

	ULOGI("WebSocket server running (requested port %d, effective %d)",
	      port,
	      detected_port);

	while (run) {
		int n = lws_service(g_context, 1000);
		if (n < 0)
			break;
	}

	lws_context_destroy(g_context);
	g_context = nullptr;
	ULOGI("WebSocket server stopped");

	if (listen_fd >= 0)
		close(listen_fd);
}

/* Start/stop API (public) */
void start_webserver(const char *process_name,
		     void (*publish_port)(const char *, int),
		     const char *bind_address,
		     int port)
{
	g_process_name = process_name ? process_name : "";
	publish_port_cb = publish_port;

	run = 1;
	server_thread = std::thread(webserver_loop, port, bind_address);
}

void stop_webserver()
{
	run = 0;
	if (g_context)
		lws_cancel_service(g_context);
	if (server_thread.joinable())
		server_thread.join();
}
