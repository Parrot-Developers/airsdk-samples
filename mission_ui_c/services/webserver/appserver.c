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
 * Lightweight WebSocket JSON dispatch server using libwebsockets (LWS).
 */

#include "appserver.h"
#include <json-c/json.h>
#include <libwebsockets.h>
#include <ulog.h>

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define ULOG_TAG appserver
ULOG_DECLARE_TAG(ULOG_TAG);

/* Configuration */
static const size_t MAX_OUTQ_PER_CLIENT = 256; /* per-client queue limit */

struct msg_node {
	char *msg; /* null-terminated JSON string */
	struct msg_node *next;
};

struct per_session_data {
	struct msg_node *head;
	struct msg_node *tail;
	size_t size; /* number of messages in queue */
	pthread_mutex_t mtx;
};

struct cmd_entry {
	char *cmd;
	ws_cmd_handler_t handler;
	struct cmd_entry *next;
};

/* Globals */
volatile sig_atomic_t run = 1; /* run flag (declared extern in header) */
static pthread_t server_thread;
static struct lws_context *g_context = NULL;

/* Clients dynamic array */
static struct lws **clients = NULL;
static size_t clients_count = 0;
static size_t clients_capacity = 0;
static pthread_mutex_t clients_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Command table */
static struct cmd_entry *cmd_table_head = NULL;
static pthread_mutex_t cmd_table_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Optional publish callback and process name */
static void (*publish_port_cb)(const char *process_name, int port) = NULL;
static char *g_process_name = NULL;

/* Forward declarations */
static void remove_client(struct lws *wsi);
static void add_client(struct lws *wsi);
static bool queue_message_for_wsi(struct lws *wsi, const char *json_msg);
static void free_cmd_table(void);

/* Signal handler stops the LWS service loop */
void sig_handler(int sig)
{
	(void)sig;
	run = 0;
	if (g_context)
		lws_cancel_service(g_context);
}

/* Helper: duplicate string safely */
static char *strdup_safe(const char *s)
{
	if (!s)
		return NULL;
	size_t n = strlen(s) + 1;
	char *r = malloc(n);
	if (!r)
		return NULL;
	memcpy(r, s, n);
	return r;
}

/* Remove client from global list */
static void remove_client(struct lws *wsi)
{
	pthread_mutex_lock(&clients_mutex);
	for (size_t i = 0; i < clients_count; ++i) {
		if (clients[i] == wsi) {
			/* shift left */
			for (size_t j = i + 1; j < clients_count; ++j)
				clients[j - 1] = clients[j];
			clients_count--;
			/* optionally shrink capacity */
			if (clients_count == 0) {
				free(clients);
				clients = NULL;
				clients_capacity = 0;
			}
			break;
		}
	}
	pthread_mutex_unlock(&clients_mutex);
}

/* Add client to global list */
static void add_client(struct lws *wsi)
{
	pthread_mutex_lock(&clients_mutex);
	if (clients_count == clients_capacity) {
		size_t newcap = clients_capacity ? clients_capacity * 2 : 8;
		struct lws **tmp = realloc(clients, newcap * sizeof(*clients));
		if (!tmp) {
			ULOGW("add_client: OOM");
			pthread_mutex_unlock(&clients_mutex);
			return;
		}
		clients = tmp;
		clients_capacity = newcap;
	}
	clients[clients_count++] = wsi;
	pthread_mutex_unlock(&clients_mutex);
}

/* lws_callback_on_writable() is the only LWS api safe to call off-thread. */
static bool queue_message_for_wsi(struct lws *wsi, const char *json_msg)
{
	if (!wsi) {
		ULOGW("queue_message_for_wsi: null wsi");
		return false;
	}

	struct per_session_data *ps =
		(struct per_session_data *)lws_wsi_user(wsi);
	if (!ps) {
		ULOGW("queue_message_for_wsi: per-session data null");
		return false;
	}

	pthread_mutex_lock(&ps->mtx);
	if (ps->size >= MAX_OUTQ_PER_CLIENT) {
		ULOGW("client queue full (size=%zu), dropping oldest message",
		      ps->size);
		/* drop oldest */
		struct msg_node *old = ps->head;
		if (old) {
			ps->head = old->next;
			if (ps->head == NULL)
				ps->tail = NULL;
			free(old->msg);
			free(old);
			ps->size--;
		}
	}

	struct msg_node *node = malloc(sizeof(*node));
	if (!node) {
		ULOGW("queue_message_for_wsi: OOM for node");
		pthread_mutex_unlock(&ps->mtx);
		return false;
	}
	node->msg = strdup_safe(json_msg);
	node->next = NULL;
	if (!node->msg) {
		free(node);
		ULOGW("queue_message_for_wsi: OOM for msg");
		pthread_mutex_unlock(&ps->mtx);
		return false;
	}

	if (ps->tail) {
		ps->tail->next = node;
		ps->tail = node;
	} else {
		ps->head = ps->tail = node;
	}
	ps->size++;
	pthread_mutex_unlock(&ps->mtx);

	lws_callback_on_writable(wsi);

	return true;
}

/* Public messaging API */

/* Send to a specific client. Returns 0 on success. */
int send_message(struct lws *wsi, const char *json_msg)
{
	if (!queue_message_for_wsi(wsi, json_msg))
		return -1;
	return 0;
}

/* Send json_msg to every connected client (best-effort). */
void broadcast(const char *json_msg)
{
	struct lws **snapshot = NULL;
	size_t snapshot_count = 0;

	pthread_mutex_lock(&clients_mutex);
	if (clients_count > 0) {
		snapshot = malloc(clients_count * sizeof(*clients));
		if (snapshot) {
			memcpy(snapshot,
			       clients,
			       clients_count * sizeof(*clients));
			snapshot_count = clients_count;
		}
	}
	pthread_mutex_unlock(&clients_mutex);

	for (size_t i = 0; i < snapshot_count; ++i) {
		(void)queue_message_for_wsi(snapshot[i], json_msg);
	}

	free(snapshot);
}

/* Command registration */

int register_command(const char *cmd, ws_cmd_handler_t handler)
{
	if (!cmd || !handler)
		return -1;

	pthread_mutex_lock(&cmd_table_mutex);
	struct cmd_entry *cur = cmd_table_head;
	while (cur) {
		if (strcmp(cur->cmd, cmd) == 0) {
			cur->handler = handler;
			pthread_mutex_unlock(&cmd_table_mutex);
			return 0;
		}
		cur = cur->next;
	}

	struct cmd_entry *n = malloc(sizeof(*n));
	if (!n) {
		pthread_mutex_unlock(&cmd_table_mutex);
		return -1;
	}
	n->cmd = strdup_safe(cmd);
	if (!n->cmd) {
		free(n);
		pthread_mutex_unlock(&cmd_table_mutex);
		return -1;
	}
	n->handler = handler;
	n->next = cmd_table_head;
	cmd_table_head = n;
	pthread_mutex_unlock(&cmd_table_mutex);
	return 0;
}

/* Find handler by cmd name (caller should hold no locks) */
static ws_cmd_handler_t find_handler(const char *cmd)
{
	if (!cmd)
		return NULL;
	pthread_mutex_lock(&cmd_table_mutex);
	struct cmd_entry *cur = cmd_table_head;
	while (cur) {
		if (strcmp(cur->cmd, cmd) == 0) {
			ws_cmd_handler_t h = cur->handler;
			pthread_mutex_unlock(&cmd_table_mutex);
			return h;
		}
		cur = cur->next;
	}
	pthread_mutex_unlock(&cmd_table_mutex);
	return NULL;
}

/* Error helper */
static void send_error(struct lws *wsi, const char *err_str)
{
	struct json_object *j = json_object_new_object();
	json_object_object_add(j, "error", json_object_new_string(err_str));
	const char *s = json_object_to_json_string(j);
	send_message(wsi, s);
	json_object_put(j);
}

/* LWS callback */
static int callback_ws(struct lws *wsi,
		       enum lws_callback_reasons reason,
		       void *user,
		       void *in,
		       size_t len)
{
	(void)user;
	switch (reason) {
	case LWS_CALLBACK_ESTABLISHED: {
		ULOGI("WebSocket client connected");
		struct per_session_data *ps =
			(struct per_session_data *)lws_wsi_user(wsi);
		if (ps) {
			ps->head = ps->tail = NULL;
			ps->size = 0;
			if (pthread_mutex_init(&ps->mtx, NULL) != 0) {
				ULOGW("pthread_mutex_init failed for per-session mtx");
			}
		} else {
			ULOGW("LWS did not provide per-session storage");
		}
		add_client(wsi);
		break;
	}

	case LWS_CALLBACK_CLOSED: {
		ULOGI("WebSocket client disconnected");
		struct per_session_data *ps =
			(struct per_session_data *)lws_wsi_user(wsi);
		if (ps) {
			pthread_mutex_lock(&ps->mtx);
			struct msg_node *n = ps->head;
			while (n) {
				struct msg_node *tmp = n->next;
				free(n->msg);
				free(n);
				n = tmp;
			}
			ps->head = ps->tail = NULL;
			ps->size = 0;
			pthread_mutex_unlock(&ps->mtx);
			pthread_mutex_destroy(&ps->mtx);
		}
		remove_client(wsi);
		break;
	}

	case LWS_CALLBACK_RECEIVE: {
		/* Received raw frame (not null-terminated) */
		char *msg = malloc(len + 1);
		if (!msg) {
			ULOGW("LWS receive: OOM");
			break;
		}
		memcpy(msg, in, len);
		msg[len] = '\0';
		ULOGI("Received message: %s", msg);

		struct json_object *jobj = json_tokener_parse(msg);
		if (!jobj) {
			ULOGW("Invalid JSON received");
			send_error(wsi, "Invalid JSON format");
			free(msg);
			break;
		}

		struct json_object *jcmd = NULL;
		if (!json_object_object_get_ex(jobj, "cmd", &jcmd)) {
			send_error(wsi, "Missing command");
			json_object_put(jobj);
			free(msg);
			break;
		}

		const char *cmd_str = json_object_get_string(jcmd);
		if (!cmd_str) {
			send_error(wsi, "Invalid cmd value");
			json_object_put(jobj);
			free(msg);
			break;
		}

		ULOGI("Command: %s", cmd_str);

		ws_cmd_handler_t handler = find_handler(cmd_str);
		if (handler) {
			int rc = handler(wsi, jobj);
			if (rc != 0) {
				send_error(wsi, "Handler returned error");
			}
		} else {
			send_error(wsi, "Unknown command");
		}

		json_object_put(jobj);
		free(msg);
		break;
	}

	case LWS_CALLBACK_SERVER_WRITEABLE: {
		struct per_session_data *ps =
			(struct per_session_data *)lws_wsi_user(wsi);
		if (!ps)
			break;

		struct msg_node *node = NULL;
		pthread_mutex_lock(&ps->mtx);
		if (ps->head) {
			node = ps->head;
			ps->head = node->next;
			if (ps->head == NULL)
				ps->tail = NULL;
			ps->size--;
		}
		pthread_mutex_unlock(&ps->mtx);

		if (!node)
			break;

		size_t msg_len = strlen(node->msg);
		/* guard insane sizes */
		if (msg_len > 10 * 1024 * 1024) { /* 10MB arbitrary cap */
			ULOGW("message too large (%zu), dropping", msg_len);
			free(node->msg);
			free(node);
			break;
		}

		unsigned char *buf = malloc(LWS_PRE + msg_len);
		if (!buf) {
			ULOGW("lws_write failed: OOM buffer");
			free(node->msg);
			free(node);
			break;
		}
		memcpy(buf + LWS_PRE, node->msg, msg_len);
		int n = lws_write(
			wsi, buf + LWS_PRE, (int)msg_len, LWS_WRITE_TEXT);
		if (n < 0) {
			ULOGW("lws_write failed: %d", n);
		}
		free(buf);
		free(node->msg);
		free(node);

		/* If more messages remain, request another writable callback */
		pthread_mutex_lock(&ps->mtx);
		bool more = (ps->head != NULL);
		pthread_mutex_unlock(&ps->mtx);
		if (more)
			lws_callback_on_writable(wsi);

		break;
	}

	default:
		break;
	}

	return 0;
}

/* Protocols array */
static struct lws_protocols protocols[] = {{"websocket-protocol",
					    callback_ws,
					    sizeof(struct per_session_data),
					    65536,
					    0,
					    NULL,
					    0},
					   {NULL, NULL, 0, 0, 0, NULL, 0}};

/* Server thread args */
struct server_args {
	int port;
	char *bind_address; /* owns this */
};

/* Server loop: create context and run lws_service */
static void *webserver_loop(void *arg)
{
	struct server_args *sa = (struct server_args *)arg;
	int port = sa->port;
	const char *bind_address = sa->bind_address;

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
			ULOGE("socket() failed: %s", strerror(errno));
			free(sa->bind_address);
			free(sa);
			return NULL;
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

		if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) <
		    0) {
			ULOGE("bind() failed: %s", strerror(errno));
			close(listen_fd);
			free(sa->bind_address);
			free(sa);
			return NULL;
		}

		if (listen(listen_fd, 16) < 0) {
			ULOGE("listen() failed: %s", strerror(errno));
			close(listen_fd);
			free(sa->bind_address);
			free(sa);
			return NULL;
		}

		struct sockaddr_in sa_loc;
		socklen_t sa_len = sizeof(sa_loc);
		if (getsockname(listen_fd,
				(struct sockaddr *)&sa_loc,
				&sa_len) == 0) {
			detected_port = ntohs(sa_loc.sin_port);
			ULOGI("Auto-bound listen socket on port %d (fd=%d)",
			      detected_port,
			      listen_fd);
		} else {
			ULOGW("getsockname() failed after bind");
		}

		info.vh_listen_sockfd = (lws_sockfd_type)listen_fd;
	} else {
		info.port = port;
		info.iface = sa->bind_address; /* lws will not take ownership */
	}

	g_context = lws_create_context(&info);
	if (!g_context) {
		ULOGE("Failed to create LWS context");
		if (listen_fd >= 0)
			close(listen_fd);
		free(sa->bind_address);
		free(sa);
		return NULL;
	}

	/* publish chosen port if requested */
	if (publish_port_cb) {
		if (detected_port != 0)
			publish_port_cb(g_process_name ? g_process_name : "",
					detected_port);
		else if (port != 0)
			publish_port_cb(g_process_name ? g_process_name : "",
					port);
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
	g_context = NULL;
	ULOGI("WebSocket server stopped");

	if (listen_fd >= 0)
		close(listen_fd);

	free(sa->bind_address);
	free(sa);
	return NULL;
}

/* Free the command table entries (called after server stopped) */
static void free_cmd_table(void)
{
	pthread_mutex_lock(&cmd_table_mutex);
	struct cmd_entry *cur = cmd_table_head;
	while (cur) {
		struct cmd_entry *tmp = cur->next;
		free(cur->cmd);
		free(cur);
		cur = tmp;
	}
	cmd_table_head = NULL;
	pthread_mutex_unlock(&cmd_table_mutex);
}

/* Start/stop API (public) */
void start_webserver(const char *process_name,
		     void (*publish_port)(const char *, int),
		     const char *bind_address,
		     int port)
{
	if (g_process_name) {
		free(g_process_name);
		g_process_name = NULL;
	}
	g_process_name = strdup_safe(process_name ? process_name : "");
	publish_port_cb = publish_port;

	run = 1;

	struct server_args *sa = malloc(sizeof(*sa));
	if (!sa) {
		ULOGE("start_webserver: OOM");
		return;
	}
	sa->port = port;
	/* If caller passed NULL, use DEFAULT_BIND_ADDRESS */
	sa->bind_address =
		strdup_safe(bind_address ? bind_address : DEFAULT_BIND_ADDRESS);
	if (!sa->bind_address) {
		ULOGE("start_webserver: OOM allocating bind_address");
		free(sa);
		return;
	}

	int rc = pthread_create(&server_thread, NULL, webserver_loop, sa);
	if (rc != 0) {
		ULOGE("pthread_create failed: %s", strerror(rc));
		free(sa->bind_address);
		free(sa);
	}
}

void stop_webserver()
{
	run = 0;
	if (g_context)
		lws_cancel_service(g_context);
	pthread_join(server_thread, NULL);

	/* free command table entries */
	free_cmd_table();

	/* free client array structure if still allocated (we don't touch wsi
	 * pointers after context destroyed) */
	pthread_mutex_lock(&clients_mutex);
	if (clients) {
		free(clients);
		clients = NULL;
		clients_count = 0;
		clients_capacity = 0;
	}
	pthread_mutex_unlock(&clients_mutex);

	/* free process name */
	if (g_process_name) {
		free(g_process_name);
		g_process_name = NULL;
	}

	/* destroy mutexes */
	pthread_mutex_destroy(&clients_mutex);
	pthread_mutex_destroy(&cmd_table_mutex);
}
