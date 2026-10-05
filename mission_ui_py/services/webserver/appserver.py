# Copyright (c) 2026 Parrot Drones SAS
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
import json
import logging
import uvicorn

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from typing import List, Callable, Dict, Optional

logger = logging.getLogger("WebSocketServer")

app = FastAPI()


# WebSocket connection manager to handle multiple clients
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        """Accepts a new WebSocket connection and adds it to the list."""
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(
            f"WebSocket connection established, total {len(self.active_connections)} clients"
        )

    def disconnect(self, websocket: WebSocket):
        """Removes a WebSocket connection from the list when a client disconnects."""
        self.active_connections.remove(websocket)
        logger.info(
            f"WebSocket client disconnected, {len(self.active_connections)} left"
        )

    async def broadcast(self, message: str):
        """Sends a message to all connected clients (best-effort)."""
        for connection in list(self.active_connections):
            try:
                await connection.send_text(message)
            except Exception:
                logger.warning("Failed to send to a client, skipping")

    async def send_message(self, websocket: WebSocket, message: str):
        """Sends a message to a specific client."""
        await websocket.send_text(message)


class WebSocketServer:
    def __init__(self):
        self.manager = ConnectionManager()
        self.command_handlers: Dict[str, Callable[[WebSocket, dict], None]] = {}
        # Set once uvicorn's own loop is running (see start()); lets other
        # threads target it with asyncio.run_coroutine_threadsafe().
        self.loop: Optional[asyncio.AbstractEventLoop] = None

    def register_command(self, cmd: str, handler: Callable[[WebSocket, dict], None]):
        self.command_handlers[cmd] = handler

    async def handle_message(self, websocket: WebSocket, message: str):
        """Parses and processes a JSON message received via WebSocket."""
        logger.info(f"Received message: {message}")
        try:
            json_message = json.loads(message)
        except json.JSONDecodeError:
            response = {"error": "Invalid JSON format"}
            await self.manager.send_message(websocket, json.dumps(response))
            return

        cmd = json_message.get("cmd")
        if not cmd:
            response = {"error": "Missing command"}
            await self.manager.send_message(websocket, json.dumps(response))
            return

        handler = self.command_handlers.get(cmd)
        if not handler:
            response = {"error": "Unknown command"}
            await self.manager.send_message(websocket, json.dumps(response))
            return

        try:
            await handler(websocket, json_message)
        except Exception as e:
            logger.warning(f"Handler for '{cmd}' failed: {e}")
            response = {"error": "Handler error"}
            await self.manager.send_message(websocket, json.dumps(response))

    # WebSocket endpoint to handle incoming messages
    async def websocket_endpoint(self, websocket: WebSocket):
        await self.manager.connect(websocket)
        try:
            while True:
                data = await websocket.receive_text()
                await self.handle_message(websocket, data)
        except WebSocketDisconnect:
            pass
        finally:
            # Always clean up, even if the loop above exits on an
            # unexpected error rather than a clean disconnect.
            self.manager.disconnect(websocket)

    def start(
        self,
        process_name: str,
        airsdk_publisher,
        host="0.0.0.0",
        port=0,
        on_ready: Callable[[], None] = None,
        on_shutdown: Callable[[], None] = None,
    ):
        """Starts FastAPI WebSocket server with port publishing via AirSDK.

        on_ready/on_shutdown run from FastAPI's startup/shutdown lifecycle,
        once self.loop is the live serving loop.
        """
        app.websocket("/ws")(self.websocket_endpoint)

        @app.on_event("startup")
        async def _on_startup():
            self.loop = asyncio.get_running_loop()
            if on_ready:
                on_ready()

        if on_shutdown:
            @app.on_event("shutdown")
            async def _on_shutdown():
                on_shutdown()

        config = uvicorn.Config(
            app,
            host=host,
            port=port,
            log_level="info",
            ws="websockets",
            # Detect a dead peer (radio/wifi drop) faster than the 20s/20s
            # defaults, since this drives a live telemetry side panel.
            ws_ping_interval=5,
            ws_ping_timeout=5,
        )
        server = uvicorn.Server(config)

        # Hack to retrieve the server port
        orig_log_started_message = server._log_started_message

        def patch_log_started_message(listeners):
            orig_log_started_message(listeners)
            port = listeners[0].getsockname()[1]
            airsdk_publisher.publish_service(process_name, port)

        server._log_started_message = patch_log_started_message
        server.run()
