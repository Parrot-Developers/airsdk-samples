# Copyright (C) 2025 Parrot

import json
import logging
import uvicorn

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from typing import List, Callable, Dict

# Set up logging
logger = logging.getLogger("WebSocketServer")

# Initialize FastAPI application
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
        """Sends a message to all connected clients."""
        for connection in self.active_connections:
            await connection.send_text(message)

    async def send_message(self, websocket: WebSocket, message: str):
        """Sends a message to a specific client."""
        await websocket.send_text(message)


class WebSocketServer:
    def __init__(self):
        self.manager = ConnectionManager()
        self.command_handlers: Dict[str, Callable[[WebSocket, dict], None]] = {}

    def register_command(self, cmd: str, handler: Callable[[WebSocket, dict], None]):
        self.command_handlers[cmd] = handler

    async def handle_message(self, websocket: WebSocket, message: str):
        """Parses and processes a JSON message received via WebSocket."""
        logger.info(f"Received message: {message}")
        try:
            json_message = json.loads(message)  # JSON decoding
            cmd = json_message.get("cmd")

            if not cmd:
                response = {"error": "Missing command"}
                await self.manager.send_message(websocket, json.dumps(response))
                return

            handler = self.command_handlers.get(cmd)
            if handler:
                await handler(websocket, json_message)
            else:
                response = {"error": "Unknown command"}
                await self.manager.send_message(websocket, json.dumps(response))
        except json.JSONDecodeError:
            response = {"error": "Invalid JSON format"}
            await self.manager.send_message(websocket, json.dumps(response))

    # WebSocket endpoint to handle incoming messages
    async def websocket_endpoint(self, websocket: WebSocket):
        await self.manager.connect(websocket)
        try:
            while True:
                data = await websocket.receive_text()
                await self.handle_message(websocket, data)
        except WebSocketDisconnect:
            self.manager.disconnect(websocket)

    def start(self, process_name: str, airsdk_publisher, host="0.0.0.0", port=0):
        """Starts FastAPI WebSocket server with port publishing via AirSDK"""
        app.websocket("/ws")(self.websocket_endpoint)

        config = uvicorn.Config(
            app, host=host, port=port, log_level="info", ws="websockets"
        )
        server = uvicorn.Server(config)

        # Hack to retrieve the server port
        orig_log_started_message = server._log_started_message

        def patch_log_started_message(listeners):
            orig_log_started_message(listeners)
            port = listeners[0].getsockname()[1]
            airsdk_publisher.publish_service(process_name, port)  # Publish the port

        server._log_started_message = patch_log_started_message
        server.run()
