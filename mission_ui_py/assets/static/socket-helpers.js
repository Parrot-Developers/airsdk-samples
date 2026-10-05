//    Copyright (C) 2026 Parrot Drones SAS
//
//    Redistribution and use in source and binary forms, with or without
//    modification, are permitted provided that the following conditions
//    are met:
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in
//      the documentation and/or other materials provided with the
//      distribution.
//    * Neither the name of the Parrot Company nor the names
//      of its contributors may be used to endorse or promote products
//      derived from this software without specific prior written
//      permission.
//
//    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//    PARROT COMPANY BE LIABLE FOR ANY DIRECT, INDIRECT,
//    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
//    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
//    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
//    SUCH DAMAGE.

class WebSocketHelper {

  static services = {};
  static allSockets = {};
  static defaultSocket = null;
  static maxReconnectionAttempts = 5;
  static reconnectionAttempts = {};

  static startWithServices(services) {
    WebSocketHelper.services = services;

    let serviceNames = Object.keys(WebSocketHelper.services);
    if (serviceNames.length > 0) {
      WebSocketHelper.connect(serviceNames[0]);
    }
  }

  static connect(serviceName) {
    let service = WebSocketHelper.services[serviceName];
    if (!service) return;

    let ws = new WebSocket("ws://" + service.address + ":" + service.port + "/ws");
    WebSocketHelper.allSockets[serviceName] = ws;
    WebSocketHelper.defaultSocket = ws;
    ws.onopen = (event) => WebSocketHelper.onSocketOpened(event, serviceName);
    ws.onmessage = (event) => WebSocketHelper.onMessageReceived(event, serviceName);
    ws.onerror = (event) => WebSocketHelper.onSocketError(event, serviceName);
    ws.onclose = (event) => WebSocketHelper.onSocketClosed(event, serviceName);
  }

  static sendMessage(message, name) {
    let socket = name != null ? WebSocketHelper.allSockets[name] : WebSocketHelper.defaultSocket;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(message);
    } else {
      console.warn(`WebSocket is not open for ${name || 'default'} service. Cannot send message.`);
    }
  }

  static onSocketOpened(event, serviceName) {
    console.log(`WebSocket connection opened for ${serviceName} `, event);
    WebSocketHelper.reconnectionAttempts[serviceName] = 0; // Reset reconnection attempts on successful connection
  }

  static onMessageReceived(event, serviceName) {
    console.log(`[${serviceName}] Message received: `, event.data);
  }

  static onSocketError(event, serviceName) {
    console.error(`WebSocket error detected for ${serviceName}: ${event.message}`, event);
  }

  static onSocketClosed(event, serviceName) {
    console.log(`WebSocket closed for ${serviceName} - Code: ${event.code}, Reason: ${event.reason}`);

    // Only attempt to reconnect if the closure was not intentional
    if (event.code !== 1000) {
      WebSocketHelper.scheduleReconnection(serviceName);
    }
  }

  static scheduleReconnection(serviceName) {
    if (!WebSocketHelper.reconnectionAttempts[serviceName]) {
      WebSocketHelper.reconnectionAttempts[serviceName] = 0;
    }

    let attempt = WebSocketHelper.reconnectionAttempts[serviceName]++;
    let delay = Math.min(1000 * Math.pow(2, attempt), 30000); // Exponential backoff (max 30s)

    if (attempt < WebSocketHelper.maxReconnectionAttempts) {
      console.log(`Reconnection attempt ${attempt + 1} for ${serviceName} in ${delay / 1000}s...`);

      setTimeout(() => {
        WebSocketHelper.onSocketReconnect(serviceName);
      }, delay);
    } else {
      console.warn(`Max reconnection attempts reached for ${serviceName}. No further attempts will be made.`);
      WebSocketHelper.notifyUser(serviceName);
    }
  }

  static onSocketReconnect(serviceName) {
    console.log(`Reconnecting to service: ${serviceName}`);
    WebSocketHelper.connect(serviceName);
  }

  static notifyUser(serviceName) {
    console.warn(`Connection to ${serviceName} lost. Please check your internet or reload the page.`);
  }
}

// Close sockets cleanly on page teardown (e.g. the panel is reloaded or
// navigated away from): the server sees the disconnect immediately instead
// of only detecting it later, and the 1000 code stops onSocketClosed() from
// scheduling a reconnect for a page that no longer exists.
window.addEventListener("pagehide", () => {
  for (const ws of Object.values(WebSocketHelper.allSockets)) {
    if (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING) {
      ws.close(1000, "page unload");
    }
  }
});

// Retrieve WebSocket service parameters from URL
let urlParams = new URL(location.href).searchParams;
let addressPattern = /_address$/;
let portPattern = /_port$/;

let addressMap = {};
let portMap = {};
let services = {};

// Collect all address and port parameters
urlParams.forEach((value, key) => {
  if (addressPattern.test(key)) {
    addressMap[key] = value;
  } else if (portPattern.test(key)) {
    portMap[key] = value;
  }
});

// Match addresses to ports
for (const [addressKey, addressValue] of Object.entries(addressMap)) {
  const service = addressKey.substring(0, addressKey.length - '_address'.length);
  const portKey = service + '_port';
  const portValue = portMap[portKey];

  if (!portValue) continue;
  const portNumber = parseInt(portValue, 10);
  if (isNaN(portNumber)) continue;

  services[service] = { address: addressValue, port: portValue };
}

// Start WebSocket connections
WebSocketHelper.startWithServices(services);
