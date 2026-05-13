// WebSocket initialization
WebSocketHelper.onMessageReceived = (event) => onRecvMsg(event.data);

// When the WebView receives a message, we want to display information about it below the button
let messagesArray = [];
function onRecvMsg(data) {
    var msg = JSON.parse(data);

    if (msg.hasOwnProperty("sendMsg")) {
        let button = document.getElementById("sendMsg");
        let message = document.createElement('p');
        message.textContent = msg.sendMsg;
        message.style.textAlign = "center";
        message.style.margin = "10px 0";

        messagesArray.push(message);
        button.parentNode.insertBefore(message, button.nextSibling);

        // Display the message below the button for a specified duration (5 seconds here)
        setTimeout(function () {
            message.remove();
            messagesArray = messagesArray.filter(item => item !== message);
        }, 5000);
    } else if (msg.hasOwnProperty("telemetry")) {
        let gyroX = document.getElementById("gyroX");
        let gyroY = document.getElementById("gyroY");
        let gyroZ = document.getElementById("gyroZ");

        let accX = document.getElementById("accX");
        let accY = document.getElementById("accY");
        let accZ = document.getElementById("accZ");

        gyroX.textContent = msg.telemetry.sensors_imu.gyro.x.toFixed(3);
        gyroY.textContent = msg.telemetry.sensors_imu.gyro.y.toFixed(3);
        gyroZ.textContent = msg.telemetry.sensors_imu.gyro.z.toFixed(3);

        accX.textContent = msg.telemetry.sensors_imu.acc.x.toFixed(3);
        accY.textContent = msg.telemetry.sensors_imu.acc.y.toFixed(3);
        accZ.textContent = msg.telemetry.sensors_imu.acc.z.toFixed(3);
    } else if (msg.hasOwnProperty("pingPong")) {
        let pingPongBtn = document.getElementById("pingPong")
        pingPongBtn.textContent = msg.pingPong
    } else if (msg.hasOwnProperty("triggerWebviews")) {
        alert(document.title + " received the trigger")
    } else {
        // Handle other messages here
    }
}

// Send the message to the drone with WebSocket
function sendMsg(msg) {
    console.log('Send: ' + JSON.stringify(msg));
    WebSocketHelper.sendMessage(JSON.stringify(msg));
}

function onClickSendMsg() {
    var message = document.getElementById("messageInput").value;

    if (message.trim() !== "") {
        var msg = { cmd: "sendMsg", value: message };
        sendMsg(msg);
        document.getElementById("messageInput").value = "";
        document.getElementById("messageInput").blur();
    } else {
        alert("Message cannot be empty!");
    }
}

// Add event listener to send the message when ENTER is pressed
document.getElementById("messageInput").addEventListener("keypress", function (event) {
    if (event.key === "Enter") {
        onClickSendMsg(); // Call the function to send the message
    }
});

function onClickPingPong() {
    var msg = { cmd: "pingPong", value: document.getElementById("pingPong").textContent };
    sendMsg(msg);
}

function onClickTriggerWebviews() {
    var msg = { cmd: "triggerWebviews" };
    sendMsg(msg);
}
