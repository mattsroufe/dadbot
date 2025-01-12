const ws = new WebSocket('ws://localhost:8080/ws');
const messages = document.getElementById('messages');
const input = document.getElementById('input');

// Store mapping of client IP to gamepad data
const gamepadMappings = {};  // key: client IP, value: array of gamepad data

ws.onmessage = function(event) {
    messages.value += event.data + '\n';
};

function sendMessage() {
    const message = input.value;
    if (message) {
	ws.send(message);
	input.value = '';
    }
}

ws.onclose = function() {
    messages.value += 'WebSocket connection closed\n';
};

const gamepadInfoContainer = document.getElementById("gamepad-info");
const gamepads = {};

// Send gamepad data to the server
function sendGamepadData() {
    const connectedGamepads = navigator.getGamepads();

    // Collect the status of each connected gamepad
    const gamepadData = connectedGamepads.map((gamepad) => {
	return [
	    Math.round(-255 + (gamepad.axes[1] - -1) * (255 - -255) / (1 - -1)),
	    Math.round(0 + (gamepad.axes[2] - -1) * (180 - 0) / (1 - -1))
	] 
    })

    // Send the data to the server (replace with WebSocket or AJAX as per your server setup)
    // fetch("/update_gamepads", {
    //     method: "POST",
    //     headers: {
    //         "Content-Type": "application/json"
    //     },
    //     body: JSON.stringify(gamepadData)
    // });
    if (ws && ws.readyState === WebSocket.OPEN) {
	ws.send(JSON.stringify(gamepadData));
    }
}

// Update the gamepad info displayed on the webpage
function updateGamepadInfo() {
    gamepadInfoContainer.innerHTML = '';
    for (const gamepadId in gamepads) {
	let gamepad = gamepads[gamepadId];
	gamepad = {
	    id: gamepad.id,
	    axes: [gamepad.axes[1].toFixed(1), gamepad.axes[2].toFixed(1)],
	    buttons: gamepad.buttons.map(button => button.pressed)
	}
	const div = document.createElement("div");
	div.classList.add("gamepad");
	div.innerHTML = `
	    <h3>Gamepad: ${gamepad.id}</h3>
	    <p>Axes: ${gamepad.axes.join(", ")}</p>
	    <p>Buttons: ${gamepad.buttons.join(", ")}</p>
	`;
	gamepadInfoContainer.appendChild(div);
    }
}

// Poll for gamepad status every 100ms
function pollGamepads() {
    sendGamepadData();
    updateGamepadInfo();
    requestAnimationFrame(pollGamepads);
}

// Start polling when gamepads are connected
window.addEventListener("gamepadconnected", (event) => {
    const gamepad = event.gamepad;
    gamepads[gamepad.index] = gamepad;
    console.log("Gamepad connected:", gamepad.id);
});

// Handle gamepad disconnection
window.addEventListener("gamepaddisconnected", (event) => {
    const gamepad = event.gamepad;
    delete gamepads[gamepad.index];
    console.log("Gamepad disconnected:", gamepad.id);
});

// Start the polling loop
pollGamepads();

