const ws = new WebSocket('ws://localhost:8080/ws');

ws.onmessage = function(event) {
};

function sendMessage() {
    ws.send(message);
}

ws.onclose = function() {
};

const gamepads = {};

// Send gamepad data to the server
function updateGamepadInfo() {
    const connectedGamepads = navigator.getGamepads();

    // Collect the status of each connected gamepad
    const gamepadData = connectedGamepads.map((gamepad) => {
        const gamepadInfoContainer = document.getElementById(`gamepad-info-${gamepad.index}`);
        gamepadInfoContainer.innerHTML = '';

	gamepadAxes = [gamepad.axes[1].toFixed(1), gamepad.axes[2].toFixed(1)];
	gamepadButtons = gamepad.buttons.map(button => +button.pressed)

	gamepadInfoContainer.innerHTML = `
	    <h3>${gamepad.id}</h3>
	    <p>Axes: ${gamepadAxes.join(", ")}</p>
	    <p>Buttons: ${gamepadButtons.join(", ")}</p>
	`;

	const result = [
	    Math.round(-255 + (gamepad.axes[1] - -1) * (255 - -255) / (1 - -1)),
	    Math.round(0 + (gamepad.axes[2] - -1) * (180 - 0) / (1 - -1))
	]

	return result;
    })

    console.log(JSON.stringify(gamepadData))

    if (ws && ws.readyState === WebSocket.OPEN) {
	ws.send(JSON.stringify(gamepadData));
    }
}

// Poll for gamepad status every 100ms
function pollGamepads() {
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

