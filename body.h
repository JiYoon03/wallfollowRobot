const char body[] PROGMEM = R"===(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Robot Control</title>
</head>
<body>
    <h1>ESP32 Robot Control</h1>
    <button onclick="sendCommand('/forward')">Move Forward</button>
    <button onclick="sendCommand('/backward')">Move Backward</button>
    <button onclick="sendCommand('/left')">Turn Left</button>
    <button onclick="sendCommand('/right')">Turn Right</button>
    <button onclick="sendCommand('/stop')">Stop</button>
    <button onclick="sendCommand('/startAutoFollow')">Start Auto Follow</button>
    <button onclick="sendCommand('/stopAutoFollow')">Stop Auto Follow</button>

    <script>
        function sendCommand(command) {
            fetch(command)
                .then(response => response.text())
                .then(data => console.log(data));
        }
    </script>
</body>
</html>
)===";
