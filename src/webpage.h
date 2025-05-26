const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <title>Table Saw Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f0f0f0;
        }
        .container {
            max-width: 600px;
            margin: 0 auto;
            background-color: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 0 10px rgba(0,0,0,0.1);
        }
        h1 {
            color: #333;
            text-align: center;
        }
        .control-group {
            margin: 20px 0;
            text-align: center;
        }
        input {
            width: 100px;
            padding: 8px;
            font-size: 16px;
            border: 1px solid #ddd;
            border-radius: 4px;
            margin-bottom: 10px;
        }
        button {
            background-color: #4CAF50;
            color: white;
            padding: 10px 20px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            font-size: 16px;
            margin-top: 10px;
        }
        button:hover {
            background-color: #45a049;
        }
        button:disabled {
            background-color: #cccccc;
            cursor: not-allowed;
        }
        .status {
            margin-top: 20px;
            padding: 10px;
            border-radius: 4px;
            text-align: center;
            color: #666;
        }
        .saved-msg {
            color: #4CAF50;
            margin-top: 10px;
            display: none;
        }
        .log-card {
            margin-top: 20px;
            border: 1px solid #ddd;
            border-radius: 4px;
            padding: 10px;
            height: 200px;
            overflow-y: scroll;
            background-color: #f9f9f9;
            font-family: monospace;
            font-size: 0.9em;
            white-space: pre-wrap; /* Allows line breaks and preserves whitespace */
        }
        .log-entry {
            padding: 2px 0;
            border-bottom: 1px dotted #eee;
        }
        .log-entry.error {
            color: red;
            font-weight: bold;
        }
        .log-entry.info {
            color: #333;
        }
        .log-entry.warn {
            color: orange;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Table Saw Control</h1>
        <div class="control-group">
            <label for="forwardDistance">Forward Distance (inches):</label><br>
            <input type="number" id="forwardDistance" value="29.5" step="0.1" min="0" max="100"><br>
            <button id="saveForwardButton" onclick="saveForwardDistance()">Save Forward Distance</button>
            <div id="savedForwardMsg" class="saved-msg">Forward distance saved!</div>
        </div>

        <div class="control-group">
            <label for="homeOffset">Home Offset (inches):</label><br>
            <input type="number" id="homeOffset" value="0.65" step="0.01" min="0" max="5"><br>
            <button id="saveHomeButton" onclick="saveHomeOffset()">Save Home Offset</button>
            <div id="savedHomeMsg" class="saved-msg">Home offset saved!</div>
        </div>

        <div class="control-group">
            <h2>Manual Control</h2>
            <button onclick="sendManualCommand('home')">Home Machine</button><br><br>
            
            <label for="jogDistance">Jog Distance (inches):</label><br>
            <input type="number" id="jogDistance" value="0.1" step="0.1" min="0.1" max="5"><br>
            <button onclick="sendManualCommand('jog_left')">Jog Left</button>
            <button onclick="sendManualCommand('jog_right')">Jog Right</button><br><br>

            <button onclick="sendManualCommand('toggle_left_clamp')">Toggle Left Clamp</button>
            <button onclick="sendManualCommand('toggle_right_clamp')">Toggle Right Clamp</button><br><br>
            <button onclick="sendManualCommand('toggle_align_cylinder')">Toggle Align Cylinder</button><br><br>
            <button onclick="sendManualCommand('router_signal')">Send Router Signal</button><br><br>
            
            <button onclick="sendManualCommand('start_cycle')" style="background-color: #ff6b35; font-weight: bold;">Start Cutting Cycle</button>
        </div>

        <div class="status" id="status">WebSocket: Disconnected</div>
        <div class="log-card" id="logCard">
            <div class="log-entry info">Log display initialized...</div>
        </div>
    </div>
    <script>
        var gateway = `ws://${window.location.hostname}/ws`;
        var websocket;
        var currentForwardDistance = 29.5;
        var currentHomeOffset = 0.65;
        
        window.addEventListener('load', onLoad);

        function onLoad(event) {
            initWebSocket();
            updateSaveButtons();
        }

        function initWebSocket() {
            console.log('Trying to open a WebSocket connection...');
            websocket = new WebSocket(gateway);
            websocket.onopen = onOpen;
            websocket.onclose = onClose;
            websocket.onmessage = onMessage;
        }

        function onOpen(event) {
            console.log('Connection opened');
            document.getElementById('status').innerHTML = 'WebSocket: Connected';
            document.getElementById('status').style.color = '#4CAF50';
            document.getElementById('saveForwardButton').disabled = false;
            document.getElementById('saveHomeButton').disabled = false;
        }

        function onClose(event) {
            console.log('Connection closed');
            document.getElementById('status').innerHTML = 'WebSocket: Disconnected';
            document.getElementById('status').style.color = '#f44336';
            document.getElementById('saveForwardButton').disabled = true;
            document.getElementById('saveHomeButton').disabled = true;
            setTimeout(initWebSocket, 2000);
        }

        function onMessage(event) {
            let data;
            try {
                data = JSON.parse(event.data);
            } catch (e) {
                console.error("Failed to parse WebSocket message:", event.data, e);
                appendLogMessage("Received unparseable message: " + event.data, "error");
                return;
            }

            if (data.type === "distance") {
                let value = parseFloat(data.value);
                if (!isNaN(value)) {
                    currentForwardDistance = value;
                    document.getElementById('forwardDistance').value = value.toFixed(1);
                    updateSaveButtons();
                }
            } else if (data.type === "home_offset") {
                let value = parseFloat(data.value);
                if (!isNaN(value)) {
                    currentHomeOffset = value;
                    document.getElementById('homeOffset').value = value.toFixed(2);
                    updateSaveButtons();
                }
            } else if (data.type === "log") {
                appendLogMessage(data.message, data.level || "info");
            } else {
                 appendLogMessage("Received unknown message type: " + event.data, "warn");
            }
        }

        function appendLogMessage(message, level = "info") {
            const logCard = document.getElementById('logCard');
            const logEntry = document.createElement('div');
            logEntry.classList.add('log-entry', level);
            
            // Create timestamp
            const now = new Date();
            const timestamp = now.toLocaleTimeString('en-US', { 
                hour12: false, 
                hour: '2-digit', 
                minute: '2-digit', 
                second: '2-digit' 
            });
            
            // Sanitize message to prevent XSS - display as text with timestamp
            logEntry.textContent = `[${timestamp}] ${message}`; 
            
            logCard.appendChild(logEntry);
            logCard.scrollTop = logCard.scrollHeight; // Auto-scroll to bottom

            // Optional: Limit number of log entries to prevent performance issues
            const maxLogEntries = 200;
            while (logCard.children.length > maxLogEntries) {
                logCard.removeChild(logCard.firstChild);
            }
        }

        function updateSaveButtons() {
            // Update forward distance button
            let forwardInputValue = parseFloat(document.getElementById('forwardDistance').value);
            let saveForwardButton = document.getElementById('saveForwardButton');
            let isForwardValid = !isNaN(forwardInputValue) && forwardInputValue >= 0 && forwardInputValue <= 100;
            let forwardHasChanged = forwardInputValue !== currentForwardDistance;
            saveForwardButton.disabled = !isForwardValid || !forwardHasChanged;

            // Update home offset button
            let homeInputValue = parseFloat(document.getElementById('homeOffset').value);
            let saveHomeButton = document.getElementById('saveHomeButton');
            let isHomeValid = !isNaN(homeInputValue) && homeInputValue >= 0 && homeInputValue <= 5;
            let homeHasChanged = homeInputValue !== currentHomeOffset;
            saveHomeButton.disabled = !isHomeValid || !homeHasChanged;
        }

        document.getElementById('forwardDistance').addEventListener('input', function() {
            updateSaveButtons();
            document.getElementById('savedForwardMsg').style.display = 'none';
        });

        document.getElementById('homeOffset').addEventListener('input', function() {
            updateSaveButtons();
            document.getElementById('savedHomeMsg').style.display = 'none';
        });

        function saveForwardDistance() {
            let value = parseFloat(document.getElementById('forwardDistance').value);
            if (value >= 0 && value <= 100) {
                let payload = { type: "update_forward_distance", value: value };
                websocket.send(JSON.stringify(payload));
                currentForwardDistance = value;
                updateSaveButtons();
                
                // Show saved message
                let savedMsg = document.getElementById('savedForwardMsg');
                savedMsg.style.display = 'block';
                setTimeout(() => {
                    savedMsg.style.display = 'none';
                }, 2000);
            }
        }

        function saveHomeOffset() {
            let value = parseFloat(document.getElementById('homeOffset').value);
            if (value >= 0 && value <= 5) {
                let payload = { type: "update_home_offset", value: value };
                websocket.send(JSON.stringify(payload));
                currentHomeOffset = value;
                updateSaveButtons();
                
                // Show saved message
                let savedMsg = document.getElementById('savedHomeMsg');
                savedMsg.style.display = 'block';
                setTimeout(() => {
                    savedMsg.style.display = 'none';
                }, 2000);
            }
        }

        function sendManualCommand(command) {
            if (websocket && websocket.readyState === WebSocket.OPEN) {
                let payload = { type: "manual_control", command: command };
                if (command === 'jog_left' || command === 'jog_right') {
                    let jogDist = parseFloat(document.getElementById('jogDistance').value);
                    if (isNaN(jogDist) || jogDist <= 0) {
                        appendLogMessage("Invalid jog distance.", "error");
                        return;
                    }
                    payload.distance = jogDist;
                }
                websocket.send(JSON.stringify(payload));
                appendLogMessage("Sent command: " + command + (payload.distance ? " (" + payload.distance + " inches)" : ""), "info");
            } else {
                appendLogMessage("WebSocket not connected. Cannot send command.", "error");
            }
        }
    </script>
</body>
</html>
)rawliteral";