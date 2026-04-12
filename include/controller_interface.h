/*
 * Modern Drone Controller Web Interface
 * Professional dashboard with dual joystick control
 * Dark theme with clean layout
 */

const HTML_INTERFACE = R"(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Anveshak Drone Controller</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
            background: linear-gradient(135deg, #0f0f1e 0%, #1a1a2e 100%);
            color: #e0e0e0;
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            overflow: hidden;
        }

        /* ==================== TOP BAR ==================== */
        .top-bar {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 12px 24px;
            background: rgba(0, 0, 0, 0.6);
            border-bottom: 1px solid rgba(255, 107, 107, 0.3);
            backdrop-filter: blur(10px);
        }

        .top-bar-left {
            display: flex;
            align-items: center;
            gap: 16px;
        }

        .menu-btn, .user-btn {
            background: none;
            border: none;
            color: #e0e0e0;
            font-size: 24px;
            cursor: pointer;
            padding: 8px;
            border-radius: 8px;
            transition: all 0.3s ease;
        }

        .menu-btn:hover, .user-btn:hover {
            background: rgba(255, 107, 107, 0.2);
            color: #ff6b6b;
        }

        .username {
            font-size: 16px;
            font-weight: 500;
            color: #b0b0b0;
        }

        .top-bar-center {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 4px;
        }

        .connection-status {
            font-size: 14px;
            font-weight: 600;
            color: #ff6b6b;
            letter-spacing: 0.5px;
        }

        .connection-status.connected {
            color: #51cf66;
        }

        .timer {
            font-size: 32px;
            font-weight: 300;
            font-family: 'Monaco', 'Courier New', monospace;
            color: #ff6b6b;
            border: 2px solid #ff6b6b;
            border-radius: 12px;
            padding: 8px 20px;
            letter-spacing: 2px;
        }

        .top-bar-right {
            display: flex;
            align-items: center;
            gap: 12px;
            font-size: 20px;
        }

        .status-icon {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background: #ff6b6b;
        }

        .status-icon.online {
            background: #51cf66;
            box-shadow: 0 0 8px #51cf66;
        }

        /* ==================== MAIN CONTENT ==================== */
        .main-content {
            display: flex;
            flex: 1;
            gap: 24px;
            padding: 24px;
            overflow: hidden;
        }

        /* ==================== CONTROL PANELS ==================== */
        .control-panel {
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            gap: 16px;
        }

        /* LEFT PANEL - PITCH/ROLL ==================== */
        .left-panel {
            flex: 0 0 auto;
        }

        .joystick-container {
            position: relative;
            width: 200px;
            height: 200px;
            background: rgba(20, 20, 35, 0.8);
            border-radius: 50%;
            border: 3px solid rgba(255, 107, 107, 0.4);
            display: flex;
            align-items: center;
            justify-content: center;
            cursor: grab;
            user-select: none;
            transition: all 0.2s ease;
            box-shadow: 
                0 0 30px rgba(255, 107, 107, 0.2),
                inset 0 0 30px rgba(255, 107, 107, 0.05);
        }

        .joystick-container:active {
            border-color: rgba(255, 107, 107, 0.8);
            box-shadow: 
                0 0 40px rgba(255, 107, 107, 0.4),
                inset 0 0 30px rgba(255, 107, 107, 0.1);
        }

        .joystick-bg {
            position: absolute;
            width: 100%;
            height: 100%;
            display: flex;
            align-items: center;
            justify-content: center;
            opacity: 0.3;
        }

        .joystick-cross {
            position: absolute;
            width: 100%;
            height: 100%;
        }

        .joystick-cross::before,
        .joystick-cross::after {
            content: '';
            position: absolute;
            background: rgba(255, 107, 107, 0.2);
        }

        .joystick-cross::before {
            width: 100%;
            height: 1px;
            top: 50%;
            left: 0;
        }

        .joystick-cross::after {
            width: 1px;
            height: 100%;
            top: 0;
            left: 50%;
        }

        .joystick-knob {
            position: relative;
            width: 60px;
            height: 60px;
            background: radial-gradient(circle at 30% 30%, #ff8585, #ff6b6b);
            border-radius: 50%;
            border: 3px solid rgba(255, 255, 255, 0.3);
            box-shadow: 0 4px 12px rgba(255, 107, 107, 0.4);
            z-index: 10;
            transition: all 0.1s ease;
        }

        .joystick-knob::after {
            content: '';
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            width: 30px;
            height: 30px;
            background: rgba(255, 255, 255, 0.2);
            border-radius: 50%;
        }

        .control-label {
            font-size: 12px;
            font-weight: 600;
            color: #b0b0b0;
            text-transform: uppercase;
            letter-spacing: 1px;
            margin-top: 12px;
        }

        .directional-arrows {
            position: absolute;
            width: 100%;
            height: 100%;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: space-between;
            padding: 12px 0;
            font-size: 20px;
            color: rgba(255, 107, 107, 0.5);
        }

        .arrow-horizontal {
            position: absolute;
            left: 0;
            right: 0;
            top: 50%;
            display: flex;
            justify-content: space-between;
            padding: 0 12px;
            transform: translateY(-50%);
        }

        /* CENTER PANEL - THROTTLE & ALTITUDE ==================== */
        .center-panel {
            flex: 0 0 auto;
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 20px;
        }

        .throttle-container {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 12px;
        }

        .throttle-bar {
            width: 60px;
            height: 300px;
            background: rgba(20, 20, 35, 0.8);
            border: 2px solid rgba(255, 107, 107, 0.4);
            border-radius: 30px;
            position: relative;
            overflow: hidden;
            cursor: grab;
        }

        .throttle-bar:active {
            border-color: rgba(255, 107, 107, 0.8);
        }

        .throttle-fill {
            position: absolute;
            bottom: 0;
            width: 100%;
            height: 0%;
            background: linear-gradient(180deg, #ff6b6b 0%, #ff4757 100%);
            transition: height 0.1s linear;
            border-radius: 28px;
        }

        .throttle-value {
            position: absolute;
            top: 10px;
            width: 100%;
            text-align: center;
            font-size: 14px;
            font-weight: 600;
            color: #ff6b6b;
        }

        .altitude-display {
            width: 180px;
            padding: 20px;
            background: rgba(20, 20, 35, 0.8);
            border: 2px solid rgba(255, 107, 107, 0.3);
            border-radius: 12px;
            text-align: center;
        }

        .altitude-label {
            font-size: 11px;
            color: #888;
            text-transform: uppercase;
            letter-spacing: 1px;
            margin-bottom: 8px;
        }

        .altitude-value {
            font-size: 28px;
            font-weight: 300;
            color: #ff6b6b;
            font-family: 'Monaco', 'Courier New', monospace;
        }

        .altitude-unit {
            font-size: 12px;
            color: #666;
            margin-left: 4px;
        }

        .altitude-indicator {
            width: 100%;
            height: 40px;
            background: rgba(0, 0, 0, 0.4);
            border-radius: 8px;
            margin-top: 12px;
            display: flex;
            align-items: center;
            justify-content: center;
            position: relative;
            overflow: hidden;
        }

        .altitude-bar {
            height: 100%;
            width: 0%;
            background: linear-gradient(90deg, transparent, #ff6b6b, transparent);
            transition: width 0.2s ease;
        }

        /* RIGHT PANEL - YAW ==================== */
        .right-panel {
            flex: 0 0 auto;
        }

        .compass {
            position: relative;
            width: 240px;
            height: 240px;
            background: rgba(20, 20, 35, 0.8);
            border-radius: 50%;
            border: 3px solid rgba(255, 107, 107, 0.4);
            display: flex;
            align-items: center;
            justify-content: center;
            cursor: grab;
            user-select: none;
            transition: all 0.2s ease;
            box-shadow: 
                0 0 30px rgba(255, 107, 107, 0.2),
                inset 0 0 30px rgba(255, 107, 107, 0.05);
        }

        .compass:active {
            border-color: rgba(255, 107, 107, 0.8);
        }

        .compass-center {
            position: relative;
            width: 80px;
            height: 80px;
            background: radial-gradient(circle at 30% 30%, #ff8585, #ff6b6b);
            border-radius: 50%;
            border: 3px solid rgba(255, 255, 255, 0.3);
            box-shadow: 0 4px 12px rgba(255, 107, 107, 0.4);
            display: flex;
            align-items: center;
            justify-content: center;
            z-index: 10;
        }

        .compass-center::before {
            content: '';
            position: absolute;
            width: 6px;
            height: 6px;
            background: rgba(255, 255, 255, 0.4);
            border-radius: 50%;
        }

        .compass-ring {
            position: absolute;
            width: 100%;
            height: 100%;
            border: 1px solid rgba(255, 107, 107, 0.2);
            border-radius: 50%;
        }

        .compass-ring:nth-child(1) {
            width: 60%;
            height: 60%;
            top: 20%;
            left: 20%;
        }

        .compass-markers {
            position: absolute;
            width: 100%;
            height: 100%;
        }

        .compass-marker {
            position: absolute;
            font-size: 14px;
            font-weight: 600;
            color: rgba(255, 107, 107, 0.5);
            width: 30px;
            text-align: center;
        }

        .compass-marker.n {
            top: -20px;
            left: 50%;
            transform: translateX(-50%);
            color: #ff6b6b;
        }

        .compass-marker.s {
            bottom: -20px;
            left: 50%;
            transform: translateX(-50%);
        }

        .compass-marker.e {
            right: -20px;
            top: 50%;
            transform: translateY(-50%);
        }

        .compass-marker.w {
            left: -20px;
            top: 50%;
            transform: translateY(-50%);
        }

        /* BOTTOM CONTROLS ==================== */
        .bottom-controls {
            display: flex;
            justify-content: center;
            align-items: center;
            gap: 24px;
            padding: 24px;
            background: rgba(0, 0, 0, 0.4);
            border-top: 1px solid rgba(255, 107, 107, 0.2);
        }

        .arm-toggle {
            position: relative;
            display: flex;
            align-items: center;
            gap: 12px;
            padding: 12px 24px;
            background: rgba(255, 107, 107, 0.1);
            border: 2px solid rgba(255, 107, 107, 0.4);
            border-radius: 8px;
            cursor: pointer;
            transition: all 0.3s ease;
            font-weight: 600;
            color: #ff6b6b;
        }

        .arm-toggle:hover {
            background: rgba(255, 107, 107, 0.2);
            border-color: rgba(255, 107, 107, 0.6);
        }

        .arm-toggle.armed {
            background: rgba(81, 207, 102, 0.1);
            border-color: rgba(81, 207, 102, 0.6);
            color: #51cf66;
        }

        .toggle-indicator {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: #ff6b6b;
            transition: all 0.3s ease;
        }

        .arm-toggle.armed .toggle-indicator {
            background: #51cf66;
            box-shadow: 0 0 8px #51cf66;
        }

        .connect-btn {
            padding: 12px 32px;
            background: linear-gradient(135deg, #ff6b6b, #ff5252);
            border: none;
            border-radius: 8px;
            color: white;
            font-weight: 600;
            font-size: 16px;
            cursor: pointer;
            transition: all 0.3s ease;
            box-shadow: 0 4px 12px rgba(255, 107, 107, 0.3);
        }

        .connect-btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 6px 16px rgba(255, 107, 107, 0.4);
        }

        .connect-btn:active {
            transform: translateY(0);
        }

        .connect-btn.connected {
            background: linear-gradient(135deg, #51cf66, #40c057);
        }

        /* RIGHT SIDEBAR ==================== */
        .right-sidebar {
            display: flex;
            flex-direction: column;
            gap: 12px;
            flex: 0 0 auto;
        }

        .side-btn {
            width: 48px;
            height: 48px;
            background: rgba(20, 20, 35, 0.8);
            border: 2px solid rgba(255, 107, 107, 0.3);
            border-radius: 8px;
            color: #ff6b6b;
            font-size: 20px;
            cursor: pointer;
            display: flex;
            align-items: center;
            justify-content: center;
            transition: all 0.3s ease;
        }

        .side-btn:hover {
            background: rgba(255, 107, 107, 0.1);
            border-color: rgba(255, 107, 107, 0.6);
        }

        .side-btn:active {
            transform: scale(0.95);
        }

        /* RESPONSIVE ==================== */
        @media (max-width: 1400px) {
            .main-content {
                gap: 16px;
                padding: 16px;
            }

            .joystick-container,
            .compass {
                transform: scale(0.9);
            }
        }

        @media (max-width: 1024px) {
            .main-content {
                flex-direction: column;
                gap: 12px;
            }

            .right-sidebar {
                flex-direction: row;
            }
        }

        /* ANIMATIONS ==================== */
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.6; }
        }

        .pulse {
            animation: pulse 2s ease-in-out infinite;
        }

        /* UTILITY CLASSES ==================== */
        .hidden {
            display: none !important;
        }
    </style>
</head>
<body>
    <!-- TOP BAR -->
    <div class="top-bar">
        <div class="top-bar-left">
            <button class="menu-btn" id="menuBtn">☰</button>
            <button class="user-btn" id="userBtn">👤</button>
            <span class="username">raj_gaurav</span>
        </div>

        <div class="top-bar-center">
            <div class="connection-status" id="connectionStatus">NOT CONNECTED</div>
            <div class="timer" id="timer">00:00</div>
        </div>

        <div class="top-bar-right">
            <span class="status-icon" id="statusIcon"></span>
            <span>📡</span>
        </div>
    </div>

    <!-- MAIN CONTENT -->
    <div class="main-content">
        <!-- LEFT: PITCH/ROLL JOYSTICK -->
        <div class="left-panel control-panel">
            <div class="joystick-container" id="pitchRollJoystick">
                <div class="joystick-cross"></div>
                <div class="directional-arrows">
                    <span>⬆</span>
                    <div class="arrow-horizontal">
                        <span>⬅</span>
                        <span>➡</span>
                    </div>
                    <span>⬇</span>
                </div>
                <div class="joystick-knob" id="pitchRollKnob"></div>
            </div>
            <div class="control-label">PITCH / ROLL</div>
        </div>

        <!-- CENTER: THROTTLE & ALTITUDE -->
        <div class="center-panel">
            <div class="throttle-container">
                <div class="throttle-bar" id="throttleBar">
                    <div class="throttle-value" id="throttleValue">0%</div>
                    <div class="throttle-fill" id="throttleFill"></div>
                </div>
                <span class="control-label">THROTTLE</span>
            </div>

            <div class="altitude-display">
                <div class="altitude-label">Altitude</div>
                <div style="display: flex; align-items: baseline; justify-content: center;">
                    <span class="altitude-value" id="altitudeValue">0.00</span>
                    <span class="altitude-unit">m</span>
                </div>
                <div class="altitude-indicator">
                    <div class="altitude-bar" id="altitudeBar"></div>
                </div>
            </div>
        </div>

        <!-- RIGHT: YAW CONTROL -->
        <div class="right-panel control-panel">
            <div class="compass" id="yawCompass">
                <div class="compass-ring"></div>
                <div class="compass-ring"></div>
                <div class="compass-markers">
                    <div class="compass-marker n">N</div>
                    <div class="compass-marker s">S</div>
                    <div class="compass-marker e">E</div>
                    <div class="compass-marker w">W</div>
                </div>
                <div class="compass-center" id="yawKnob"></div>
            </div>
            <div class="control-label">YAW</div>
        </div>

        <!-- RIGHT SIDEBAR: MENU -->
        <div class="right-sidebar">
            <button class="side-btn" id="homeBtn" title="Home">🏠</button>
            <button class="side-btn" id="settingsBtn" title="Settings">⚙️</button>
            <button class="side-btn" id="cameraBtn" title="Camera">📷</button>
            <button class="side-btn" id="batteryBtn" title="Battery">🔋</button>
            <button class="side-btn" id="statsBtn" title="Statistics">📊</button>
        </div>
    </div>

    <!-- BOTTOM CONTROLS -->
    <div class="bottom-controls">
        <div class="arm-toggle" id="armToggle">
            <div class="toggle-indicator"></div>
            <span>ARM</span>
        </div>
        <button class="connect-btn" id="connectBtn">CONNECT</button>
    </div>

    <script>
        // ==================== GLOBAL STATE ====================
        let droneState = {
            pitch: 0,
            roll: 0,
            throttle: 0,
            yaw: 0,
            altitude: 0,
            isArmed: false,
            isConnected: false,
            time: 0
        };

        // ==================== TIMER ====================
        function updateTimer() {
            droneState.time++;
            const minutes = Math.floor(droneState.time / 60);
            const seconds = droneState.time % 60;
            document.getElementById('timer').textContent = 
                `${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
        }

        setInterval(updateTimer, 1000);

        // ==================== JOYSTICK HANDLER ====================
        function setupJoystick(container, knob, onMove) {
            let isActive = false;
            const rect = container.getBoundingClientRect();
            const centerX = rect.width / 2;
            const centerY = rect.height / 2;
            const maxRadius = rect.width / 2 - 35;

            function getAngle(x, y) {
                return Math.atan2(y - centerY, x - centerX);
            }

            function getDistance(x, y) {
                return Math.hypot(x - centerX, y - centerY);
            }

            container.addEventListener('mousedown', () => isActive = true);
            container.addEventListener('mouseup', () => isActive = false);
            container.addEventListener('mouseleave', () => isActive = false);

            container.addEventListener('mousemove', (e) => {
                if (!isActive) return;

                const rect = container.getBoundingClientRect();
                const x = e.clientX - rect.left;
                const y = e.clientY - rect.top;

                const distance = Math.min(getDistance(x, y), maxRadius);
                const angle = getAngle(x, y);

                const knobX = centerX + Math.cos(angle) * distance;
                const knobY = centerY + Math.sin(angle) * distance;

                knob.style.transform = `translate(${knobX - centerX - 30}px, ${knobY - centerY - 30}px)`;

                onMove({
                    x: (knobX - centerX) / maxRadius,
                    y: (knobY - centerY) / maxRadius
                });
            });
        }

        // Setup Pitch/Roll
        setupJoystick(
            document.getElementById('pitchRollJoystick'),
            document.getElementById('pitchRollKnob'),
            (pos) => {
                droneState.roll = pos.x * 100;
                droneState.pitch = -pos.y * 100;
                sendControl();
            }
        );

        // ==================== YAW HANDLER ====================
        const yawCompass = document.getElementById('yawCompass');
        const yawKnob = document.getElementById('yawKnob');

        yawCompass.addEventListener('click', (e) => {
            const rect = yawCompass.getBoundingClientRect();
            const centerX = rect.left + rect.width / 2;
            const centerY = rect.top + rect.height / 2;
            const angle = Math.atan2(e.clientY - centerY, e.clientX - centerX);
            const degrees = (angle * 180 / Math.PI + 90) % 360;
            
            droneState.yaw = degrees;
            yawKnob.style.transform = `rotate(${degrees}deg)`;
            sendControl();
        });

        // ==================== THROTTLE HANDLER ====================
        const throttleBar = document.getElementById('throttleBar');
        let isThrottleActive = false;

        throttleBar.addEventListener('mousedown', () => isThrottleActive = true);
        document.addEventListener('mouseup', () => isThrottleActive = false);

        throttleBar.addEventListener('mousemove', (e) => {
            if (!isThrottleActive) return;

            const rect = throttleBar.getBoundingClientRect();
            const y = e.clientY - rect.top;
            const percentage = Math.max(0, Math.min(100, 100 - (y / rect.height * 100)));

            droneState.throttle = percentage;
            document.getElementById('throttleFill').style.height = percentage + '%';
            document.getElementById('throttleValue').textContent = Math.round(percentage) + '%';
            sendControl();
        });

        // ==================== ARM TOGGLE ====================
        document.getElementById('armToggle').addEventListener('click', function() {
            droneState.isArmed = !droneState.isArmed;
            this.classList.toggle('armed');
            sendControl();
        });

        // ==================== CONNECT BUTTON ====================
        document.getElementById('connectBtn').addEventListener('click', function() {
            droneState.isConnected = !droneState.isConnected;
            this.classList.toggle('connected');
            this.textContent = droneState.isConnected ? 'DISCONNECT' : 'CONNECT';
            
            const status = document.getElementById('connectionStatus');
            const icon = document.getElementById('statusIcon');
            
            if (droneState.isConnected) {
                status.textContent = 'CONNECTED';
                status.classList.add('connected');
                icon.classList.add('online');
            } else {
                status.textContent = 'NOT CONNECTED';
                status.classList.remove('connected');
                icon.classList.remove('online');
            }
        });

        // ==================== UPDATE ALTITUDE ====================
        function updateAltitude(altitude) {
            document.getElementById('altitudeValue').textContent = altitude.toFixed(2);
            const barWidth = Math.min(100, (altitude / 5) * 100);
            document.getElementById('altitudeBar').style.width = barWidth + '%';
        }

        // ==================== SEND CONTROL DATA ====================
        function sendControl() {
            fetch('/control', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    pitch: Math.round(droneState.pitch),
                    roll: Math.round(droneState.roll),
                    throttle: Math.round(droneState.throttle * 10),
                    yaw: Math.round(droneState.yaw),
                    armed: droneState.isArmed ? 1 : 0
                })
            });
        }

        // ==================== FETCH TELEMETRY ====================
        function updateTelemetry() {
            fetch('/telemetry')
                .then(r => r.json())
                .then(data => {
                    if (data.altitude !== undefined) {
                        updateAltitude(data.altitude);
                    }
                })
                .catch(err => console.log('Telemetry error:', err));
        }

        setInterval(updateTelemetry, 100);

        // ==================== KEYBOARD CONTROLS ====================
        document.addEventListener('keydown', (e) => {
            const speed = 5;
            switch(e.key) {
                case 'ArrowUp': droneState.pitch += speed; break;
                case 'ArrowDown': droneState.pitch -= speed; break;
                case 'ArrowLeft': droneState.roll -= speed; break;
                case 'ArrowRight': droneState.roll += speed; break;
                case 'w': droneState.throttle += speed; break;
                case 's': droneState.throttle -= speed; break;
                case 'a': droneState.yaw -= speed; break;
                case 'd': droneState.yaw += speed; break;
            }
            droneState.pitch = Math.max(-100, Math.min(100, droneState.pitch));
            droneState.roll = Math.max(-100, Math.min(100, droneState.roll));
            droneState.throttle = Math.max(0, Math.min(100, droneState.throttle));
            sendControl();
        });
    </script>
</body>
</html>
)";
