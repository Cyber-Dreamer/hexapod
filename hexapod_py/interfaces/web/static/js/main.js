document.addEventListener('DOMContentLoaded', function () {
    // --- Element References ---
    const mainViewContainer = document.getElementById('main-view');
    const previewContainer = document.getElementById('preview-container');
    const vxValue = document.getElementById('vx-value');
    const vyValue = document.getElementById('vy-value');
    const omegaValue = document.getElementById('omega-value');
    const pitchValue = document.getElementById('pitch-value');
    const powerBtn = document.getElementById('power-btn');
    const modeToggleBtn = document.getElementById('mode-toggle-btn');
    const rightJoystickLabel = document.getElementById('right-joystick-label');
    const omegaLabel = document.getElementById('omega-label'); // This is now the <strong> tag
    const locomotionStatusElem = document.getElementById('locomotion-status');

    // Settings Panel
    const settingsBtn = document.getElementById('settings-btn');
    const closeSettingsBtn = document.getElementById('close-settings-btn');
    const settingsPanel = document.getElementById('settings-panel');

    // Sliders
    const bodyHeightSlider = document.getElementById('body-height-slider');
    const standoffSlider = document.getElementById('standoff-slider');
    const stepHeightSlider = document.getElementById('step-height-slider');
    const bodyHeightValue = document.getElementById('body-height-value');
    const aiVisionToggle = document.getElementById('ai-vision-toggle');
    const aiVisionLabel = document.getElementById('ai-vision-label');
    const standoffValue = document.getElementById('standoff-value');
    const stepHeightValue = document.getElementById('step-height-value');

    // --- State ---
    let controlData = { vx: 0, vy: 0, omega: 0, pitch: 0, roll: 0, body_height: 200, standoff: 200, step_height: 40, power: false };
    let controlData = { vx: 0, vy: 0, omega: 0, pitch: 0, roll: 0, body_height: 275, standoff: 200, step_height: 40, power: false };
    let sendInterval = null;
    let sensorData = { imu: {} }; // Shared state for sensor data
    let views = {}; // To hold our view elements

    // --- Event Listeners ---
    settingsBtn.addEventListener('click', () => settingsPanel.classList.add('visible'));
    closeSettingsBtn.addEventListener('click', () => settingsPanel.classList.remove('visible'));


    // --- View Management ---
    function setupViews() {
        const frontCamCanvas = document.createElement('canvas');
        frontCamCanvas.id = 'front-cam-canvas';
        const rearCamCanvas = document.createElement('canvas');
        rearCamCanvas.id = 'rear-cam-canvas';

        views = {
            'front-cam': { id: 'front-cam', el: frontCamCanvas, label: 'Front Cam' },
            'rear-cam': { id: 'rear-cam', el: rearCamCanvas, label: 'Rear Cam' },
        };
        
        // Initialize all views and their websockets at low FPS
        for (const id in views) {
            const view = views[id];
            const previewWrapper = document.createElement('div');
            previewWrapper.className = 'preview-view';
            previewWrapper.appendChild(view.el);

            const label = document.createElement('div');
            label.className = 'preview-label';
            label.textContent = view.label;
            previewWrapper.appendChild(label);
            previewWrapper.onclick = () => switchView(id);
            
            // Store the wrapper for later use
            view.previewWrapper = previewWrapper;

            // Initialize websockets for camera views
            if (id === 'front-cam') setupWebSocketVideo(view.el, 0);
            if (id === 'rear-cam') setupWebSocketVideo(view.el, 1);
        }
        
        // Set initial view
        switchView('front-cam');
    }

    function switchView(mainViewId) {
        // Detach all view elements before re-arranging them
        mainViewContainer.innerHTML = '';
        previewContainer.innerHTML = '';

        for (const id in views) {
            const view = views[id];
            
            // Ensure the canvas is always inside its wrapper first.
            // This prevents the canvas from being orphaned when it was a main view.
            if (!view.previewWrapper.contains(view.el)) {
                view.previewWrapper.prepend(view.el);
            }
            if (id === mainViewId) {
                mainViewContainer.appendChild(view.el);
            } else {
                previewContainer.appendChild(view.previewWrapper);
            }
        }
    }

    // --- WebSocket Video ---
    function setupWebSocketVideo(canvas, cameraId) {
        const ctx = canvas.getContext('2d');
        let ws;
        let noSignalTimeout;
        let noiseInterval;

        function connect() {
            const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            ws = new WebSocket(`${wsProtocol}//${window.location.host}/ws/video/${cameraId}`);
            ws.binaryType = 'blob';

            ws.onopen = () => {
                console.log(`WebSocket connected for camera ${cameraId}`);
                // Set a timeout to show error if no frame is received
                noSignalTimeout = setTimeout(showErrorState, 5000); // Increased timeout
            };

            ws.onmessage = (event) => {
                // First frame received, clear the no-signal timeout and any error state
                clearTimeout(noSignalTimeout);
                clearInterval(noiseInterval);

                createImageBitmap(event.data).then(imageBitmap => {
                    // Set the canvas drawing buffer size to match its display size
                    canvas.width = canvas.clientWidth;
                    canvas.height = canvas.clientHeight;

                    // Calculate the aspect ratio of the video and the canvas
                    const videoAspectRatio = imageBitmap.width / imageBitmap.height;
                    const canvasAspectRatio = canvas.width / canvas.height;
                    let renderWidth = canvas.width;
                    let renderHeight = canvas.height;

                    // Adjust dimensions to maintain aspect ratio (like 'object-fit: cover')
                    if (videoAspectRatio < canvasAspectRatio) {
                        // Video is TALLER than canvas, so we fill the width and crop the height.
                        renderWidth = canvas.height * videoAspectRatio;
                    } else {
                        // Video is WIDER than canvas, so we fill the height and crop the width.
                        renderHeight = canvas.width / videoAspectRatio;
                    }

                    // Clear the canvas before drawing
                    ctx.clearRect(0, 0, canvas.width, canvas.height);

                    // --- Rotate the image 180 degrees ---
                    ctx.save(); // Save the current state
                    ctx.translate(canvas.width / 2, canvas.height / 2); // Move origin to the center of the canvas
                    ctx.rotate(Math.PI); // Rotate by 180 degrees
                    // Draw the image centered on the new, rotated origin
                    ctx.drawImage(imageBitmap, -renderWidth / 2, -renderHeight / 2, renderWidth, renderHeight);
                    ctx.restore(); // Restore the original state (unrotated)


                    // Only draw the overlay if this canvas is the current main view
                    if (canvas.parentElement === mainViewContainer) {
                        drawIMUOverlayIcon(ctx, canvas.width, canvas.height, sensorData.imu);
                    }
                });
            };

            ws.onclose = () => {
                console.log(`WebSocket disconnected for camera ${cameraId}. Reconnecting...`);
                showErrorState();
                setTimeout(connect, 3000); // Attempt to reconnect after 3 seconds
            };

            ws.onerror = (err) => {
                console.error(`WebSocket error for camera ${cameraId}:`, err);
                ws.close(); // This will trigger the onclose handler for reconnection
            };
        }

        function showErrorState() {
            clearInterval(noiseInterval);
            const w = canvas.width || 320;
            const h = canvas.height || 240;
            canvas.width = w;
            canvas.height = h;

            // Draw error icon (a simple broken camera)
            ctx.fillStyle = '#111';
            ctx.fillRect(0, 0, w, h);
            ctx.strokeStyle = '#888';
            ctx.lineWidth = 4;
            ctx.strokeRect(w * 0.3, h * 0.3, w * 0.4, h * 0.4);
            ctx.beginPath();
            ctx.moveTo(w * 0.3, h * 0.3);
            ctx.lineTo(w * 0.7, h * 0.7);
            ctx.stroke();
            ctx.fillStyle = '#888';
            ctx.font = '16px sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText('NO SIGNAL', w / 2, h * 0.85);

            // Draw a single frame of static noise
            drawStaticNoise(w, h);
        }

        function drawStaticNoise(w, h) {
            const noiseData = ctx.createImageData(w, h);
            const buffer = new Uint32Array(noiseData.data.buffer);
            const len = buffer.length;
            // Use a dark grey (0x33) for the noise instead of bright white
            const darkGrey = 0xFF333333; 
            for (let i = 0; i < len; i++) {
                if (Math.random() > 0.5) {
                    buffer[i] = darkGrey;
                }
            }
            ctx.putImageData(noiseData, 0, 0);
        }

        canvas.webSocket = ws;
        connect();
    }

    function drawIMUOverlayIcon(ctx, w, h, imuData) {
        const iconWidth = 120; // Overall width of the combined icon
        const iconHeight = 60; // Overall height of the combined icon
        const padding = 10;
        const iconLeft = padding;
        const iconTop = padding;

        const accel = imuData.accel || { x: 0, y: 0, z: 0 };
        const gyro = imuData.gyro || { x: 0, y: 0, z: 0 };

        // --- Background for the combined icon ---
        ctx.fillStyle = 'rgba(0, 0, 0, 0.6)'; // Semi-transparent black background
        ctx.fillRect(iconLeft, iconTop, iconWidth, iconHeight);

        // --- Acceleration Arrow (Left part of the icon area) ---
        const arrowCenterX = iconLeft + iconWidth * 0.25; // Center of the left quarter
        const arrowCenterY = iconTop + iconHeight / 2;
        const arrowLength = iconHeight * 0.4; // Length of the arrow body

        // Calculate the angle of the apparent gravity vector in the XY plane
        // When the robot is static, the accelerometer measures the negative of the gravity vector.
        // If accel.x is positive, it means the robot is tilted nose-down (gravity pulling forward).
        // If accel.y is positive, it means the robot is tilted left-side-down (gravity pulling left).
        // So, the arrow should point in the direction of (accel.x, accel.y)
        const accelMagnitudeXY = Math.sqrt(accel.x * accel.x + accel.y * accel.y);
        let accelArrowAngle = 0;
        if (accelMagnitudeXY > 0.1) { // Avoid division by zero and noise
            accelArrowAngle = Math.atan2(accel.y, accel.x); // Angle in radians
        }

        ctx.save();
        ctx.translate(arrowCenterX, arrowCenterY);
        ctx.rotate(accelArrowAngle); // Rotate by the calculated angle

        // Draw the arrow body
        ctx.strokeStyle = 'rgba(255, 200, 0, 0.9)'; // Yellow for acceleration
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(-arrowLength * 0.5, 0);
        ctx.lineTo(arrowLength * 0.5, 0);
        ctx.stroke();

        // Draw the arrowhead
        ctx.beginPath();
        ctx.moveTo(arrowLength * 0.5, 0);
        ctx.lineTo(arrowLength * 0.5 - 8, -5);
        ctx.moveTo(arrowLength * 0.5, 0);
        ctx.lineTo(arrowLength * 0.5 - 8, 5);
        ctx.stroke();

        ctx.restore();

        // --- Gyro Sphere (Right part of the icon area) ---
        const gyroCenterX = iconLeft + iconWidth * 0.75 + padding; // Center of the right quarter, with some padding
        const gyroCenterY = iconTop + iconHeight / 2;
        const gyroRadius = iconHeight * 0.3;

        ctx.save();
        ctx.translate(gyroCenterX, gyroCenterY);

        // Draw the outer sphere/circle
        ctx.beginPath();
        ctx.arc(0, 0, gyroRadius, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
        ctx.lineWidth = 1;
        ctx.stroke();

        // Draw an inner indicator for angular velocity (yaw rate)
        // A small line that rotates based on gyro.z
        const yawRate = gyro.z;
        const rotationIndicatorLength = gyroRadius * 0.6;

        // Rotate the inner indicator based on yaw rate
        // A positive yaw rate (gyro.z) means rotating counter-clockwise around Z axis (looking down).
        // So, a positive yaw rate should rotate the arrow counter-clockwise on screen.
        // Scale factor 0.1 is arbitrary for visual effect, adjust as needed.
        ctx.rotate(-yawRate * 0.1); 

        ctx.strokeStyle = 'rgba(0, 255, 0, 0.9)'; // Green for gyro
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(0, -rotationIndicatorLength * 0.5);
        ctx.lineTo(0, rotationIndicatorLength * 0.5);
        ctx.stroke();

        // Draw arrowhead for yaw indicator
        ctx.beginPath();
        ctx.moveTo(0, -rotationIndicatorLength * 0.5);
        ctx.lineTo(-4, -rotationIndicatorLength * 0.5 + 8);
        ctx.moveTo(0, -rotationIndicatorLength * 0.5);
        ctx.lineTo(4, -rotationIndicatorLength * 0.5 + 8);
        ctx.stroke();

        ctx.restore();
    }
    // --- Joystick Setup ---
    const joystickOptions = {
        mode: 'static',
        position: { left: '50%', top: '50%' },
        color: 'dodgerblue',
        size: 150,
        threshold: 0.1,
    };

    // Left Joystick (Movement)
    const joystickLeft = nipplejs.create({ ...joystickOptions, zone: document.getElementById('joystick-zone-left') });
    joystickLeft.on('start end', (evt) => {
        controlData.vx = 0;
        controlData.vy = 0;
        updateJoystickUI();
        if (evt.type === 'start') startSendingInterval();
        else stopSendingInterval();
    }).on('move', (evt, data) => {
        const normalizedDistance = Math.min(data.distance, joystickOptions.size / 2) / (joystickOptions.size / 2);
        controlData.vx = Math.sin(data.angle.radian) * normalizedDistance;
        controlData.vy = Math.cos(data.angle.radian) * normalizedDistance;
        updateJoystickUI();
    });

    // Right Joystick (Rotation)
    const joystickRight = nipplejs.create({ ...joystickOptions, zone: document.getElementById('joystick-zone-right') });
    joystickRight.on('start end', (evt) => {
        controlData.omega = 0;
        controlData.roll = 0;
        controlData.pitch = 0;
        updateJoystickUI();
        if (evt.type === 'start') startSendingInterval();
        else stopSendingInterval();
    }).on('move', (evt, data) => {
        // When locomotion is enabled, X is omega. When disabled, X is roll.
        if (locomotionStatusElem.textContent === 'ENABLED') {
            controlData.omega = -data.vector.x;
            controlData.roll = 0;
        } else {
            controlData.omega = 0;
            controlData.roll = -data.vector.x;
        }
        // NippleJS Y is negative for 'up', so we invert it for positive pitch
        controlData.pitch = -data.vector.y; 
        updateJoystickUI();
    });

    function updateJoystickUI() {
        vxValue.textContent = controlData.vx.toFixed(2);
        vyValue.textContent = controlData.vy.toFixed(2);
        omegaValue.textContent = controlData.omega.toFixed(2);
        pitchValue.textContent = controlData.pitch.toFixed(2);
    }

    function setupSliders() {
        // Initialize slider values from state
        bodyHeightSlider.value = controlData.body_height;
        standoffSlider.value = controlData.standoff;
        stepHeightSlider.value = controlData.step_height;
        updateSliderUI();

        bodyHeightSlider.addEventListener('input', handleSliderChange);
        standoffSlider.addEventListener('input', handleSliderChange);
        stepHeightSlider.addEventListener('input', handleSliderChange);
    }

    function handleSliderChange() {
        controlData.body_height = parseFloat(bodyHeightSlider.value);
        controlData.standoff = parseFloat(standoffSlider.value);
        controlData.step_height = parseFloat(stepHeightSlider.value);
        updateSliderUI();
        // Send an immediate update when a slider is moved
        sendMovementCommand();
    }

    function updateSliderUI() {
        bodyHeightValue.textContent = `${controlData.body_height.toFixed(0)} mm`;
        standoffValue.textContent = `${controlData.standoff.toFixed(0)} mm`;
        stepHeightValue.textContent = `${controlData.step_height.toFixed(0)} mm`;
    }

    // --- Server Communication ---
    function startSendingInterval() {
        if (!sendInterval) {
            sendMovementCommand();
            sendInterval = setInterval(sendMovementCommand, 50); // 20Hz
        }
    }

    function stopSendingInterval() {
        // Use a small timeout to catch if the other joystick is still active
        setTimeout(() => {
            if (joystickLeft.isPressed || joystickRight.isPressed) return;
            clearInterval(sendInterval);
            sendInterval = null;
            sendMovementCommand(); // Send final zero state
        }, 100);
    }

    function sendMovementCommand() {
        fetch('/move', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(controlData),
        }).catch(err => console.error('Move command failed:', err));
    }

    modeToggleBtn.addEventListener('click', () => {
        fetch('/toggle_locomotion', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                // The server responds with the new status, e.g., "locomotion_enabled".
                // We can use this to update the UI immediately instead of waiting for the next sensor data packet.
                const isEnabled = data.status === 'locomotion_enabled';
                updateLocomotionStatusUI(isEnabled);
            })
            .catch(err => console.error('Mode toggle failed:', err));
    });

    powerBtn.addEventListener('click', () => {
        // Immediately update the button to give user feedback
        const isPoweringOn = !controlData.power;
        powerBtn.textContent = isPoweringOn ? 'POWERING ON...' : 'POWERING OFF...';
        powerBtn.disabled = true;

        fetch('/power', { method: 'POST' })
            .then(response => response.json())
            .catch(err => console.error('Power toggle failed:', err));
    });

    aiVisionToggle.addEventListener('change', () => {
        fetch('/toggle_ai_vision', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                const isEnabled = data.status === 'ai_vision_enabled';
                updateAIVisionStatusUI(isEnabled);
            });
    });

    function updatePowerStatusUI(isPoweredOn) {
        controlData.power = isPoweredOn;
        powerBtn.disabled = false;
        if (isPoweredOn) {
            powerBtn.textContent = 'POWER OFF';
            powerBtn.className = 'control-btn stop';
            modeToggleBtn.disabled = false; // Enable mode switching
            document.body.classList.remove('powered-off');
        } else {
            powerBtn.textContent = 'POWER ON';
            powerBtn.className = 'control-btn start';
            modeToggleBtn.disabled = true; // Disable mode switching
            document.body.classList.add('powered-off');
            // When powered off, also ensure locomotion is shown as disabled
            updateLocomotionStatusUI(false);
        }
    }

    function updateLocomotionStatusUI(isEnabled) {
        // This function now updates the text on the mode-toggle-btn
        if (isEnabled) {
            locomotionStatusElem.textContent = 'ENABLED';
            locomotionStatusElem.className = 'enabled';
            modeToggleBtn.textContent = 'WALKING';
            modeToggleBtn.classList.add('active');
        } else {
            locomotionStatusElem.textContent = 'DISABLED';
            locomotionStatusElem.className = 'disabled';
            modeToggleBtn.textContent = 'BODY';
            modeToggleBtn.classList.remove('active');
        }
    }

    function updateAIVisionStatusUI(isEnabled) {
        aiVisionToggle.checked = isEnabled;
        if (isEnabled) {
            aiVisionLabel.textContent = 'AI Vision (ON)';
        } else {
            aiVisionLabel.textContent = 'AI Vision (OFF)';
        }
    }

    // --- Sensor WebSocket ---
    function setupSensorWebSocket() {
        let ws;

        function connect() {
            const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            ws = new WebSocket(`${wsProtocol}//${window.location.host}/ws/sensors`);

            ws.onopen = () => {
                console.log('Sensor WebSocket connected.');
            };

            ws.onmessage = (event) => {
                const data = JSON.parse(event.data);
                handleSensorData(data);
            };

            ws.onclose = () => {
                console.log('Sensor WebSocket disconnected. Reconnecting...');
                setTimeout(connect, 3000); // Attempt to reconnect after 3 seconds
            };

            ws.onerror = (err) => {
                console.error('Sensor WebSocket error:', err);
                ws.close(); // This will trigger the onclose handler for reconnection
            };
        }

        connect();
    }

    function handleSensorData(data) {
        // Store sensor data in the shared state
        const { power_on, locomotion_enabled, ai_vision_enabled, joint_angles } = data;
        sensorData = data; // Store the whole packet

        // Update locomotion status and button appearance
        updatePowerStatusUI(power_on);
        updateLocomotionStatusUI(locomotion_enabled);
        updateAIVisionStatusUI(ai_vision_enabled);

        // Update joystick labels based on locomotion mode
        if (locomotion_enabled) {
            rightJoystickLabel.textContent = 'Rotation (omega)';
            omegaLabel.textContent = 'Omega';
        } else {
            rightJoystickLabel.textContent = 'Tilt (roll/pitch)';
            omegaLabel.textContent = 'Roll';
        }

    }

    // --- Initialization ---
    function init() {
        console.log('Initializing web interface...');
        setupViews();
        setupSliders();
        setupSensorWebSocket(); // Connect to the sensor data stream
    }

    init();
});