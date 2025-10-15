document.addEventListener('DOMContentLoaded', function () {
    const joystickZone = document.getElementById('joystick-zone');
    const vxValue = document.getElementById('vx-value');
    const vyValue = document.getElementById('vy-value');
    const imuDataElem = document.getElementById('imu-data');
    const locomotionToggleBtn = document.getElementById('locomotion-toggle-btn');
    const locomotionStatusElem = document.getElementById('locomotion-status');

    let controlData = { vx: 0, vy: 0, omega: 0 };
    let sendInterval = null;

    // --- NippleJS Joystick Setup ---
    const joystickOptions = {
        zone: joystickZone,
        mode: 'static',
        position: { left: '50%', top: '50%' },
        color: 'dodgerblue',
        size: 150,
    };

    const manager = nipplejs.create(joystickOptions);

    manager.on('start', () => {
        // Start sending data to the server at 20Hz
        if (!sendInterval) {
            sendInterval = setInterval(sendMovementCommand, 50); // 20Hz
        }
    });

    manager.on('move', (evt, data) => {
        if (data.distance > 0) {
            const angleRad = data.angle.radian;
            const force = Math.min(data.force, 1.0); // Clamp force to max 1.0

            // Map joystick position to vx, vy, and omega
            // Y-axis on joystick controls forward/backward (vx)
            // X-axis on joystick controls strafing (vy)
            // We'll use a separate mechanism for turning (omega) for simplicity,
            // but for now, we can map it to X-axis as well.
            controlData.vx = Math.sin(angleRad) * force;
            controlData.vy = Math.cos(angleRad) * force;
            
            // Let's use X-axis for turning when strafing is minimal
            // This is a simple mapping, can be improved.
            controlData.omega = -Math.cos(angleRad) * force;

            // Update UI
            vxValue.textContent = controlData.vx.toFixed(2);
            vyValue.textContent = controlData.vy.toFixed(2);
            document.getElementById('omega-value').textContent = controlData.omega.toFixed(2);
        }
    });

    manager.on('end', () => {
        // Stop sending data and reset values
        if (sendInterval) {
            clearInterval(sendInterval);
            sendInterval = null;
        }
        controlData = { vx: 0, vy: 0, omega: 0 };
        sendMovementCommand(); // Send one last command to stop movement

        // Reset UI
        vxValue.textContent = '0.00';
        vyValue.textContent = '0.00';
        document.getElementById('omega-value').textContent = '0.00';
    });

    // --- Server Communication ---
    function sendMovementCommand() {
        fetch('/move', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(controlData),
        }).catch(err => console.error('Failed to send movement command:', err));
    }

    // --- Locomotion Toggle Button ---
    locomotionToggleBtn.addEventListener('click', () => {
        fetch('/toggle_locomotion', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                console.log('Locomotion toggled:', data.status);
                // The status will be updated by the sensor poll
            })
            .catch(err => console.error('Failed to toggle locomotion:', err));
    });

    // --- Sensor and Status Polling ---
    function pollSensorData() {
        fetch('/sensor_data')
            .then(response => response.json())
            .then(data => {
                // Update IMU data
                imuDataElem.textContent = JSON.stringify(data.imu, null, 2);

                // Update locomotion status and button appearance
                if (data.locomotion_enabled) {
                    locomotionStatusElem.textContent = 'ENABLED';
                    locomotionStatusElem.className = 'enabled';
                    locomotionToggleBtn.textContent = 'STOP';
                    locomotionToggleBtn.className = 'control-btn stop';
                } else {
                    locomotionStatusElem.textContent = 'DISABLED';
                    locomotionStatusElem.className = 'disabled';
                    locomotionToggleBtn.textContent = 'START';
                    locomotionToggleBtn.className = 'control-btn start';
                }
            })
            .catch(err => {
                imuDataElem.textContent = 'Error fetching sensor data.';
                console.error('Sensor poll error:', err);
            });
    }

    // Poll for sensor data every 500ms
    setInterval(pollSensorData, 500);
    pollSensorData(); // Initial call
});