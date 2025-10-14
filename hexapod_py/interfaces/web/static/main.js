
document.addEventListener('DOMContentLoaded', () => {
    const modeSelector = document.getElementById('mode');
    const buttons = document.querySelectorAll('.controls button');
    const camera1 = document.getElementById('camera1');
    const camera2 = document.getElementById('camera2');
    const gpsData = document.getElementById('gps-data');
    const gyroData = document.getElementById('gyro-data');

    let mode = modeSelector.value;

    modeSelector.addEventListener('change', () => {
        mode = modeSelector.value;
        // Here you could send the mode to the backend if needed
        console.log(`Mode changed to: ${mode}`);
    });

    buttons.forEach(button => {
        button.addEventListener('click', () => {
            const direction = button.id;
            fetch(`/move/${direction}`)
                .then(response => response.text())
                .then(text => console.log(text));
        });
    });

    function updateSensorData() {
        fetch('/sensor_data')
            .then(response => response.json())
            .then(data => {
                gpsData.textContent = JSON.stringify(data.gps_data);
                gyroData.textContent = JSON.stringify(data.gyro_data);
            });
    }

    function updateCameraFeeds() {
        // Update camera feeds by setting the src attribute
        // The backend will provide the video streams at these URLs
        camera1.src = "/video_feed/0";
        camera2.src = "/video_feed/1";
    }

    // Update sensor data every 5 seconds
    setInterval(updateSensorData, 5000);

    // Initial updates
    updateSensorData();
    updateCameraFeeds();
});


