# Real-time-railway-track-safety-monitoring-with-Raspberry-Pi-IoT
🚆 An IoT-based real-time railway track monitoring system built on Raspberry Pi 4.
The system integrates LiDAR, RTK GPS, Ultrasonic sensor, MPU6050 vibration sensor, and LED indicators to detect cracks, misalignments, vibrations, and provide precise geolocation.

🔹 Features

  Ultrasonic Sensor → Short-range distance measurement (misalignment detection).

  LiDAR (e.g. RPLIDAR A1/A2, YDLIDAR) → Long-range scanning for obstacles/cracks.

  MPU6050 → Detects abnormal vibrations from tracks.

  RTK GPS (e.g. u-blox ZED-F9P) → Provides centimeter-level accurate positioning of detected issues.

  LED Indicators → Risk visualization:

  🔴 Red → HIGH risk (critical defect, crack, or obstacle).

  🟡 Yellow → MEDIUM risk (possible misalignment/obstacle).

  🟢 Green → LOW risk (safe condition).

  Standalone Operation → Runs automatically on boot using systemd.

  Real-time JSON Logs → Console output includes distance, vibration, GPS, and LiDAR data.

🔧 Hardware Requirements

  Raspberry Pi 4 (2GB/4GB/8GB)

  Sensors & Modules:

  Ultrasonic Sensor (HC-SR04 or equivalent)

  MPU6050 (Accelerometer + Gyro)

  LiDAR (RPLIDAR A1/A2 / YDLIDAR)

  RTK GPS Module (u-blox ZED-F9P or similar)

  3x LEDs (Red, Yellow, Green) with resistors

  Breadboard, jumper wires, power supply

🖥️ Software Requirements

  Raspberry Pi OS (64-bit recommended)

  Python 3.11+

  Required Libraries:

    pip3 install mpu6050-raspberrypi rplidar pynmea2 --break-system-packages
    sudo apt install python3-opencv

🚀 Future Enhancements

  Cloud dashboard for remote monitoring

  AI-based crack detection using Pi Camera

  SMS/Email alerts for authorities

  Integration with railway control systems

