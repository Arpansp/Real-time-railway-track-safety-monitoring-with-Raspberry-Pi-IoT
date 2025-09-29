import time
import json
import RPi.GPIO as GPIO
from mpu6050 import mpu6050
from rplidar import RPLidar
import serial
import pynmea2

# ========== GPIO Setup ==========
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# LED Pins
RED_LED_PIN = 18
YELLOW_LED_PIN = 23
GREEN_LED_PIN = 24

for pin in [RED_LED_PIN, YELLOW_LED_PIN, GREEN_LED_PIN]:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# Ultrasonic Pins
TRIG_PIN = 17
ECHO_PIN = 27
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# MPU6050
mpu = mpu6050(0x68)

# LiDAR Setup (USB/Serial Port may vary, e.g. /dev/ttyUSB0)
lidar = RPLidar('/dev/ttyUSB0')

# RTK GPS Setup (adjust baudrate and port)
gps_serial = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)

# ================= Helper Functions =================
def read_mpu():
    accel = mpu.get_accel_data()
    gyro = mpu.get_gyro_data()
    return accel, gyro

def abnormal_vibration(accel):
    ax, ay, az = accel["x"], accel["y"], accel["z"]
    magnitude = (ax**2 + ay**2 + az**2) ** 0.5
    return magnitude > 15

def measure_distance():
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.05)

    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    pulse_start, pulse_end = 0, 0
    timeout = time.time() + 0.02
    while GPIO.input(ECHO_PIN) == 0 and time.time() < timeout:
        pulse_start = time.time()
    timeout = time.time() + 0.02
    while GPIO.input(ECHO_PIN) == 1 and time.time() < timeout:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)

def read_lidar():
    """Get minimum distance from LIDAR scan"""
    try:
        for scan in lidar.iter_scans(max_buf_meas=500):
            distances = [m[2] for m in scan if m[2] > 0]
            if distances:
                return min(distances) / 10.0  # mm -> cm
    except:
        return None

def read_gps():
    """Parse NMEA RTK GPS data"""
    try:
        line = gps_serial.readline().decode("utf-8", errors="ignore")
        if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
            msg = pynmea2.parse(line)
            return float(msg.latitude), float(msg.longitude), msg.gps_qual
    except:
        return None, None, None
    return None, None, None

def set_leds(risk_level):
    GPIO.output(RED_LED_PIN, risk_level == "HIGH")
    GPIO.output(YELLOW_LED_PIN, risk_level == "MEDIUM")
    GPIO.output(GREEN_LED_PIN, risk_level == "LOW")

# ================= Main Loop =================
try:
    while True:
        # Sensors
        accel, gyro = read_mpu()
        vibration = abnormal_vibration(accel)
        distance = measure_distance()
        lidar_dist = read_lidar()
        lat, lon, fix_quality = read_gps()

        # Risk logic (example: combine vibration + distance + lidar)
        if vibration or (distance < 10) or (lidar_dist and lidar_dist < 50):
            risk = "HIGH"
        elif distance < 20 or (lidar_dist and lidar_dist < 100):
            risk = "MEDIUM"
        else:
            risk = "LOW"

        # LED indication
        set_leds(risk)

        # JSON log
        data = {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "risk": risk,
            "ultrasonic_cm": distance,
            "lidar_cm": lidar_dist,
            "vibration_abnormal": vibration,
            "accel": accel,
            "gyro": gyro,
            "gps_lat": lat,
            "gps_lon": lon,
            "gps_fix_quality": fix_quality
        }
        print(json.dumps(data, indent=2))

        time.sleep(1)

except KeyboardInterrupt:
    print("Exiting...")
    lidar.stop()
    lidar.disconnect()
    GPIO.cleanup()
    gps_serial.close()
