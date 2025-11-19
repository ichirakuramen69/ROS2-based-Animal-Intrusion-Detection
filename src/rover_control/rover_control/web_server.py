import os
import serial
import time
from flask import Flask, send_from_directory
from rclpy.node import Node
import rclpy

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600

# Locate installed package directory
PACKAGE_DIR = os.path.dirname(__file__)
PARENT_DIR = os.path.abspath(os.path.join(PACKAGE_DIR, ".."))
HTML_FILE = os.path.join(PARENT_DIR, "index.html")

app = Flask(__name__)
arduino_ser = None

command_map = {
    'F': 'R',
    'B': 'L',
    'L': 'B',
    'R': 'F',
    'S': 'S',
}

@app.route("/")
def index():
    return send_from_directory(PARENT_DIR, "index.html")

@app.route("/control/<cmd>")
def control(cmd):
    global arduino_ser

    arduino_cmd = command_map.get(cmd)

    if arduino_cmd and arduino_ser:
        try:
            arduino_ser.write(arduino_cmd.encode())
            time.sleep(0.05)
            arduino_ser.readline()
            return "OK"
        except Exception as e:
            return f"Error: {e}", 500
    return "Invalid or unavailable", 400


def main(args=None):
    global arduino_ser
    rclpy.init(args=args)

    # Connect Arduino
    try:
        arduino_ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2.5)
        print("[MAIN] Arduino connected successfully.")
    except Exception as e:
        print(f"[MAIN] Failed to connect: {e}")
        arduino_ser = None

    print("[INFO] Rover control web server running on port 5001")
    app.run(host="0.0.0.0", port=5001, debug=False)


if __name__ == "__main__":
    main()
