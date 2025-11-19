import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, Response, render_template
import threading

app = Flask(__name__)
latest_frame = None


class WebDashboard(Node):
    def __init__(self):
        super().__init__("web_dashboard")

        self.bridge = CvBridge()
        self.create_subscription(Image, "/yolo/annotated_image", self.callback, 10)

        self.get_logger().info("🌐 Web Dashboard Started at http://<IP>:5000")

    def callback(self, msg):
        global latest_frame
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        ret, jpeg = cv2.imencode(".jpg", frame)
        latest_frame = jpeg.tobytes()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video_feed")
def video_feed():
    def generate():
        global latest_frame
        while True:
            if latest_frame is not None:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" +
                    latest_frame +
                    b"\r\n"
                )
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


def start_flask():
    app.run(host="0.0.0.0", port=5001, threaded=True)


def main(args=None):
    rclpy.init(args=args)

    node = WebDashboard()

    flask_thread = threading.Thread(target=start_flask)
    flask_thread.daemon = True
    flask_thread.start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
