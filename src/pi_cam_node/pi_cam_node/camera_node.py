import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Publisher for raw image topic
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Open the default USB camera
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("❌ Could not open /dev/video0. Is the camera connected?")
        else:
            self.get_logger().info("✅ Camera initialized successfully.")

        # Timer to publish at ~20 FPS
        self.timer = self.create_timer(0.05, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠ Failed to read frame from camera.")
            return

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
