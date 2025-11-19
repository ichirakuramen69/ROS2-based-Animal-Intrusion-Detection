import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from yolo_interfaces.msg import Detection, Detections


class AnnotatorNode(Node):
    def __init__(self):
        super().__init__('annotator_node')

        self.bridge = CvBridge()
        self.current_image = None
        self.current_detections = []

        self.subscription_image = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.subscription_dets = self.create_subscription(
            Detections, '/yolo/detections', self.detection_callback, 10)

        self.publisher = self.create_publisher(
            Image, '/yolo/annotated_image', 10)

        self.get_logger().info("🟩 Annotator Node Started (new package)")

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.publish_annotated()

    def detection_callback(self, msg):
        self.current_detections = msg.detections
        self.publish_annotated()

    def publish_annotated(self):
        if self.current_image is None:
            return

        img = self.current_image.copy()

        for det in self.current_detections:
            x1, y1, x2, y2 = map(int, [det.xmin, det.ymin, det.xmax, det.ymax])

            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                img,
                f"{det.label} {det.confidence:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

        ros_img = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.publisher.publish(ros_img)


def main(args=None):
    rclpy.init(args=args)
    node = AnnotatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
