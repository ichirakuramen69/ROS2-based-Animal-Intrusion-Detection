import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

from yolo_interfaces.msg import Detection, Detections


class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")
        self.conf_threshold = 0.75

        self.sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(
            Detections,
            "/yolo/detections",
            10
        )

        self.get_logger().info("🚀 YOLOv8 Detector Node Started!")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(frame, verbose=False)

        dets_msg = Detections()
        dets_msg.detections = []

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                label = self.model.names[cls]

                if conf < self.conf_threshold:
                    continue

                det = Detection()
                det.xmin = float(box.xyxy[0][0])
                det.ymin = float(box.xyxy[0][1])
                det.xmax = float(box.xyxy[0][2])
                det.ymax = float(box.xyxy[0][3])
                det.confidence = conf
                det.label = label

                dets_msg.detections.append(det)

                self.get_logger().info(f"Detected {label} ({conf:.2f})")

        self.pub.publish(dets_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
