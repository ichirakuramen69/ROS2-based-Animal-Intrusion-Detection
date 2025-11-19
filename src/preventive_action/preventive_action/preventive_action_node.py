import rclpy
from rclpy.node import Node
from yolo_interfaces.msg import Detections
import pygame
import os

class PreventiveActionNode(Node):
    def __init__(self):
        super().__init__("preventive_action_node")

        # Load pygame mixer
        pygame.mixer.init()

        # Find sound file inside the package
        sound_path = os.path.join(
            os.path.dirname(__file__),
            "sounds",
            "siren.mp3"
        )

        self.alert_sound = pygame.mixer.Sound(sound_path)

        # Subscribe to YOLO detections
        self.subscription = self.create_subscription(
            Detections,
            "/yolo/detections",
            self.detection_callback,
            10
        )

        self.get_logger().info("🔊 Preventive Action Node Started")

    def detection_callback(self, msg):
        for det in msg.detections:
            if det.label.lower() == "elephant":
                self.get_logger().warn("🐘 Elephant detected! Playing warning sound...")
                self.alert_sound.play()

def main(args=None):
    rclpy.init(args=args)
    node = PreventiveActionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
