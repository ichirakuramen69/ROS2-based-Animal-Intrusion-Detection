# telegram_alert/telegram_alert/alert_node.py
import rclpy
from rclpy.node import Node
from yolo_interfaces.msg import Detections
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import requests
import time
import io

class TelegramAlertNode(Node):
    def __init__(self):
        super().__init__('telegram_alert_node')

        # ---- CONFIG: put your token & chat id here ----
        self.bot_token = "YOUR_BOT_TOKEN_HERE"
        self.chat_id = "YOUR_CHAT_ID_HERE"
        # ------------------------------------------------

        # Labels considered "animals"
        self.animal_labels = {
            'elephant', 'cow', 'dog', 'cat', 'horse', 'goat', 'sheep',
            'buffalo', 'deer', 'boar', 'bear', 'rabbit', 'monkey'
        }

        # Person is handled specially
        self.critical_labels = {'person', 'elephant'}

        # Cooldown in seconds per label for normal alerts
        self.cooldown_seconds = 30.0
        # Cooldown for critical alerts (person/elephant)
        self.critical_cooldown_seconds = 60.0

        # Track last sent times: dict label -> timestamp
        self._last_sent = {}
        self._lock = threading.Lock()

        # CvBridge + image cache
        self.bridge = CvBridge()
        self.latest_annotated_image = None
        self.img_lock = threading.Lock()

        # Subscriptions:
        #  - get Detections messages
        self.sub = self.create_subscription(
            Detections,
            '/yolo/detections',
            self.detections_cb,
            10
        )
        # - cache annotated image (optional, used for attaching to messages)
        self.img_sub = self.create_subscription(
            Image,
            '/yolo/annotated_image',
            self.annotated_image_cb,
            5
        )

        self.get_logger().info("📨 Telegram Alert Node Started")

    # ---- Image callback: cache latest annotated image ----
    def annotated_image_cb(self, msg: Image):
        try:
            # Convert to BGR image and then to JPEG bytes
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # encode as JPEG in memory
            import cv2
            ret, jpeg = cv2.imencode('.jpg', cv_image)
            if ret:
                with self.img_lock:
                    self.latest_annotated_image = jpeg.tobytes()
        except Exception as e:
            self.get_logger().warn(f"Failed to convert/cache annotated image: {e}")

    # ---- Detections callback ----
    def detections_cb(self, msg: Detections):
        # quick check
        if not msg.detections:
            return

        # Count detections and handle each
        total = len(msg.detections)
        self.get_logger().info(f"📩 CALLBACK TRIGGERED! Incoming {total} detections")

        for det in msg.detections:
            # label field in your message
            label = (det.label or "").strip().lower()

            # Skip empty labels
            if not label:
                continue

            # Decide if this is an "animal" or "person"
            is_person = (label == 'person')
            is_animal = (label in self.animal_labels)

            # If neither person nor animal -> skip
            if not (is_person or is_animal):
                self.get_logger().debug(f"Skipping non-person/animal detection: {label}")
                continue

            # Prepare level and cooldown
            is_critical = label in self.critical_labels
            now = time.time()
            cooldown = self.critical_cooldown_seconds if is_critical else self.cooldown_seconds

            # Rate limit per label
            with self._lock:
                last = self._last_sent.get(label, 0.0)
                if now - last < cooldown:
                    self.get_logger().info(f"Skipping {label} alert due to cooldown ({now-last:.1f}s).")
                    continue
                # mark now
                self._last_sent[label] = now

            # Build message (aesthetic)
            bbox_str = (f"xmin: {det.xmin:.1f}\n"
                        f"ymin: {det.ymin:.1f}\n"
                        f"xmax: {det.xmax:.1f}\n"
                        f"ymax: {det.ymax:.1f}")

            confidence = getattr(det, 'confidence', None)
            if confidence is None:
                # fallback if not present
                try:
                    confidence = float(det.confidence)
                except Exception:
                    confidence = 0.0

            # Compose caption and text. Use Markdown (Telegram) for bold/emphasis.
            if is_critical:
                # Critical formatting
                caption = f"🚨 *CRITICAL ALERT* 🚨\n*{label.upper()}* detected!"
                text = (f"🚨 *CRITICAL ALERT* 🚨\n"
                        f"*Type:* *{label.title()}*\n"
                        f"*Confidence:* `{confidence:.2f}`\n\n"
                        f"*Bounding Box:*\n```\n{bbox_str}\n```")
            else:
                # Normal (animal) formatting
                caption = f"⚠️ Detection: {label.title()} ({confidence:.2f})"
                text = (f"⚠️ *Detection*\n"
                        f"Type: *{label.title()}*\n"
                        f"Confidence: `{confidence:.2f}`\n\n"
                        f"Bounding box:\n```\n{bbox_str}\n```")

            # Send: prefer image attachment with sendPhoto; fallback to sendMessage
            sent = False
            try:
                img_bytes = None
                with self.img_lock:
                    img_bytes = self.latest_annotated_image

                if img_bytes:
                    # send photo with caption (Markdown not fully supported in caption; use parse_mode)
                    url = f"https://api.telegram.org/bot{self.bot_token}/sendPhoto"
                    files = {'photo': ('annotated.jpg', io.BytesIO(img_bytes), 'image/jpeg')}
                    payload = {
                        'chat_id': self.chat_id,
                        'caption': caption,
                        'parse_mode': 'Markdown'
                    }
                    resp = requests.post(url, data=payload, files=files, timeout=10)
                    if resp.ok:
                        sent = True
                        self.get_logger().info(f"📸 Sent photo alert for {label}")
                    else:
                        self.get_logger().warn(f"Photo send failed ({resp.status_code}): {resp.text}")
                # Fallback or if no image
                if not sent:
                    url = f"https://api.telegram.org/bot{self.bot_token}/sendMessage"
                    payload = {
                        'chat_id': self.chat_id,
                        'text': text,
                        'parse_mode': 'Markdown'
                    }
                    resp = requests.post(url, json=payload, timeout=10)
                    if resp.ok:
                        sent = True
                        self.get_logger().info(f"✉️ Sent text alert for {label}")
                    else:
                        self.get_logger().warn(f"Text send failed ({resp.status_code}): {resp.text}")
            except Exception as e:
                self.get_logger().error(f"Failed to send Telegram alert for {label}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TelegramAlertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
