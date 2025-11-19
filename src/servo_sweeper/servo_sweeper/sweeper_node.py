import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time

class ServoSweeper(Node):
    def __init__(self):
        super().__init__("servo_sweeper")

        self.servo_pin = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(0)

        self.step_angle = 45
        self.min_angle = 0
        self.max_angle = 180
        self.current_angle = 0
        self.direction = 1 

        self.get_logger().info("🔄 Servo Sweeper Node Started")

        # --- FIX 1: Move to start position immediately ---
        self.move_servo(self.current_angle)
        
        # Run every 2 seconds
        self.timer = self.create_timer(2.0, self.sweep_step)

    def angle_to_duty(self, angle):
        return 2.5 + (angle / 180.0) * 10.0

    def move_servo(self, angle):
        """Helper function to handle the physical movement and jitter fix"""
        duty = self.angle_to_duty(angle)
        self.pwm.ChangeDutyCycle(duty)
        self.get_logger().info(f"➡️ Moving servo to {angle}°")
        
        # Note: strictly speaking, sleeping in a callback is blocking, 
        # but required here for the 'jitter fix' logic on SG90 servos.
        time.sleep(0.4) 
        self.pwm.ChangeDutyCycle(0) 

    def sweep_step(self):
        # Calculate NEXT angle
        next_angle = self.current_angle + (self.direction * self.step_angle)

        # Check Limits
        if next_angle >= self.max_angle:
            next_angle = self.max_angle
            self.direction = -1
            self.get_logger().warn("↩️ Reversing direction (max)")
        elif next_angle <= self.min_angle:
            next_angle = self.min_angle
            self.direction = 1
            self.get_logger().warn("↪️ Reversing direction (min)")

        # Update state and move
        self.current_angle = next_angle
        self.move_servo(self.current_angle)

    def destroy_node(self):
        self.get_logger().info("🛑 Cleaning up GPIO...")
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoSweeper()

    # --- FIX 3: Safer cleanup on Ctrl+C ---
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
