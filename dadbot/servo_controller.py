import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import pigpio

class ServoController(Node):
         
    def __init__(self):
        super().__init__('servo_controller')

        # Initialize pigpio and set the GPIO pin for the servo
        global pi
        pi = pigpio.pi()

        global pwm_UpDownServo
        global pwm_LeftRightServo
        global panPosition
        global ServoLeftRightPin
        global ServoUpDownPin
        global MinPosition
        global MaxPosition

        ServoLeftRightPin = 9
        ServoUpDownPin = 11
        MinPosition = 20
        MaxPosition = 160
        self.tiltPosition = 90
        self.panPosition = 90

        pulse_width = (500 + (self.tiltPosition * 2000 / 180))  # Convert angle to pulse width
        pi.set_servo_pulsewidth(ServoLeftRightPin, pulse_width)
        pi.set_servo_pulsewidth(ServoUpDownPin, pulse_width)

        self.subscription = self.create_subscription(
            Twist,
            '/servo/cmd_vel',
            self.subscription_callback,
            10
        )

        self.get_logger().info('Servo initialized to position 0')


    def subscription_callback(self, msg):
        self.get_logger().info(f'{msg.linear.x}')
        self.get_logger().info(f'{msg.angular.z}')

        movementAmount = 1

        if msg.linear.x > 0:
            self.tiltPosition = max(MinPosition, min(self.tiltPosition + movementAmount, MaxPosition))
            pulse_width = (500 + (self.tiltPosition * 2000 / 180))  # Convert angle to pulse width
            pi.set_servo_pulsewidth(ServoUpDownPin, pulse_width)
        elif msg.linear.x < 0:
            self.tiltPosition = max(MinPosition, min(self.tiltPosition - movementAmount, MaxPosition))
            pulse_width = (500 + (self.tiltPosition * 2000 / 180))  # Convert angle to pulse width
            pi.set_servo_pulsewidth(ServoUpDownPin, pulse_width)

        if msg.angular.z > 0:
            self.panPosition = max(MinPosition, min(self.panPosition + movementAmount, MaxPosition))
            pulse_width = (500 + (self.panPosition * 2000 / 180))  # Convert angle to pulse width
            pi.set_servo_pulsewidth(ServoLeftRightPin, pulse_width)
        elif msg.angular.z < 0:
            self.panPosition = max(MinPosition, min(self.panPosition - movementAmount, MaxPosition))
            pulse_width = (500 + (self.panPosition * 2000 / 180))  # Convert angle to pulse width
            pi.set_servo_pulsewidth(ServoLeftRightPin, pulse_width)

    def __del__(self):
        self.get_logger().info('Cleaning up pigpio pins.')
        pi.set_servo_pulsewidth(ServoLeftRightPin, 0)  # Turn off the servo
        pi.set_servo_pulsewidth(ServoUpDownPin, 0)  # Turn off the servo
        pi.stop()  # Stop the pigpio instance


def main(args=None):
    rclpy.init(args=args)
    servo_controller = ServoController()
    rclpy.spin(servo_controller)
    servo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

