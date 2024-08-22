import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class YahboomG1Teleop(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Motor pin definition
        global IN1
        global IN2
        global IN3
        global IN4
        global ENA
        global ENB
        global CarSpeedControl
        global delaytime
        global pwm_ENA
        global pwm_ENB
        global pwm_FrontServo
        global pwm_UpDownServo
        global pwm_LeftRightServo
        global pwm_rled
        global pwm_gled
        global pwm_bled
        
        IN1 = 20
        IN2 = 21
        IN3 = 19
        IN4 = 26
        ENA = 16
        ENB = 13
        CarSpeedControl = 20
        delaytime = 0.2

        GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)

        #Set the pwm pin and frequency to 2000hz
        pwm_ENA = GPIO.PWM(ENA, 2000)
        pwm_ENB = GPIO.PWM(ENB, 2000)
        pwm_ENA.start(0)
        pwm_ENB.start(0)

        # Subscription to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info("Motor control node has been started.")

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocity from the Twist message
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        if linear_velocity > 0:
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
            GPIO.output(IN3, GPIO.HIGH)
            GPIO.output(IN4, GPIO.LOW)
            pwm_ENA.ChangeDutyCycle(CarSpeedControl)
            pwm_ENB.ChangeDutyCycle(CarSpeedControl)
            time.sleep(delaytime)
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN3, GPIO.LOW)
        elif linear_velocity < 0:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.HIGH)
            pwm_ENA.ChangeDutyCycle(CarSpeedControl)
            pwm_ENB.ChangeDutyCycle(CarSpeedControl)
            time.sleep(delaytime)
            GPIO.output(IN2, GPIO.LOW)
            GPIO.output(IN4, GPIO.LOW)
        else:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW)
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.LOW)

        if angular_velocity > 0:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
            GPIO.output(IN3, GPIO.HIGH)
            GPIO.output(IN4, GPIO.LOW)
            pwm_ENA.ChangeDutyCycle(CarSpeedControl)
            pwm_ENB.ChangeDutyCycle(CarSpeedControl)
            time.sleep(delaytime)
            GPIO.output(IN2, GPIO.LOW)
            GPIO.output(IN3, GPIO.LOW)
        elif angular_velocity < 0:
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.HIGH)
            pwm_ENA.ChangeDutyCycle(CarSpeedControl)
            pwm_ENB.ChangeDutyCycle(CarSpeedControl)
            time.sleep(delaytime)
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN4, GPIO.LOW)
        else:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW)
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.LOW)

        self.get_logger().info(f"Linear velocity: {linear_velocity}, Angular velocity: {angular_velocity}")

    def __del__(self):
        pwm_ENA.stop()
        pwm_ENB.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = YahboomG1Teleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

