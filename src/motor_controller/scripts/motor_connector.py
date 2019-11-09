from adafruit_servokit import ServoKit
import board
import busio
import rospy

class MotorConnector:
    SERVO_TOPIC = "servo_topic"
    MOTOR_TOPIC = "motor_topic"
    BREAK_TOPIC = "break_topic"

    def __init__(self, servo_index = 0, motor_index = 1, i2c_index = 1):
        rospy.loginfo("Initializing MotorController, servo_index {}, motor index {}, i2c_index {}".format(servo_index, motor_index, i2c_index))
        self.servo_index = servo_index
        self.motor_index = motor_index
        self.i2c_index = i2c_index
        self.i2c_bus = None
        self.servos_kit = None
        self.initialized = False
        self.channels = 16
        self.straight_angle = 76
        self.max_angle = 30
        self.min_speed = .09

        self.try_connecting_driver()

    def try_connecting_driver(self):
        rospy.loginfo("Initializing Servos")
        if self.i2c_index == 0:
            self.i2c_bus = (busio.I2C(board.SCL_0, board.SDA_0))
        else:
            self.i2c_bus = (busio.I2C(board.SCL_1, board.SDA_1))
        rospy.loginfo("Initializing ServoKit")
        try:
            self.kit = ServoKit(channels=self.channels, i2c=self.i2c_bus)
        except:
            rospy.loginfo("Failed to initialize MotorConnector, i2c index {}, msg {}".format(self.i2c_index, e))
            return
        self.initialized = True
        rospy.loginfo("Initialized MotorConnector, i2c index {}".format(self.i2c_index))
        
        self.set_turning_angle(0)
        self.motor_break()

    def set_turning_angle(self, angle):
        if not self._check_index(self.servo_index):
            return

        if not -30 <= angle <= 30:
            rospy.loginfo("Angle should be in the range [-30, 30]")
            return

        self.kit.servo[self.servo_index].angle = self.straight_angle + angle

    def motor_break(self):
        if not self._check_index(self.motor_index):
            return

        self.kit.continuous_servo[self.motor_index].throttle = -1

    def set_speed(self, speed):
        if not self._check_index(self.motor_index):
            return

        if not 0 <= speed <= 100:
            rospy.loginfo("Speed should be in the range [0, 1]")
            return

        # set 0 explicitly
        target_speed = 0 if speed < .1 else self.min_speed + float(speed) / 100 * (1-self.min_speed)

        if not 0 < self.min_speed <= 1:
            rospy.loginfo("min_speed should be in the range [0, 1]")
            return

        self.kit.continuous_servo[self.motor_index].throttle = target_speed


    def _check_index(self, index):
        if not self.initialized:
            rospy.loginfo('MotorConnector not initialized')
            return False

        if not 0 <= index < self.channels:
            rospy.loginfo("Typed invalid index of the motor")
            return False

        return True

    def servo_callback(self, angle):
        self.set_turning_angle(angle.data)

    def motor_callback(self, throttle):
        self.set_speed(throttle.data)

    def break_callback(self, is_breaking):
        if is_breaking.data:
            self.motor_break()