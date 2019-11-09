#!/usr/bin/env python
import rospy
from motor_connector import MotorConnector
from std_msgs.msg import Bool, Int8

node_name = "motor_controller"

def main():
    rospy.init_node(node_name, anonymous=False)
    controller = MotorConnector(servo_index=0, motor_index=1, i2c_index=1)
    rospy.Subscriber(MotorConnector.SERVO_TOPIC, Int8, controller.servo_callback)
    rospy.Subscriber(MotorConnector.MOTOR_TOPIC, Int8, controller.motor_callback)
    rospy.Subscriber(MotorConnector.BREAK_TOPIC, Bool, controller.break_callback)

    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            controller.set_turning_angle(0)


if __name__ == "__main__":
    main()