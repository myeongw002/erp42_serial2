#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from erp42_serial2.msg import ERP_SET

class Controller:
    def __init__(self, max_speed, max_steer, max_brake):
        self.max_speed = max_speed * 10
        self.max_steer = max_steer
        self.max_brake = max_brake
        self.current_gear = 1
        self.vel = 0
        self.steer = 0
        self.brake = 0
        self.cruise = 0

        self.pub = rospy.Publisher('erp42_set', ERP_SET, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)
        rospy.Subscriber("/cmd_steer", Twist, self.steer_callback)
        rospy.Subscriber("/btn_cruise", Bool, self.cruise_callback)
        rospy.Subscriber("/btn_press0", Bool, self.gear0_callback)
        rospy.Subscriber("/btn_press1", Bool, self.gear1_callback)
        rospy.Subscriber("/btn_press2", Bool, self.gear2_callback)

    def cruise_callback(self, msg):
        if msg.data:
            self.cruise = not self.cruise
            self.current_gear = 0 if self.cruise else 1

    def gear0_callback(self, msg):
        if msg.data:
            self.current_gear = 0
            self.cruise = 0

    def gear1_callback(self, msg):
        if msg.data:
            self.current_gear = 1
            self.cruise = 0

    def gear2_callback(self, msg):
        if msg.data:
            self.current_gear = 2 if self.current_gear != 2 else 1

    def vel_callback(self, msg):
        self.vel = msg.linear.x

    def steer_callback(self, msg):
        self.steer = -msg.angular.z

    def publish_state(self):
        state_msg = ERP_SET()
        state_msg.set_steer = int(self.steer * self.max_steer)

        if self.cruise:
            if self.vel < 0:
                self.brake = int(abs(self.vel) * self.max_brake)
                state_msg.set_brake = self.brake
                state_msg.set_speed = 0
            else:
                state_msg.set_speed = int(self.max_speed)
                state_msg.set_brake = 0
        else:
            if self.vel > 0:
                state_msg.set_speed = int(self.vel * self.max_speed)
                state_msg.set_brake = 0
            else:
                state_msg.set_speed = 0
                self.brake = int(abs(self.vel) * self.max_brake)
                state_msg.set_brake = self.brake

        state_msg.set_gear = int(self.current_gear)

        self.pub.publish(state_msg)


if __name__ == "__main__":
    rospy.init_node('ros_mobile')
    max_speed = float(rospy.get_param("~Max_Speed", "10"))
    max_degree = 2000
    max_brake = 150  # Adjusted to positive value for brake range
    mobile = Controller(max_speed, max_degree, max_brake)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        mobile.publish_state()
        rate.sleep()
