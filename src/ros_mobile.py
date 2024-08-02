#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from erp42_serial2.msg import ERP_CMD


class Controller:
    def __init__(self, ms, md, mb, accel_time):
        self.max_speed = ms
        self.max_degree = md
        self.max_brake = mb
        self.accel_time = accel_time  # 최대 속도에 도달하는 시간(초)
        self.current_gear = 1
        self.vel = 0
        self.steer = 0
        self.brake = 0
        self.cruise = 0
        self.cruise_speed = 0
        self.ramp_mode = 0
        self.last_time = rospy.get_time()
        self.pub = rospy.Publisher('erp42_cmd', ERP_CMD, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)
        rospy.Subscriber("/cmd_steer", Twist, self.steer_callback)
        rospy.Subscriber("/btn_cruise", Bool, self.cruise_callback)
        rospy.Subscriber("/btn_press0", Bool, self.gear0_callback)
        rospy.Subscriber("/btn_press1", Bool, self.gear1_callback)
        rospy.Subscriber("/btn_press2", Bool, self.gear2_callback)

    def cruise_callback(self, msg:Bool):
        if msg.data:
            if self.cruise == 0:
                self.cruise = 1
                self.current_gear = 0
            else:
                self.cruise = 0
                self.current_gear = 1

    def gear0_callback(self, msg:Bool):
        if msg.data:
            if self.current_gear == 0:
                if self.cruise == 1:
                    self.current_gear = 0
                    self.cruise = 0
                else:
                    self.current_gear = 1
                    self.cruise = 0
            else:
                self.current_gear = 0
                self.cruise = 0

    def gear1_callback(self, msg:Bool):
        if msg.data:
            self.current_gear = 1
            self.cruise = 0

    def gear2_callback(self, msg:Bool):
        if msg.data:
            if self.current_gear == 2:
                self.current_gear = 1
            else:
                self.current_gear = 2

    def vel_callback(self, msg:Twist):
        self.vel = msg.linear.x

    def steer_callback(self, msg:Twist):
        self.steer = -msg.angular.z

    def publish_state(self):
        current_time = rospy.get_time()
        elapsed_time = current_time - self.last_time
        self.last_time = current_time

        serial_msg = ERP_CMD()
        serial_msg.cmd_steer = int(self.steer * self.max_degree)

        max_acceleration = self.max_speed / self.accel_time  # m/s^2

        if self.cruise == 1:
            if self.ramp_mode == 0:
                self.cruise_speed += max_acceleration * elapsed_time  # Increase speed
                if self.cruise_speed >= self.max_speed:
                    self.ramp_mode = 1
            elif self.ramp_mode == 1:
                self.cruise_speed = self.max_speed

            serial_msg.cmd_speed = min(max(int(self.cruise_speed), 0), self.max_speed)  # Set cruise_speed

            if self.vel < 0:
                self.brake = int(self.vel * self.max_brake)
                serial_msg.cmd_brake = self.brake
                serial_msg.cmd_speed = 0
        else:
            if self.vel > 0:
                target_speed = min(self.vel, self.max_speed)
                current_speed = self.cruise_speed + max_acceleration * elapsed_time
                self.cruise_speed = max(min(target_speed, current_speed),self.max_speed)
                serial_msg.cmd_speed = int(self.cruise_speed)
                self.brake = 0
                serial_msg.cmd_brake = 0
            elif self.vel <= 0:
                self.cruise_speed = 0
                serial_msg.cmd_speed = 0
                self.brake = int(self.vel * self.max_brake)
                serial_msg.cmd_brake = self.brake

        if self.current_gear == 1:
            serial_msg.cmd_speed = 0
            serial_msg.cmd_brake = self.brake
            serial_msg.cmd_gear = self.current_gear
        else:
            serial_msg.cmd_gear = self.current_gear

        rospy.loginfo(serial_msg)
        self.pub.publish(serial_msg)


if __name__ == "__main__":
    rospy.init_node('ros_mobile')
    max_speed = rospy.get_param("~Max_Speed", 30)
    max_degree = rospy.get_param("~Max_Degree", 2000)
    max_brake = rospy.get_param("~Max_Brake", -150)
    accel_time = rospy.get_param("~Accel_Time", 10)  # 최대 속도에 도달하는 시간

    mobile = Controller(max_speed, max_degree, max_brake, accel_time)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        mobile.publish_state()
        rate.sleep()
