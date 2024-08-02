#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import sin, cos
from sensor_msgs.msg import Imu
from erp42_serial2.msg import ERP_STATUS
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import tf

class ERP_ODOM:

    def __init__(self):
        rospy.init_node('erp_odom', anonymous=True)
        self.prev_time = rospy.get_rostime()
        self.is_status = False

        # Publishers
        self.odom_pub = rospy.Publisher('/erp42_odom', Odometry, queue_size=1)
        self.path_pub = rospy.Publisher('/erp42_path', Path, queue_size=1)

        # Subscribers
        rospy.Subscriber('/erp42_status', ERP_STATUS, self.erpcallback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        # Variables
        self.x = 0
        self.y = 0
        self.z = 0
        self.heading_rad = 0
        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.rate = rospy.Rate(20)  # 30Hz

        while not rospy.is_shutdown():
            self.publish_odometry()
            self.rate.sleep()

    def publish_odometry(self):
        if self.is_status:
            current_time = rospy.get_rostime()
            interval_time = (current_time - self.prev_time).to_sec()
            linear_x = self.status_msg.status_speed / (10.0 * 3.6) # kph to m/s
            angular_z = self.imu_msg.angular_velocity.z

            self.x += linear_x * cos(self.heading_rad) * interval_time
            self.y += linear_x * sin(self.heading_rad) * interval_time
            self.heading_rad += angular_z * interval_time
            q = tf.transformations.quaternion_from_euler(0, 0, self.heading_rad)

            # Broadcast the transform
            br = tf.TransformBroadcaster()
            br.sendTransform((self.x, self.y, self.z),
                             q,
                             current_time,
                             "imu_link",
                             "odom")

            # Publish the odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'imu_link'
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = self.z
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            odom_msg.twist.twist.linear.x = linear_x
            odom_msg.twist.twist.angular.z = angular_z
            self.odom_pub.publish(odom_msg)

            # Add pose to the path
            pose = PoseStamped()
            pose.header.stamp = current_time
            pose.header.frame_id = 'odom'
            pose.pose.position.x = self.x
            pose.pose.position.y = self.y
            pose.pose.position.z = self.z
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            self.path.poses.append(pose)
            self.path.header.stamp = current_time

            # Publish the path message
            self.path_pub.publish(self.path)

            self.prev_time = current_time

    def erpcallback(self, msg):
        self.status_msg = msg
        self.is_status = True

    def imu_callback(self, msg):
        self.imu_msg = msg

if __name__ == '__main__':
    try:
        ERP_ODOM()
    except rospy.ROSInterruptException:
        pass
