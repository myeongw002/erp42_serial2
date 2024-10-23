#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import sin, cos, pi
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
<<<<<<< HEAD
        self.imu_msg = Imu()
=======

>>>>>>> Update
        # Variables
        self.x = 0
        self.y = 0
        self.z = 0
        self.heading_rad = 0
        self.path = Path()
        self.path.header.frame_id = 'odom'
<<<<<<< HEAD

        self.rate = rospy.Rate(20)  # 30Hz

        # Encoder related variables
        self.prev_encoder = 0
        self.wheel_radius = 0.27  # Assuming a wheel radius of 0.3 meters
        self.encoder_resolution = 100  # Assuming 4096 ticks per revolution
=======
        self.prev_velocity = 0.0  # Previous velocity initialization

        self.rate = rospy.Rate(20)  # 20Hz

        # Encoder related variables
        self.prev_encoder = 0
        self.wheel_radius = 0.27  
        self.encoder_resolution = 200  
>>>>>>> Update

        while not rospy.is_shutdown():
            self.publish_odometry()
            self.rate.sleep()

    def calculate_velocity_from_encoder(self, current_encoder):
        # Skip calculation if this is the first reading
        if self.prev_encoder == 0:
            self.prev_encoder = current_encoder
            self.prev_time = rospy.get_rostime()
            return 0.0

<<<<<<< HEAD
=======
        # Check if the encoder value has not changed
        if current_encoder == self.prev_encoder:
            return self.prev_velocity  # Maintain previous velocity if encoder hasn't changed

>>>>>>> Update
        encoder_diff = current_encoder - self.prev_encoder
        if encoder_diff < 0:
            encoder_diff += self.encoder_resolution  # Handle encoder wrap-around

        # Calculate the distance traveled by the wheel
        distance_traveled = (encoder_diff / self.encoder_resolution) * 2 * 3.14159 * self.wheel_radius
        current_time = rospy.get_rostime()
        interval_time = (current_time - self.prev_time).to_sec()

        if interval_time > 0:
            velocity = distance_traveled / interval_time
<<<<<<< HEAD
        else:
            velocity = 0.0
=======
            self.prev_velocity = velocity  # Update previous velocity
        else:
            velocity = self.prev_velocity  # Maintain previous velocity if interval_time <= 0
>>>>>>> Update

        # Update previous values
        self.prev_encoder = current_encoder
        self.prev_time = current_time
<<<<<<< HEAD
        rospy.loginfo(f"Velocity: {velocity}")
        return velocity


=======

        return velocity




>>>>>>> Update
    def publish_odometry(self):
        if self.is_status:
            current_time = rospy.get_rostime()
            interval_time = (current_time - self.prev_time).to_sec()

            # Calculate linear velocity from encoder
            linear_x = self.calculate_velocity_from_encoder(self.status_msg.status_enc)
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
<<<<<<< HEAD
            rospy.loginfo(odom_msg.pose.pose)
=======
            
            rospy.loginfo(f"{linear_x * 3.6} km/h")
            rospy.loginfo(f"{linear_x} m/s")
            rospy.loginfo(odom_msg.pose.pose)
            
>>>>>>> Update
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

