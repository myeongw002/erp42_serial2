#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, Twist, Pose, PoseStamped, Vector3
import math

class ImuOdometryNode:
    def __init__(self):
        rospy.init_node('imu_to_odometry')
        
        # Parameters
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'imu_link')
        self.rate = rospy.get_param('~rate', 10)
        self.imu_topic = rospy.get_param('~imu_topic', '/imu/data')
        self.path_topic = rospy.get_param('~path_topic', '/path3')

        # Publishers and Subscribers
        self.odom_pub = rospy.Publisher('erp42_odom3', Odometry, queue_size=10)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)

        # Variables for odometry calculation
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = rospy.Time.now()

        # Path message initialization
        self.path = Path()
        self.path.header.frame_id = self.odom_frame_id

        rospy.loginfo("IMU to Odometry node with Path generation started.")

    def imu_callback(self, data):
        # Current time
        current_time = rospy.Time.now()

        # Calculate time delta
        dt = (current_time - self.last_time).to_sec()
        #rospy.loginfo(dt)
        # Extract yaw rate (angular velocity around z-axis)
        yaw_rate = data.angular_velocity.z
        
        # Extract linear acceleration in the robot's frame
        ax = data.linear_acceleration.x
        ay = data.linear_acceleration.y
        
        # Assume constant velocity model (for simplicity)
        vx = ax * dt
        vy = ay * dt
        
        # Calculate change in orientation
        delta_th = yaw_rate * dt
        
        # Integrate to get position change in the world frame
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        
        # Update robot's position
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # Create quaternion from yaw angle
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # Publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            odom_quat,
            current_time,
            self.base_frame_id,
            self.odom_frame_id
        )

        # Create the odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame_id

        # Set the position
        odom.pose.pose = Pose(Vector3(self.x, self.y, 0.0), Quaternion(*odom_quat))

        # Set the velocity
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, yaw_rate))

        # Publish the odometry message
        self.odom_pub.publish(odom)

        # Add the current pose to the path
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = current_time
        pose_stamped.header.frame_id = self.odom_frame_id
        pose_stamped.pose = odom.pose.pose
        self.path.poses.append(pose_stamped)

        # Update the path header timestamp
        self.path.header.stamp = current_time

        # Publish the path message
        self.path_pub.publish(self.path)

        # Update time for the next iteration
        self.last_time = current_time

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ImuOdometryNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

