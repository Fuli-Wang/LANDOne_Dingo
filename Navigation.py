#!/usr/bin/env python3

import time
import rospy
import math
import Navigation_Basic
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import tf.transformations

navigation = Navigation_Basic.Navigation_Basic()


class Navigation:

    def __init__(self):
        ########## The sensor needs to be set when using the robot. ##########
        self.imu_control = rospy.Subscriber("/imu/data", Imu, self.imu_control, queue_size=10)
        self.laser_control = rospy.Subscriber("/scan", LaserScan, self.laser_control, queue_size=10)
        self.odometry_control = rospy.Subscriber("/dingo_velocity_controller/odom", Odometry, self.odometry_control,
                                                 queue_size=10)
        # rospy.init_node("Navigation", anonymous=True, disable_signals=True)
        # self.velocity_control = rospy.Publisher("/dingo_velocity_control/cmd_vel", Twist, queue_size=10)
        # self.transformation_control = tf.TransformListener()
        # self.transformation_control.waitForTransform("base_link", "/odom", rospy.Time(), rospy.Duration(4))
        # self.time = self.transformation_control.getLatestCommonTime("base_link", "map")

        self.laser_front = []
        self.laser_left = []
        self.laser_right = []
        self.laser_back = []

        self.odometry_x = 0.0
        self.odometry_y = 0.0
        self.odometry_orientation = 0.0

        self.imu_x = 0.0
        self.imu_y = 0.0
        self.imu_z = 0.0

        self.x_orientation = 0.0
        self.y_orientation = 0.0
        self.z_orientation = 0.0

        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0

        self.velocity = Twist()
        self.velocity_linear = 0.0
        self.velocity_angular = 0.0

        self.rate = rospy.Rate(4)

    def laser_control(self, laser_data):
        laser_data = laser_data.ranges

        self.laser_front = laser_data[0]
        self.laser_left = laser_data[90]
        self.laser_right = laser_data[180]
        self.laser_back = laser_data[359]

        # print("\nLaser: \n", "\nFront: ", self.laser_front,
        #     "\nLeft: ", self.laser_left, "\nRight: ", self.laser_right, "\nBack: ", self.laser_back)

        # self.rate.sleep()

        return self.laser_front, self.laser_left, self.laser_right, self.laser_back

    def odometry_control(self, odometry_data):
        self.odometry_x = odometry_data.pose.pose.position.x
        self.odometry_y = odometry_data.pose.pose.position.y
        self.odometry_orientation = odometry_data.pose.pose.orientation

        # print("\nOdometry: \n", "\nX: ", self.odometry_x, "\nY: ", self.odometry_y,
        #     "\nOrientation: ", self.odometry_orientation)

        # self.rate.sleep()

        return self.odometry_x, self.odometry_y

    def imu_control(self, imu_data):
        self.imu_x = imu_data.orientation.x
        self.imu_y = imu_data.orientation.y

        orientation = [imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w]
        self.x_orientation, self.y_orientation, self.z_orientation = euler_from_quaternion(orientation)

        # print(orientation)
        # print(self.x, self.y, self.z)
        # print("\nIMU: \n", "\nX: ", self.imu_x, "\nY: ", self.imu_y)
        # self.rate.sleep()

        return self.imu_x, self.imu_y, self.x_orientation, self.y_orientation

    def navigation_controller(self):
        navigation.movement_controller()

    def move_robot_arms(self):
        navigation.move_timer()

    def robot_movement_control(self):
        navigation.local_robot_control()
