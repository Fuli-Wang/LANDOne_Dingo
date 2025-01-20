#!/usr/bin/env python3
#export DINGO_OMNI=1
import Navigation
import time, math
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import termios, sys


class Navigation_Alignment:

    def __init__(self):

        self.velocity_publisher = rospy.Publisher("/dingo_velocity_controller/cmd_vel", Twist, queue_size=10)

        self.odometry_subscriber = rospy.Subscriber("/dingo_velocity_controller/odom", Odometry, self.x_y_odometry,
                                                    queue_size=10)

        self.imu_subscriber = rospy.Subscriber("/imu/data", Imu, self.x_y_imu, queue_size=10)

        self.laser_subscriber = rospy.Subscriber("/qr/coordinates", Point, self.get_coordinates, queue_size=10)

        self.velocity = Twist()

        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.yaw = 0.0

        self.x_imu = 0.0
        self.y_imu = 0.0

        self.target_position = [0.0, 0.0, 0.0]

        self.rate = rospy.Rate(10)

        self.velocity_linear_value = None
        self.velocity_angular_value = None

    def forward(self):
        self.velocity.linear.x = 0.1
        self.velocity.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity)

    def backward(self):
        self.velocity.linear.x = -0.1
        self.velocity.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity)

    def left(self):
        self.velocity.linear.x = 0.1
        self.velocity.angular.z = -0.3
        self.velocity_publisher.publish(self.velocity)

    def right(self):
        self.velocity.linear.x = 0.1
        self.velocity.angular.z = 0.3
        self.velocity_publisher.publish(self.velocity)

    def stop_robot(self):
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity)

    def x_y_odometry(self, x_y_odometry):
        self.x = x_y_odometry.pose.pose.position.x
        self.y = x_y_odometry.pose.pose.position.y

        self.orientation = x_y_odometry.pose.pose.orientation

        #self.rate.sleep()

    def x_y_imu(self, x_y_imu):

        self.x_imu = x_y_imu.linear_acceleration.x
        self.y_imu = x_y_imu.linear_acceleration.y

        #self.rate.sleep()

    def get_coordinates(self, data):

        self.target_position[0] = data.x
        self.target_position[1] = data.y
        self.target_position[2] = data.z

    def translate_left(self, t):

        self.attribute = termios.tcgetattr(sys.stdin)
        value = 0
        velocity_linear_value = 0.0
        velocity_angular_value = 0.0
        velocity_linear_value = round(velocity_linear_value + 0.1, 1)
        self.velocity.linear.y = velocity_linear_value
        self.velocity.angular.z = velocity_angular_value
        print("The robot is translating left...")
        start_time = time.time()
        end_time = start_time + t
        while time.time() < end_time:
            self.velocity
            self.velocity_publisher.publish(self.velocity)
            pass

    def translate_right(self, t):

        self.attribute = termios.tcgetattr(sys.stdin)
        value = 0
        velocity_linear_value = 0.0
        velocity_angular_value = 0.0
        velocity_linear_value = round(velocity_linear_value - 0.1, 1)
        self.velocity.linear.y = velocity_linear_value
        self.velocity.angular.z = velocity_angular_value
        print("The robot is translating right...")
        start_time = time.time()
        end_time = start_time + t
        while time.time() < end_time:
            self.velocity
            self.velocity_publisher.publish(self.velocity)
            pass


    def move_alignment(self):
        rate = rospy.Rate(4)

        y_position = self.target_position[0] #reading the value from the camera
        x_position = self.target_position[2]

        front_accuracy = 350 # mm
        side_accuracy = 10 #mm
        velocity = 100 # mm/s

        t=0

        while x_position > front_accuracy or abs(y_position) > side_accuracy:
            t += 1
            if x_position >front_accuracy:
                forward_dist =  x_position-front_accuracy
                forward_time = forward_dist/velocity
                start_time = time.time()
                end_time = start_time + forward_time
                print("moving forward\n")
                while time.time() < end_time:
                    self.forward()
                    pass
            self.stop_robot()
            translate_time = abs(y_position)/velocity
            if y_position < 0:
                self.translate_left(translate_time)
                rate.sleep()
                print("moving left\n")
            elif y_position > 0:
                self.translate_right(translate_time)
                rate.sleep()
                print("moving right\n")
            print("The distance between the robot and the target is 0.35m ")

            y_position = self.target_position[0]
            x_position = self.target_position[2]

            if t >5:
                break







# def main():


# nav = Navigation_Basic()
# nav.move_timer()
# nav.movement_controller()


# if __name__ == "__main__":
# main()
