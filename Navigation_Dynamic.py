#!/usr/bin/env python3
#export DINGO_OMNI=1
from collections import deque
import Navigation
import time, math
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import termios, sys

target_destination = np.array([0.0, 0.0, 0.0])

class Navigation_Dynamic:

    def __init__(self):

        self.velocity_publisher = rospy.Publisher("/dingo_velocity_controller/cmd_vel", Twist, queue_size=10)

        self.odometry_subscriber = rospy.Subscriber("/dingo_velocity_controller/odom", Odometry, self.x_y_odometry,
                                                    queue_size=10)

        self.imu_subscriber = rospy.Subscriber("/imu/data", Imu, self.x_y_imu, queue_size=10)

        self.xyz_subscriber = rospy.Subscriber("/target/coordinates", Point, self.get_coordinates, queue_size=10)

        self.movement_subscriber = rospy.Subscriber("/target/motion", Point, self.get_movement, queue_size=10)

        self.velocity = Twist()

        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.yaw = 0.0

        self.x_imu = 0.0
        self.y_imu = 0.0

        self.target_position = [0.0, 0.0, 0.0]

        self.target_movement = [-1.0, -1.0, -1.0]

        self.rate = rospy.Rate(10)

        self.velocity_linear_value = None
        self.velocity_angular_value = None
        
        self.start_x = 0.0
        
        self.movement_history = deque(maxlen=5)  # Keep a history of the last 5 movement statuses

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

    def get_movement(self, data):

        self.target_movement[0] = data.x  # Distance from the robot
        self.target_movement[1] = data.y  # Status (0: stationary, 1: about to move, 2: moving)
        self.target_movement[2] = data.z  # Direction (0: towards, 1: away, 2: lateral)
        
        # Append to history for smoothing
        self.movement_history.append(self.target_movement[1])
        
        # Use the most common value in the history to smooth noise
        if len(self.movement_history) > 1:
            self.target_movement[1] = max(set(self.movement_history), key=self.movement_history.count)

    def get_destination(self):
        while True:
            try:
                f = open('/home/fuli/Documents/Dingo/Calibration/targetposition.txt')
                matrix = f.read().split()
                print(matrix)
                target_destination[0] = float(matrix[0])
                target_destination[1] = float(matrix[1])
                target_destination[2] = float(matrix[2])
                break
            except:
                time.sleep(0.5)
                print("Oops!  Cannot find the file.  Try again...")
        return target_destination

    def translate_left(self, t):
        self.velocity.linear.x = 0.0  # Ensure forward/backward movement is stopped
        self.velocity.linear.y = 0.1
        self.velocity.angular.z = 0.0
        print("The robot is translating left...")
        start_time = time.time()
        end_time = start_time + t
        while time.time() < end_time:
            self.velocity_publisher.publish(self.velocity)

    def translate_right(self, t):
        self.velocity.linear.x = 0.0  # Ensure forward/backward movement is stopped
        self.velocity.linear.y = -0.1
        self.velocity.angular.z = 0.0
        print("The robot is translating right...")
        start_time = time.time()
        end_time = start_time + t
        while time.time() < end_time:
            self.velocity_publisher.publish(self.velocity)


    def move_dynamic(self):
    
        self.start_x = self.x  # Store the current odometry reading as the start point
        
        rate = rospy.Rate(4)
            

        target_destination = self.get_destination()

        y_position = target_destination[0] # Lateral position
        x_position = target_destination[2] # Forward position

        front_accuracy = 350 # mm
        side_accuracy = 10 #mm
        velocity = 100 # mm/s
        
        while not rospy.is_shutdown():
        
            # Calculate the relative x-position from the start point
            relative_x = abs(self.x - self.start_x) * 1000

            # Use movement and coordinates information to adjust movement
            obstacle_distance = self.target_movement[0]
            obstacle_status = self.target_movement[1]
            obstacle_direction = self.target_movement[2]
            
            #rospy.loginfo(f"Current Odometry: relative_x = {relative_x}, Target Distance: {obstacle_distance}, Status: {obstacle_status}, Direction: {obstacle_direction}")
            
            # Movement control logic
            if relative_x < x_position - front_accuracy:
                # Check if there's a moving obstacle in front of the robot
                if obstacle_distance < 1350 and obstacle_status in [1, 2] and obstacle_direction == 0:
                    # Stop if the obstacle is moving towards or is about to move and too close
                    print("Obstacle detected! Stopping.")
                    self.stop_robot()
                    rospy.sleep(1)
                    # Re-check obstacle status
                    if obstacle_distance < 1300 and obstacle_status in [1, 2] and obstacle_direction == 0:
                        print("Obstacle still detected! Continuing to wait.")
                        continue
                    else:
                        # Move forward if there is no significant obstacle in front
                        print("Obstacle cleared. Resuming movement")
                        self.forward()
                else:
                    print("Moving forward towards the target.")
                    self.forward()
            else:
                # If within target distance, stop
                self.stop_robot()
                print("Reached target distance. Stopping.")
                break
            rate.sleep()  # Sleep to maintain loop rate

        # Stop when reaching the front accuracy limit
        self.stop_robot()


        translate_time = abs(y_position)/velocity
        if y_position < 0:
            print("moving left\n")
            self.translate_left(translate_time)
        elif y_position > 0:
            print("moving right\n")
            self.translate_right(translate_time)







# def main():


# nav = Navigation_Basic()
# nav.move_timer()
# nav.movement_controller()


# if __name__ == "__main__":
# main()
