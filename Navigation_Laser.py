#!/usr/bin/env python3
import time
import math
import numpy as np
import rospy
import Navigation
import termios, sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sklearn.cluster import KMeans #pip install scikit-learn

target_position = np.array([0.0, 0.0, 0.0])
target_pose = 0.0

class Navigation_Robot:

    def __init__(self):

        self.velocity_publisher = rospy.Publisher("/dingo_velocity_controller/cmd_vel", Twist, queue_size=10)

        self.odometry_subscriber = rospy.Subscriber("/dingo_velocity_controller/odom", Odometry, self.x_y_odometry,
                                                    queue_size=10)

        self.laser_publisher = rospy.Subscriber("/scan", LaserScan, self.navigation_robot_laser_sensor, queue_size=10)

        self.velocity = Twist()

        self.laser_data = []
        self.laser_front_range = 0.0
        self.laser_back_range = 0.0
        self.laser_left_range = 0.0
        self.laser_right_range = 0.0
        self.laser_side_min =10.0
        self.laser_front_min =10.0
        self.laser_back_min = 10.0

        self.velocity_linear_value = None
        self.velocity_angular_value = None

        self.rate = rospy.Rate(10)

        #### Robot Parameters ####

        self.stop_robot = 0.7
        self.robot_stop = 0.0
        self.robot_slow = 0.1
        self.robot_medium = 0.3
        self.robot_fast = 0.5


    def get_centroids(self, arr):
        kmeans = KMeans(n_clusters=3)
        kmeans.fit(arr.reshape(-1,1))
        centroids = kmeans.cluster_centers_
        min_centroid = min(centroids)
        del kmeans
        return min_centroid[0]


    def navigation_robot_laser_sensor(self, laser_data):

        self.laser_data = laser_data.ranges #0-896
        #print(len(self.laser_data))

        #### Back ####
        laser_back = self.laser_data[0]
        laser_back_obstacles = self.laser_data[0:111]+self.laser_data[784:897]
        self.laser_back_range = np.array(laser_back_obstacles)
        self.laser_back_min = min(self.laser_back_range)


        #### Right ####
        laser_right = self.laser_data[112]
        laser_right_obstacles = self.laser_data[112:335]
        self.laser_right_range = np.array(self.laser_right_obstacles)
        laser_right_min = min(self.laser_right_range)

        #### Front ####
        laser_front = self.laser_data[336]
        laser_front_obstacles = self.laser_data[336:559]
        self.laser_front_range = np.array(self.laser_front_obstacles)
        self.laser_front_min = min(self.laser_front_range)

        #### Left ####
        laser_left = self.laser_data[560]
        laser_left_obstacles = self.laser_data[560:783]
        self.laser_left_range = np.array(self.laser_left_obstacles)
        laser_left_min = min(self.laser_left_range)

        self.laser_side_min = min(laser_left_min, laser_right_min)


    def laser_ranges(self):

        rate = rospy.Rate(4)

        while True:
            print(" Front Range: ", "{:.2f}".format(self.laser_front_average), "m", "\n", "Back Range: ",
                  "{:.2f}".format(self.laser_back_average), "m", "\n", "Left Range: ",
                  "{:.2f}".format(self.laser_left_average), "m", "\n", "Right Range: ",
                  "{:.2f}".format(self.laser_right_average), "m")
            print(len(self.laser_data))
            rate.sleep()

    def obstacle_detection(self):
        obstalce = False
        if self.laser_min_side < 0.5 or self.laser_min_front < 0.35:
            print("Obstacle!!! I have to stop")
            obstalce = True
        return obstalce

    def x_y_odometry(self, x_y_odometry):
        self.x = x_y_odometry.pose.pose.position.x
        self.y = x_y_odometry.pose.pose.position.y

        self.orientation = x_y_odometry.pose.pose.orientation

        #self.rate.sleep()
    def stop(self):
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity)

    def forward(self, t):
        self.velocity.linear.x = 0.2
        self.velocity.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity)
        start_time = time.time()
        end_time = start_time + t
        print("moving forward\n")
        while time.time() < end_time:
            self.velocity_publisher.publish(self.velocity)
            obstalce = self.obstacle_detection()
            if obstalce:
                self.stop()
                break
            pass

    def backward(self):
        self.velocity.linear.x = -0.2
        self.velocity.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity)

    def translate_right(self, t):

        self.attribute = termios.tcgetattr(sys.stdin)
        value = 0
        velocity_linear_value = 0.0
        velocity_angular_value = 0.0
        velocity_linear_value = round(velocity_linear_value - 0.2, 1)
        self.velocity.linear.y = velocity_linear_value
        self.velocity.angular.z = velocity_angular_value
        print("The robot is translating right...")
        start_time = time.time()
        end_time = start_time + t
        while time.time() < end_time:
            self.velocity
            self.velocity_publisher.publish(self.velocity)
            obstalce = self.obstacle_detection()
            if obstalce:
                self.stop()
                break
            pass

    def translate_left(self, t):

        self.attribute = termios.tcgetattr(sys.stdin)
        value = 0
        velocity_linear_value = 0.0
        velocity_angular_value = 0.0
        velocity_linear_value = round(velocity_linear_value + 0.2, 1)
        self.velocity.linear.y = velocity_linear_value
        self.velocity.angular.z = velocity_angular_value
        print("The robot is translating left...")
        start_time = time.time()
        end_time = start_time + t
        while time.time() < end_time:
            self.velocity
            self.velocity_publisher.publish(self.velocity)
            obstalce = self.obstacle_detection()
            if obstalce:
                self.stop()
                break
            pass

    def get_xyz(self):

        while True:
            try:
                f = open('Calibration/targetposition.txt')
                matrix = f.read().split()
                target_position[0] = float(matrix[0])
                target_position[1] = float(matrix[1])
                target_position[2] = float(matrix[2])
                target_pose = float(matrix[3])
                break
            except:
                print("Oops!  Cannot find the file.  Try again...")
        return target_position, target_pose

    def pose_guide(self):

        self.stop()
        rate = rospy.Rate(4)
        velocity = 200 # mm/s this value depends on the V from forward/backward/translate functions
        safe_front = 400 # mm safety distance
        safe_side = 600 # mm safety distance
        y_gap = float('inf')
        x_gap = float('inf')

        while (y_gap> safe_side and x_gap>safe_front):
            target_position, target_pose = self.get_xyz()

            y_position = round(target_position[0]) #reading the value from the camera
            x_position = round(target_position[2])

            laser_front_dis = self.get_centroids(self.laser_front_range) * 1000 #reading the value from the Lidar
            laser_back_dis = self.get_centroids(self.laser_back_range)  * 1000
            laser_left_dis = self.get_centroids(self.laser_left_range) * 1000
            laser_right_dis = self.get_centroids(self.laser_right_range) * 1000

            #calculate the force
            y_force = y_position
            x_force = x_position

            if target_pose == 7.7 and y_force<=0:
                y_force = -(abs(x_force)+1)
            elif target_pose == 6.6 and y_force>=0:
                y_force = abs(x_force)+1
            else:
                y_force = abs(x_force)-1

            if x_force>=abs(y_force):
                # forword movement first
                if x_position <= safe_front:
                    print("The forward distance between the robot and the target is too short\n")
                    self.stop()
                else:
                    forward_dist =  x_position - safe_front
                    if laser_front_dis <= safe_front and laser_front_dis !=0.0:
                        print(laser_front_dis)
                        print("The front obstacle is too close\n")
                        rate.sleep()
                    elif safe_front < laser_front_dis < forward_dist:
                        print("front obstalce detected by LIDAR")
                        rate.sleep()
                        forward_time = math.floor(laser_front_dis)/(velocity+50) # plus 50 for safety
                        start_time = time.time()
                        end_time = start_time + forward_time
                        print("moving forward\n")
                        while time.time() < end_time:
                            self.forward()
                            pass
                    else:
                        forward_time = forward_dist/velocity
                        start_time = time.time()
                        end_time = start_time + forward_time
                        print("moving forward\n")
                        while time.time() < end_time:
                            self.forward()
                            pass
                    self.stop()

            else:
                translate_time = abs(y_position)/velocity
                if y_force < 0:
                    if laser_left_dis <= safe_side:
                        print("The left side obstacle is too close\n")
                    elif safe_side <laser_left_dis < abs(y_position):
                        print("lef side obstalce detected by LIDAR\n")
                        translate_time = math.floor(laser_left_dis)/(velocity+50)
                        self.translate_left(translate_time)
                    else:
                        self.translate_left(translate_time)
                        print("moving left\n")
                        print("The robot has reached target")
                    rate.sleep()
                    self.stop()
                if y_force > 0:
                    if laser_right_dis <= safe_side:
                        print("The right obstacle is too close\n")
                    elif safe_side < laser_right_dis < abs(y_position):
                        print("right side obstalce detected by LIDAR\n")
                        translate_time = math.floor(laser_right_dis)/(velocity+50)
                        self.translate_right(translate_time)
                    else:
                        self.translate_right(translate_time)
                        print("moving right\n")
                        print("The robot has reached target")
                    rate.sleep()
                    self.stop()

                target_position, target_pose = self.get_xyz()
                y_newposition = round(target_position[0]) #reading the value from the camera
                x_newposition = round(target_position[2])

                y_gap = abs(y_newposition)
                x_gap = abs(x_newposition)

    def move_timer(self):
        self.stop()
        target_position, target_pose = self.get_xyz()
        rate = rospy.Rate(4)
        velocity = 200 # mm/s this value depends on the V from forward/backward/translate functions
        safe_front = 400 # mm safety distance
        safe_side = 600 # mm safety distance

        y_position = round(target_position[0]) #reading the value from the camera
        x_position = round(target_position[2])

        laser_front_centroids = self.get_centroids(self.laser_front_range)
        laser_left_centroids = self.get_centroids(self.laser_left_range)
        laser_right_centroids = self.get_centroids(self.laser_right_range)

        if y_position < 0:
            if laser_left_average < laser_front_average: #forward
                laser_front_dis = laser_front_average * 1000
                if x_position < laser_front_dis:
                    forward_time = x_position/velocity
                    start_time = time.time()
                    end_time = start_time + forward_time
                    print("moving forward\n")
                    while time.time() < end_time:
                        self.forward()
                        pass
                    #move Left
                    self.stop()
                    rate.sleep()
                    translate_time = abs(y_position)/velocity
                    self.translate_left(translate_time)
                    self.stop()
                else:
                    forward_time = (laser_front_dis-safe_front)/velocity
                    start_time = time.time()
                    end_time = start_time + forward_time
                    print("moving forward\n")
                    while time.time() < end_time:
                        self.forward()
                        pass
                    self.stop()
                    rate.sleep()
                    translate_time = abs(y_position)/velocity
                    self.translate_left(translate_time)
                    self.stop()

                    forward_time2 = (x_position-laser_front_dis+safe_front)/velocity
                    start_time = time.time()
                    end_time = start_time + forward_time2
                    print("moving forward\n")
                    while time.time() < end_time:
                        self.forward()
                        pass
                    self.stop()
            else:
                laser_left_dis = laser_left_average * 1000
                if abs(y_position) < laser_left_dis:
                    translate_time = abs(y_position)/velocity
                    self.translate_left(translate_time)
                    self.stop()
                    forward_time = x_position/velocity
                    start_time = time.time()
                    end_time = start_time + forward_time
                    print("moving forward\n")
                    while time.time() < end_time:
                        self.forward()
                        pass
                    #move Left
                    self.stop()
                else:
                    translate_time = (laser_left_dis-safe_side)/velocity
                    self.translate_left(translate_time)
                    self.stop()
                    forward_time = x_position/velocity
                    start_time = time.time()
                    end_time = start_time + forward_time
                    print("moving forward\n")
                    while time.time() < end_time:
                        self.forward()
                        pass
                    #move Left
                    self.stop()
                    translate_time2 = (abs(y_position)-(laser_left_dis-safe_side))/velocity
                    self.translate_left(translate_time)
                    self.stop()
        else:
            if laser_right_average < laser_front_average: #forward
                laser_front_dis = laser_front_average * 1000
                if x_position < laser_front_dis:
                    forward_time = x_position/velocity
                    start_time = time.time()
                    end_time = start_time + forward_time
                    print("moving forward\n")
                    while time.time() < end_time:
                        self.forward()
                        pass
                    #move Left
                    self.stop()
                    rate.sleep()
                    translate_time = y_position/velocity
                    self.translate_right(translate_time)
                    self.stop()
                else:
                    forward_time = (laser_front_dis-safe_front)/velocity
                    start_time = time.time()
                    end_time = start_time + forward_time
                    print("moving forward\n")
                    while time.time() < end_time:
                        self.forward()
                        pass
                    self.stop()
                    rate.sleep()
                    translate_time = y_position/velocity
                    self.translate_right(translate_time)
                    self.stop()

                    forward_time2 = (x_position-laser_front_dis+safe_front)/velocity
                    start_time = time.time()
                    end_time = start_time + forward_time2
                    print("moving forward\n")
                    while time.time() < end_time:
                        self.forward()
                        pass
                    self.stop()
            else:
                laser_right_dis = laser_right_average * 1000
                if y_position < laser_right_dis:
                    translate_time = y_position/velocity
                    self.translate_right(translate_time)
                    self.stop()
                    forward_time = x_position/velocity
                    start_time = time.time()
                    end_time = start_time + forward_time
                    print("moving forward\n")
                    while time.time() < end_time:
                        self.forward()
                        pass
                    #move Left
                    self.stop()
                else:
                    translate_time = (laser_right_dis-safe_side)/velocity
                    self.translate_right(translate_time)
                    self.stop()
                    forward_time = x_position/velocity
                    start_time = time.time()
                    end_time = start_time + forward_time
                    print("moving forward\n")
                    while time.time() < end_time:
                        self.forward()
                        pass
                    #move Left
                    self.stop()
                    translate_time2 = (y_position-(laser_right_dis-safe_side))/velocity
                    self.translate_right(translate_time)
                    self.stop()

#def main():
#    nav_robot = Navigation_Robot()

#    nav = Navigation.Navigation()

#    rate = rospy.Rate(4)

#    while not rospy.is_shutdown():
       # rate.sleep()
        #nav_robot.navigation_robot_laser_sensor()
#        nav_robot.laser_ranges()
        #nav_robot.navigation_home()

        # nav_robot.navigation_position()
#        rate.sleep()
#    rospy.spin()


#if __name__ == "__main__":
#    main()
