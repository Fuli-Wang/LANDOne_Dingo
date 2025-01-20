#!/usr/bin/env python3
#export DINGO_OMNI=1
import time, math
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
import tty, termios, sys, select


cue = np.array([0.0, 0.0, 0.0, 0.0])
target_position = np.array([0.0, 0.0, 0.0])
target_information = np.array([0])
target_pose = 0.0

class Navigation_Basic:

    def __init__(self):

        rospy.init_node("Navigation_Basic", anonymous=True, disable_signals=True)

        self.velocity_publisher = rospy.Publisher("/dingo_velocity_controller/cmd_vel", Twist, queue_size=10)

        self.odometry_subscriber = rospy.Subscriber("/dingo_velocity_controller/odom", Odometry, self.x_y_odometry,
                                                    queue_size=10)

        self.imu_subscriber = rospy.Subscriber("/imu/data", Imu, self.x_y_imu, queue_size=10)

        self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.laser_movement, queue_size=10)
        
        self.point_subscriber = rospy.Subscriber("/target/coordinates", Point, self.get_xyz, queue_size=10)

        self.velocity = Twist()

        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.yaw = 0.0

        self.x_imu = 0.0
        self.y_imu = 0.0

        self.laser = []
        self.laser_front = []
        self.laser_back = []
        self.laser_left = []
        self.laser_right = []
        
        self.target_position = [0.0, 0.0, 0.0]

        self.rate = rospy.Rate(10)

        self.velocity_linear_value = None
        self.velocity_angular_value = None

    def forward(self):
        self.velocity.linear.x = 0.2
        self.velocity.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity)

    def backward(self):
        self.velocity.linear.x = -0.2
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

    def laser_movement(self, laser_msg):

        laser = laser_msg.ranges
        self.laser = laser_msg
        self.laser_front = laser[0]
        self.laser_back = laser[180]
        self.laser_left = laser[270]
        self.laser_right = laser[90]

    #def get_xyz(self):

        #while True:
            #try:
                #f = open('Calibration/targetposition.txt')
                #matrix = f.read().split()
                #target_position[0] = float(matrix[0])
                #target_position[1] = float(matrix[1])
                #target_position[2] = float(matrix[2])
                #target_pose = float(matrix[3])
                #break
            #except:
                #time.sleep(0.5)
                #print("Oops!  Cannot find the file.  Try again...")
        #return target_position, target_pose

    def get_path(self):
        try:
            path_matrix = np.loadtxt('Calibration/path.txt')
            print("Path loaded!")
        except:
            print("Oops!  Cannot find the path file.")
        return path_matrix
        
    def get_xyz(self, data):

        self.target_position[0] = data.x
        self.target_position[1] = data.y
        self.target_position[2] = data.z     

    def turn_left(self, t):
        self.attribute = termios.tcgetattr(sys.stdin)
        value = 0
        velocity_linear_value = 0.0
        velocity_angular_value = 0.0
        velocity_angular_value = round(velocity_angular_value + 0.2, 1)
        self.velocity.linear.x = velocity_linear_value
        self.velocity.angular.z = velocity_angular_value
        print("The robot is turnig left...")

        while True:
            value += 1
            self.velocity
            self.velocity_publisher.publish(self.velocity)
            if value == 182800*t: #365735:
                break

    def turn_right(self, t):
        self.attribute = termios.tcgetattr(sys.stdin)
        value = 0
        velocity_linear_value = 0.0
        velocity_angular_value = 0.0
        velocity_angular_value = round(velocity_angular_value - 0.2, 1)
        self.velocity.linear.x = velocity_linear_value
        self.velocity.angular.z = velocity_angular_value
        print("The robot is turnig right...")

        while True:
            value += 1
            self.velocity
            self.velocity_publisher.publish(self.velocity)
            if value == 182800*t: #398500:
                break

    def turn_around(self, t):
        self.attribute = termios.tcgetattr(sys.stdin)
        value = 0
        velocity_linear_value = 0.0
        velocity_angular_value = 0.0
        velocity_angular_value = round(velocity_angular_value + 0.2, 1)
        self.velocity.linear.x = velocity_linear_value
        self.velocity.angular.z = velocity_angular_value
        while True:
            value += 1
            self.velocity
            self.velocity_publisher.publish(self.velocity)
            print("The robot is turnig around...", value)
            if value == 364000*t:#  654681
                break

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
            pass

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
            pass


    def move_timer(self):
    
        self.stop_robot()

        #target_position, target_pose = self.get_xyz()
        rate = rospy.Rate(4)

        y_position = round(self.target_position[0]) #reading the value from the camera
        x_position = round(self.target_position[2])
        if x_position <=300:
            print("The forward distance between the robot and the target is too short\n")
            self.stop_robot()
        elif x_position >300:
            forward_dist =  x_position-300
            forward_time = forward_dist/200
            start_time = time.time()
            end_time = start_time + forward_time
            print("moving forward\n")
            while time.time() < end_time:
                self.forward()
                pass

        self.stop_robot()
        translate_time = abs(y_position)/200
        if y_position < 0:
            self.translate_left(translate_time)
            rate.sleep()
            print("moving left\n")
        elif y_position > 0:
            self.translate_right(translate_time)
            rate.sleep()
            print("moving right\n")
        print("The distance between the robot and the target is 0.3m ")

    def move_path(self):

        self.stop_robot()
        velocity = 200 # mm/s

        path_matrix = self.get_path()
        rate = rospy.Rate(4)

        for row in path_matrix:
            x_position = row[0]
            y_position = row[1]
            forward_dist =  x_position
            if forward_dist > 100:
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
                print("moving left\n")
            elif y_position > 0:
                self.translate_right(translate_time)
                print("moving right\n")
            self.stop_robot()
            rate.sleep()

    def shutdown_base(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity)
        exit("Close")

    def get_char(self):

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

        if rlist:

            get_char = sys.stdin.read(1)
        else:
            get_char = ""

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.attribute)
        return get_char

    def movement_controller(self):

        self.attribute = termios.tcgetattr(sys.stdin)

        value = 0
        velocity_linear_value = 0.0
        velocity_angular_value = 0.0

        while True:
            get_char = self.get_char()
            try:
                if get_char == "w":
                    velocity_linear_value = round(velocity_linear_value + 0.1, 1)
                    value += 1
                elif get_char == "a":
                    velocity_angular_value = round(velocity_angular_value + 0.1, 1)
                    value += 1
                elif get_char == "s":
                    velocity_linear_value = round(velocity_linear_value - 0.1, 1)
                    value += 1
                elif get_char == "d":
                    velocity_angular_value = round(velocity_angular_value - 0.1, 1)
                    value += 1
                elif get_char == "x":
                    value = 0
                    velocity_linear_value = 0
                    velocity_angular_value = 0
                    self.movement_controller()
                elif get_char == "q":
                    exit()
                else:
                    if value == 20:
                        value = 0
                    if velocity_linear_value == 1.0 or velocity_linear_value == -1.0:
                        velocity_linear_value = 0.0
                    if velocity_angular_value == 1.0 or velocity_angular_value == -1.0:
                        velocity_angular_value = 0.0

                self.velocity
                self.velocity.linear.x = velocity_linear_value
                #self.velocity.linear.y = velocity_angular_value #this is for translate left/right
                self.velocity.angular.z = velocity_angular_value #this is for rotation left/right
                self.velocity_publisher.publish(self.velocity)

            except KeyboardInterrupt:
                quit("Quit")
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.attribute)

    def human_following(self):

        #target_position, target_pose = self.get_xyz()
        target_position = self.target_position
        rate = rospy.Rate(3)

        while 200<target_position[2]:
            if target_position[0]==-999:
                print("I have to stop")
                break
            elif target_position[0]>750:
                self.turn_right(0.25)
                self.stop_robot()
                rate.sleep()
            elif target_position[0]<-750:
                self.turn_left(0.25)
                self.stop_robot()
                rate.sleep()
            elif -750<target_position[0]<750:
                origin = [0, 0, 0]
                z_position = round(target_position[2]) #reading the value from the camera
                z_required = 0 # the position where thr target shoule be
                #print("The distance between the robot and the leader: ", z_position)
                print(target_position[0],target_position[1],target_position[2])
                while z_required < 10:
                    self.velocity.linear.x = 0.1
                    z_required = z_required +1
                    self.velocity_publisher.publish(self.velocity)
                    #time.sleep(1.0)
                    print("MOVING")
                    rate.sleep()
                    #print("Moving: current position", z_position, "target position", z_required)

            time.sleep(0.2)

            #target_position, target_pose = self.get_xyz()
            target_position = self.target_position

    def pose_timer(self):

        #target_position, target_pose = self.get_xyz()
        self.stop_robot()
        target_pose = self.target_position[1]
        rate = rospy.Rate(4)

        y_position = round(self.target_position[0]) #reading the value from the camera
        x_position = round(self.target_position[2])

        x_safe_dis = 600
        y_safe_dis = 100

        velocity = 200 # mm/s this value depends on the V from forward/backward/translate functions

        if target_pose == 7.7:
            translate_time = abs(y_position)/200
            if y_position < 0:
                self.translate_left(translate_time)
                print("moving left\n")
            elif y_position > 0:
                self.translate_right(translate_time)
                print("moving right\n")
            self.stop_robot()
            rate.sleep()
            if x_position <=300:
                print("The forward distance between the robot and the target is too short\n")

            x_gap = float('inf')
            while x_gap>x_safe_dis:
                target_position = self.target_position
                x_position = round(target_position[2])
                forward_dist =  x_position - x_safe_dis
                forward_time = forward_dist/velocity
                start_time = time.time()
                end_time = start_time + forward_time
                print("moving forward\n")
                while time.time() < end_time:
                    self.forward()
                    pass
                self.stop_robot()
                rate.sleep()
                target_position = self.target_position
                x_newposition = round(target_position[2])
                x_gap = abs(x_newposition)

        else:
            forward_dist =  x_position - x_safe_dis
            forward_time = forward_dist/velocity
            start_time = time.time()
            end_time = start_time + forward_time
            print("moving forward\n")
            while time.time() < end_time:
                self.forward()
                pass
            self.stop_robot()
            rate.sleep()

            y_gap = float('inf')
            while y_gap>y_safe_dis:
                target_position = self.target_position
                y_position = round(target_position[0])
                translate_time = abs(y_position)/velocity
                if y_position < 0:
                    self.translate_left(translate_time)
                    print("moving left\n")
                elif y_position > 0:
                    self.translate_right(translate_time)
                    print("moving right\n")
                self.stop_robot()
                rate.sleep()
                target_position = self.target_position
                y_newposition = round(target_position[0])
                y_gap = abs(y_newposition)




# def main():


# nav = Navigation_Basic()
# nav.move_timer()
# nav.movement_controller()


# if __name__ == "__main__":
# main()
