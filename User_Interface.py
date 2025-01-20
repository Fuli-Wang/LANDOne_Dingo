#!/usr/bin/env python3
#!/bin/bash
import os
import subprocess

#print("\n Which robot are you using now?\n")
#robot_input = input("Select 1 for Robot A, 0 for Robot B: \n")

import time
import Navigation, Navigation_Basic, Navigation_Laser, Navigation_Alignment, Navigation_Dynamic
import rospy
from subprocess import Popen
import numpy as np
#from multiprocessing import Process
########################################################################################################################
####              User_Interface                                                                                    ####
########################################################################################################################

Navigation_B = Navigation_Basic.Navigation_Basic()
Navigation = Navigation.Navigation()
Navigation_L = Navigation_Laser.Navigation_Robot()
Navigation_A = Navigation_Alignment.Navigation_Alignment()
Navigation_D = Navigation_Dynamic.Navigation_Dynamic()

class User_Interface:


    def navigation(self):

        print("Navigation\n ")
        print("Use W, A, S, D to move, X to stop, Q to Quit\n ")
        Navigation_B.movement_controller()

    def navigation_reach(self):

        print("\n Autonomous Navigation based on the perception\n")
        Navigation_B.move_timer()

    def navigation_auto(self):

        print("\n Autonomous Navigation based on the perception\n")
        time.sleep(3.0)
        Navigation_B.pose_timer()

    def launch_perception_system(self):

        file_to_execute = "/home/fuli/Documents/yolov8/"
        os.chdir(file_to_execute)
        subprocess.call(["gnome-terminal", "--", "python3", "zed_yolov8.py"])

    def human_following(self):

        print("\n Human Following\n")
        time.sleep(3.0)
        Navigation_B.human_following()

    def navigation_laser(self):

        print("\n Autonomous Navigation based on the perception\n")
        time.sleep(3.0)
        Navigation_L.move_timer()

    def navigation_alignment(self):

        print("\n Switch the required perception model\n")
        file_to_execute = "/home/fuli/Documents/yolov8/"
        os.chdir(file_to_execute)
        subprocess.call(["gnome-terminal", "--", "python3", "ali_yolov8.py"])
        time.sleep(15.0)
        Navigation_A.move_alignment()

    def navigation_dynamic(self):

        print("\n Switch the required perception model \n")
        file_to_execute = "/home/fuli/Documents/yolov8/"
        os.chdir(file_to_execute)
        subprocess.call(["gnome-terminal", "--", "python3", "zed_yolov11_movement_ros.py"])
        time.sleep(15.0)
        Navigation_D.move_dynamic()

    def navigation_map(self):

        print("\n Autonomous Navigation based on the map information\n")
        Navigation_B.move_path()

    #def demo(self):

        #robot_a_move = Process(name="robot_a_move", target=Navigation_B.turn_left)
        #robot_b_move = Process(name="robot_b_move", target=Navigation_B.turn_right)
        #time.sleep(1.0)
        #robot_a_move.start()
        #time.sleep(1.0)
        #robot_b_move.start()
        #exit()

    def exit(self):

        exit("Close")

    def user_interface(self):

        UI = User_Interface()


        print("\n Dingo User Interface \n")

        option = input("################################################################################\n"
                       "    \nDingo Control \n\n"
                       "################################################################################\n"
                       "\n  - 1: Launch the Perception System:\n"
                       "\n  - 2: Basic Navigation:\n"
                       "\n  - 3: Target-Following Guider for mobile robot:\n"
                       "\n  - 4: Directly approaching to the target(camera only):\n"
                       "\n  - 5: Approaching to the target with camera feedback loop:\n"
                       "\n  - 6: Approaching to the target with collision avoidance(camera + LIDAR):\n "
                       "\n  - 7: Move based on the map:\n "
                       "\n  - 8: Alignment based on the camera:\n "
                       "\n  - 9: Recognizing the state of moving objects (under development)\n"
                       "\n  - 10: Exit\n \n"
                       "################################################################################\n"
                       "\n"
                       "################################################################################\n"
                       "\n \n  - Option: ")

        while True:
            try:
                if option == "1":
                    print("\n Launch the Perception System \n")
                    time.sleep(1.0)
                    UI.launch_perception_system()
                    time.sleep(8.0)
                    print("\n Launch User Interface \n")
                    UI.user_interface()
                    break
                if option == "2":
                    print("\n Navigation using the Keyboard \n")
                    time.sleep(1.0)
                    UI.navigation()
                    break
                if option == "3":
                    print("\n Target-Following Guider for Robot\n")
                    time.sleep(1.0)
                    UI.human_following()
                    UI.user_interface()
                    break
                if option == "4":
                    print("\n Moving to the target\n")
                    time.sleep(1.0)
                    UI.navigation_reach()
                    UI.user_interface()
                    break
                if option == "5":
                    print("\n Moving to the target with feedback\n")
                    time.sleep(1.0)
                    UI.navigation_auto()
                    UI.user_interface()
                    break
                if option == "6":
                    print("\n Moving to the target with collision avoidance\n")
                    time.sleep(1.0)
                    UI.navigation_laser()
                    UI.user_interface()
                    break
                if option == "7":
                    print("\n Move based on the map\n")
                    time.sleep(1.0)
                    UI.navigation_map()
                    UI.user_interface()
                    break
                if option == "8":
                    print("\n Automated alignment\n")
                    time.sleep(1.0)
                    UI.navigation_alignment()
                    UI.user_interface()
                    break
                if option == "9":
                    print("\n Dynamic obstacle avoidance \n")
                    time.sleep(1.0)
                    UI.navigation_dynamic()
                    UI.user_interface()
                    break
                if option == "10":
                    print("\n Exit \n")
                    time.sleep(1.0)
                    UI.exit()
                    break
                #if option == "03":
                #    UI.user_interface()
                #    break
                #if option == "04":
                #    #UI.lettuce_arm_control() ## Not Used ##
                #    UI.single_arm()
                #    UI.user_interface()
                #    break
            except KeyboardInterrupt:
                exit("close")


def main():
    ui = User_Interface()
    ui.user_interface()
    # ui.launch_perception_system()


if __name__ == "__main__":
    main()
