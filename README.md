# LANDOne_Dingo
The Dingo mobile robot system based on ROS1

# Dependency
This is the ROS1 (Noetic) implementation based on Ubuntu 20.04

To get started with the Dingo simulation, make sure you have a working ROS installation set up on your Ubuntu desktop, and install the Dingo-specific metapackages for desktop and simulation:

    sudo apt-get install ros-noetic-dingo-simulator ros-noetic-dingo-desktop

More information about the Dingo's basic tutorial: [Dingo Tutorila](https://docs.clearpathrobotics.com/docs/ros1noetic/robots/indoor_robots/dingo/tutorials_dingo#simulating-dingo).

# Interfacing with Dingo
First, ensure that Dingo and your PC/Laptop are running properly and on the same LAN (WiFi). The remote-dingoA.sh file can be configured as required. Then in your PC terminal：

    source remote-dingoA.sh

    python3 User_Interface.py

Now you can access the user interface of the system. It looks like below.

"################################################################################"

" Dingo Control "

 "################################################################################
 - 1: Launch the Perception System:
 - 2: Basic Navigation
 - 3: Target-Following Guider for mobile robot
 - 4: Directly approaching to the target(camera only)
 - 5: Approaching the target with a camera feedback loop
 - 6: Approaching the target with collision avoidance(camera + LIDAR)
 - 7: Move based on the map
 -  8: Alignment based on the camera
 -  9: Recognizing the state of moving objects (under development)
 -  10: Exit
   ################################################################################


    
  
