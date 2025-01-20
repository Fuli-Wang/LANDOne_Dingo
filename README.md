# LANDOne_Dingo
The Dingo mobile robot system based on ROS1

# Dependency
This is the ROS1 (Noetic) implementation based on Ubuntu 20.04

To get started with the Dingo simulation, make sure you have a working ROS installation set up on your Ubuntu desktop, and install the Dingo-specific metapackages for desktop and simulation:

    sudo apt-get install ros-noetic-dingo-simulator ros-noetic-dingo-desktop

More information about the Dingo's basic tutorial: [Dingo Tutorila](https://docs.clearpathrobotics.com/docs/ros1noetic/robots/indoor_robots/dingo/tutorials_dingo#simulating-dingo).

Sensors of our platform:

For the ZED camera, please install the SDK: [ZED SDK](https://www.stereolabs.com/en-gb/developers) and the required CUDA toolkit.

LIDAR(velodyne-16):

    sudo apt-get install ros-noetic-velodyne
    
Recommended install gmapping for testing:

    sudo apt-get install ros-noetic-gmapping
    sudo apt-get install ros-noetic-laser-scan-matcher

IMU(3DM-CV7-AHRS):

    sudo apt-get update && sudo apt-get install ros-noteic-microstrain-inertial-driver
    
For calibration of the IMU, you may need to install and use a tool to complete it. For example, you can install [Ceres Solver](https://www.stereolabs.com/en-gb/developers) and [code_utils](https://github.com/gaowenliang/code_utils) Then install and use this tool for calibration: [imu_utils](https://github.com/gaowenliang/imu_utils) 

# Interfacing with Dingo
First, ensure that Dingo and your PC/Laptop are running properly and on the same LAN (WiFi). The remote-dingoA.sh file can be configured as required. Then in your PC terminal：

    source remote-dingoA.sh

    python3 User_Interface.py

Now you can access the user interface of the system. It looks like below.

################################################################################

 Dingo Control 
 
 ################################################################################
 - 1: Launch the Perception System:
 - 2: Basic Navigation
 - 3: Target-Following guide for mobile robot
 - 4: Directly approaching to the target(camera only)
 - 5: Approaching the target with a camera feedback loop
 - 6: Approaching the target with collision avoidance(camera + LIDAR)
 - 7: Move based on the map
 - 8: Alignment based on the camera
 - 9: Recognizing the state of moving objects (under development)
 - 10: Exit
###############################################################################

 Press 1 to launch the basic perception system via the ZED camera to detect/localise the person. When this perception system is turned on, you can continue to enter in the user interface terminal:

2 (using the keyboard to control the robot), 3 (the robot will follow the nearest person in the field of view), 4 (approaching the target directly), 5 (approaching the target based on the camera's real-time feedback).

Function 6 is needed to activate LIDAR:

    sudo apt-get update && sudo apt-get install ros-noteic-microstrain-inertial-driver
 


    
  
