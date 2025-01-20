# LANDOne_Dingo
The Dingo mobile robot system based on ROS1

# Dependency
This is the ROS1 (Noetic) implementation based on Ubuntu 20.04

To get started with the Dingo simulation, make sure you have a working ROS installation set up on your Ubuntu desktop, and install the Dingo-specific metapackages for desktop and simulation:

    sudo apt-get install ros-noetic-dingo-simulator ros-noetic-dingo-desktop

More information about the Dingo's basic tutorial: [Dingo Tutorila](https://docs.clearpathrobotics.com/docs/ros1noetic/robots/indoor_robots/dingo/tutorials_dingo#simulating-dingo).

# Sensors of our platform:

![platform](https://github.com/Fuli-Wang/LANDOne_Dingo/blob/main/platform.jpg) 

Sensors are used herein: ZED camera, Velodyne-16 LIDAR and IMU.

For the ZED camera, please install the SDK: [ZED SDK](https://www.stereolabs.com/en-gb/developers) and the required CUDA toolkit.

LIDAR(Velodyne-16):

    sudo apt-get install ros-noetic-velodyne
    
Recommended install gmapping for testing:

    sudo apt-get install ros-noetic-gmapping
    sudo apt-get install ros-noetic-laser-scan-matcher

IMU(3DM-CV7-AHRS):

    sudo apt-get update && sudo apt-get install ros-noteic-microstrain-inertial-driver

To activate the IMU, open a new terminal:

    roslaunch microstrain_inertial_driver microstrain.launch params_file:=Calibration/IMU/my_params.yml
    
For calibration of the IMU, you may need to install and use a tool to complete it. For example, you can install [Ceres Solver](https://www.stereolabs.com/en-gb/developers) and [code_utils](https://github.com/gaowenliang/code_utils) Then install and use this tool for calibration: [imu_utils](https://github.com/gaowenliang/imu_utils) Then you can calibrate the IMU and update the cv7_imu_param.yaml in Calibration/IMU. This would help to use SLAM via LIDAR and IMU, refer: [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)

# Interfacing with Dingo
option 1 (interface with physical robot): 
First, ensure that Dingo and your PC/Laptop are running properly and on the same LAN (WiFi). The remote-dingoA.sh file can be configured as required. Then in your PC terminal：

    source remote-dingoA.sh

    python3 User_Interface.py

option 2 (interface with simulated robot): 
Besides interfacing with physical robots, highly recommended to use the simulation environment to do testing. No need to configure IP, just launch the simulator and run the User Interface:

If you want to use an omnidirectional drive Dingo in simulation, run:

    export DINGO_OMNI=1
    
Then:

    roslaunch dingo_gazebo dingo_world.launch config:=front_laser

    python3 User_Interface.py

Please be aware that this simulation is using front_laser instead of using Velodyne-16, you may need to change Navigation_Laser.py to do relevant testing. For example, "/scan" in Line 25 can be changed to "front/scan" and in the function navigation_robot_laser_sensor, the parameters can be changed as well, please refer to LIDAR ranges.

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

 Press 1 to launch the basic perception system via the ZED camera to detect/localise the person. When this perception system is turned on/activated, you can continue to use the user interface terminal:

2 (using the keyboard to control the robot), 3 (the robot will follow the nearest person in the field of view), 4 (approaching the target directly), and 5 (approaching the target based on the camera's real-time feedback).

Function 6 is needed to activate LIDAR first, you need to open a new terminal:

    roslaunch velodyne_pointcloud VLP16_points.launch

Function 7 (not recommended use): the robot will move based on the SLAM and path planning algorithm's output. This function is more focused on research, it needs to use SLAM to map the environment and use a path planning algorithm to generate the path, the robot will move based on the path. There is an example of a path planning algorithm in /path planning.

Function 8: This function uses the camera to detect/localise a QR code and the robot will do the alignment (make sure the QR code is in the central view of the robot). Please shut down function 1 (no need to activate  perception function), as the function will automatical load and open the required perception algorithm.

Function 9: This function uses the camera to detect/localise the moving object, the robot will stop or continue the movement based on the object's status. Please shut down function 1 (no need to activate  perception function), as the function will automatical load and open the required perception algorithm. (research phase, it is currently possible to recognize dynamic humans).
 


    
  
