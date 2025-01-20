#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
import cv2
import numpy as np
import pyzed.sl as sl
import math
from sort import *  # Import all from sort.py
from ultralytics import YOLO
from collections import defaultdict

# Initialize the ZED camera
zed = sl.Camera()

# Create a ZED configuration object
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
init_params.coordinate_units = sl.UNIT.MILLIMETER
init_params.depth_minimum_distance = 300 # Set the minimum depth perception distance to 30cm

# Open the camera
if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    exit(1)

# Create objects to store depth, point cloud, and image
runtime_parameters = sl.RuntimeParameters()

image_zed = sl.Mat()
depth_map = sl.Mat()
point_cloud = sl.Mat()

# Load the YOLOv11 model from ultralytics
model = YOLO('yolo11n-pose.pt')  # Load your model (e.g., yolov11n.pt)

# Create an instance of the SORT tracker
mot_tracker = Sort()

# Dictionary to keep track of positions and velocities
track_history = defaultdict(lambda: {"positions": [], "status": 0})

# Initialize ROS node and publisher
rospy.init_node('publish_coordinates', anonymous=True)
publisher = rospy.Publisher('/target/motion', Point, queue_size=10)
xyz_publisher = rospy.Publisher('/target/coordinates', Point, queue_size=10)
rate = rospy.Rate(10)  # 10hz

# Function to calculate the 3D coordinates from ZED point cloud
def get_3d_coordinates(x, y, point_cloud):
    err, point_cloud_value = point_cloud.get_value(x, y)
    if err == sl.ERROR_CODE.SUCCESS and not np.isnan(point_cloud_value).any():
        X, Y, Z = point_cloud_value[0], point_cloud_value[1], point_cloud_value[2]
    else:
        X, Y, Z = float('nan'), float('nan'), float('nan')  # Handle invalid data
    return X, Y, Z


# Function to calculate movement status based on XZ plane
def calculate_movement_status(track_id, new_position):
    history = track_history[track_id]["positions"]
    if len(history) > 0:
        # Calculate displacement as average over the last few frames for smoothing

        displacement = np.linalg.norm(np.array(new_position) - np.array(history[-1]))  # Assign a default value or handle appropriately

        # Adjust these thresholds as necessary based on testing
        print(f"Displacement: {displacement}")
        if displacement < 30:  # Stationary threshold
            status = 0  # Stationary
        elif 30 <= displacement < 50:  # About to move threshold
            status = 1  # About to move
        else:  # Moving threshold
            status = 2  # Moving

        track_history[track_id]["status"] = status  # Store as numeric value

    # Update position history
    track_history[track_id]["positions"].append(new_position[:2])  # Store only X and Z

# Function to calculate the direction of motion relative to the robot
def calculate_direction(track_id):
    distance_threshold = 2.5  # This value can be fine-tuned based on your specific environment and noise level
    history = track_history[track_id]["positions"]
    if len(history) > 1:
        prev_position = np.array(history[-2])
        current_position = np.array(history[-1])

        # Calculate direction vector
        direction_vector = current_position - prev_position
        robot_origin = np.array([0.0, 0.0])

        prev_distance = np.linalg.norm(prev_position - robot_origin)
        current_distance = np.linalg.norm(current_position - robot_origin)

        # Determine direction using the threshold to detect lateral movement
        if abs(current_distance - prev_distance) < distance_threshold:
            return 2  # Lateral movement
        elif current_distance < prev_distance:
            return 0  # Moving towards the robot
        else:
            return 1  # Moving away from the robot

    return 2  # Default lateral movement if not enough history



# Main loop
while not rospy.is_shutdown():
    # Grab an image and depth map from ZED
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

        # Convert ZED image to OpenCV format
        frame = image_zed.get_data()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        # YOLOv8 detection
        results = model(frame)
        dets = []

        # Parse YOLOv8 detections
        for result in results:
            for detection in result.boxes:
                x1, y1, x2, y2 = detection.xyxy[0].cpu().numpy()  # Extract bounding box and move to CPU
                conf = detection.conf[0].cpu().numpy()  # Confidence score, move to CPU
                class_id = int(detection.cls[0].cpu().numpy())  # Class ID, move to CPU

                if conf > 0.5 and class_id == 0:  # Assuming class_id 0 is 'person'
                    dets.append([x1, y1, x2, y2, conf])

        dets = np.array(dets)

        # Only update SORT if there are detections and they are correctly formatted
        if dets.shape[0] > 0 and dets.shape[1] == 5:
            track_bbs_ids = mot_tracker.update(dets)
        else:
            # Handle cases where no valid detections are found
            track_bbs_ids = np.empty((0, 5))

        # Process the tracks and find the nearest object based on X, Y coordinates
        nearest_distance = float('inf')
        nearest_track_id = -1
        origin = [0.0, 0.0] # Robot origin in the X-Y plane
        for track in track_bbs_ids:
            x1, y1, x2, y2, track_id = track.astype(int)

            # Draw bounding box and track ID
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'ID: {track_id}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Get the 3D coordinates from ZED
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            X, Y, Z = get_3d_coordinates(center_x, center_y, point_cloud)

            if not np.isnan(X) and not np.isnan(Z):
                # Calculate movement status
                calculate_movement_status(track_id, (X, Z))
                distance = math.dist([X, Z], origin)
                if distance < nearest_distance:
                    nearest_track_id = track_id
                    nearest_distance = distance

                # Get status
                status = track_history[track_id]["status"]
                
                # Status text mapping
                status_text_mapping = {
                0: "stationary",
                1: "about to move",
                2: "moving"}

                # Get the text label for the current status
                status_text = status_text_mapping.get(status, "unknown")

                # Display the status on the frame
                cv2.putText(frame, f'Status: {status_text}', (x1, y2 + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f'X: {X:.2f}, Y: {Y:.2f}, Z: {Z:.2f}', (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                # Handle cases where the XYZ data is invalid
                cv2.putText(frame, f'Invalid Depth', (x1, y2 + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        #Publish the nearest object's distance, status, and direction
        if nearest_track_id != -1:
            try:
                nearest_track = [track for track in track_bbs_ids if int(track[4]) == nearest_track_id]
                
                if len(nearest_track) > 0:
                    x1, y1, x2, y2, track_id = nearest_track[0].astype(int)
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    X, Y, Z = get_3d_coordinates(center_x, center_y, point_cloud)
                    
                    status_id = track_history[nearest_track_id]["status"] 
                    
                    direction = calculate_direction(nearest_track_id)

                    rospy.loginfo(f'Distance: {nearest_distance}, Status: {status_id}, Direction: {direction}')

                    point_msg = Point()
                    point_msg.x = float(nearest_distance)  # Distance from the robot to the object (on X-Z plane)
                    point_msg.y = float(status_id)  # Status (stationary, about to move, moving)
                    point_msg.z = float(direction)  # Direction (towards, away, lateral)

                    publisher.publish(point_msg)

                    # Publish nearest object's XYZ coordinates if valid
                    if not np.isnan(X) and not np.isnan(Y) and not np.isnan(Z):
                        xyz_msg = Point()
                        xyz_msg.x = X  # Nearest object's X coordinate
                        xyz_msg.y = Y  # Nearest object's Y coordinate
                        xyz_msg.z = Z  # Nearest object's Z coordinate
                        xyz_publisher.publish(xyz_msg)

                    rate.sleep()

            except rospy.ROSInterruptException:
                rospy.loginfo("ROS Interrupt Exception")

        # Display the resulting frame
        cv2.imshow("Frame", frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release resources
cv2.destroyAllWindows()
zed.close()
