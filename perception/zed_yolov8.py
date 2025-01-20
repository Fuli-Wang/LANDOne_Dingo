#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
import pyzed.sl as sl
import logging
import time
import cv2
from PIL import Image
from ultralytics import YOLO
import numpy as np
import statistics
import math
import subprocess

# Initialize ROS node and publisher
rospy.init_node('publish_coordinates', anonymous=True)
publisher = rospy.Publisher('/target/coordinates', Point, queue_size=10)
rate = rospy.Rate(10)  # 10hz

def get_object_depth(depth, bounds):
    '''
    Calculates the median x, y, z position of top slice(area_div) of point cloud
    in camera frame.
    Arguments:
        depth: Point cloud data of whole frame.
        bounds: Bounding box for object in pixels.
            bounds[0]: x-center
            bounds[1]: y-center
            bounds[2]: width of bounding box.
            bounds[3]: height of bounding box.
    Return:
        x, y, z: Location of object in millimeters.
    '''
    area_div = 2

    x_vect = []
    y_vect = []
    z_vect = []

    for j in range(int(bounds[0] - area_div), int(bounds[0] + area_div)):
        for i in range(int(bounds[1] - area_div), int(bounds[1] + area_div)):
            z = depth[i, j, 2]
            if not np.isnan(z) and not np.isinf(z):
                x_vect.append(depth[i, j, 0])
                y_vect.append(depth[i, j, 1])
                z_vect.append(z)
    try:
        x_median = statistics.median(x_vect)
        y_median = statistics.median(y_vect)
        z_median = statistics.median(z_vect)
    except Exception:
        x_median = -1
        y_median = -1
        z_median = -1
        pass

    return x_median, y_median, z_median

def takeSecond(elem):
    return elem[2]

def publish_coordinates(point):

    point_msg = Point()
    point_msg.x = point[0]
    point_msg.y = point[1]
    point_msg.z = point[2]
    publisher.publish(point_msg)
    rate.sleep()

def detect(image, depth):
    # Load a pretrained YOLOv8n model
    model = YOLO('yolov8n-pose.pt')

    # Run inference on an image
    #im0s = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    img = image[:, :, :3]
    results = model(img)  # results list
    # View results
    coordinates = []
    pose = 0.0
    for r in results:
        im_array = r.plot()  # plot a BGR numpy array of predictions
        im = Image.fromarray(im_array[..., ::-1])  # RGB PIL image

        labels = r.names

        boxes = r.boxes  # the Boxes object containing the detection bounding boxes
        keypoints = r.keypoints.xy
        keypoints_xy = keypoints.cpu().numpy()
        xywh = (boxes.xywh).view(-1).tolist()
        bounds=tuple(xywh)
        null_tuple = ()
        if bounds != null_tuple:
            for i in range(len(boxes.xywh)):
                bounds=tuple(boxes.xywh[i])
                x_cen, y_cen, z_cen = get_object_depth(depth, bounds)
                cls = boxes.cls[i].view(-1).tolist()
                origin = [0, 0, 0]
                target_position = [x_cen, y_cen, z_cen]
                distance = round(math.dist(target_position, origin))
                if distance<200:
                    print("The target is too close")
                    coordinates.append(origin)
                elif distance>9000:
                    print("The target is too far away")
                    coordinates.append(origin)
                else:
                    print("The person is in the following area")
                    coordinates.append(target_position)
                #print(labels[cls[0]], x_cen, y_cen, z_cen)
                try:
                    keypoints_xy = keypoints_xy[0]
                    right_wrist = keypoints_xy[10]
                    left_wrist = keypoints_xy[9]
                    shoulder = keypoints_xy[5]
                    if right_wrist[1]<=shoulder[1] and left_wrist[1]<=shoulder[1]: #hands up
                        print("I detect a stop pose")
                        print("I will not do any movements")
                        #coordinates.append([-999, -999, -999])
                        #subprocess.run(['pkill', '-f', 'python'], check=True)
                    elif right_wrist[1]<=shoulder[1] and left_wrist[1]>shoulder[1]: #right hand up
                        pose = 7.7
                    elif right_wrist[1]>shoulder[1] and left_wrist[1]<=shoulder[1]: #left hand up
                        pose = 6.6
                    else:
                        pose = 0.0
                except:
                    continue
            coordinates.sort(reverse=False, key=takeSecond)
    nearest_position = [0.0, 0.0, 0.0]
    if coordinates:
        #file_handle=open('/home/fuli/Documents/Dingo/Calibration/targetposition.txt',mode='w')
        guide = coordinates[0]
        coordinates = []
        nearest_position = [guide[0], pose, guide[2]]
        print(nearest_position)
        #file_handle.writelines("%f		%f	 	%f 	    %f    \n" % (guide[0],guide[1],guide[2],pose))
        try:
            publish_coordinates(nearest_position)
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS Interrupt Exception")




    cv2.imshow("ZED",results[0].plot())
    key = cv2.waitKey(5)
    log.info("FPS: {}".format(1.0 / (time.time() - start_time)))


if __name__ == '__main__':
    log = logging.getLogger(__name__)
    logging.basicConfig(level=logging.INFO)


    zed_id=0
    input_type = sl.InputType()

    # Launch camera by id

    input_type.set_from_camera_id(zed_id)

    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
    init.camera_fps = 30  # Set fps at 30
    init.coordinate_units = sl.UNIT.MILLIMETER
    init.depth_minimum_distance = 300 # Set the minimum depth perception distance to 30cm

    cam = sl.Camera()
    if not cam.is_opened():
        log.info("Opening ZED Camera...")
    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        log.error(repr(status))
        exit()
    runtime = sl.RuntimeParameters()

    mat = sl.Mat()
    point_cloud_mat = sl.Mat()

    log.info("Running...")

    while not rospy.is_shutdown():  # for 'q' key
        start_time = time.time() # start time of the loop
        err = cam.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            cam.retrieve_image(mat, sl.VIEW.LEFT)
            image = mat.get_data()

            cam.retrieve_measure(
                point_cloud_mat, sl.MEASURE.XYZRGBA)
            depth = point_cloud_mat.get_data()
            detect(image, depth)
            #cv2.imshow("ZED",)
        else:
            key = cv2.waitKey(5)
    cv2.destroyAllWindows()

    cam.close()
    log.info("\nFINISH")
