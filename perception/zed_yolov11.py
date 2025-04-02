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
import csv
import os

# ---------------- 全局参数及手势模板 ----------------
gesture_buffer = []         # 用于存储连续帧归一化关键点的列表
MIN_GESTURE_FRAMES = 2     # 触发手势识别所需最少帧数

def load_gesture_templates():
    """
    模拟加载预先录入的手势模板数据，
    每个模板由若干帧归一化后的关键点数据构成，
    实际应用中请使用真实标注数据。
    """
    templates = {}
    num_frames = 15     # 每个模板序列由15帧组成
    num_keypoints = 17  # 假设 YOLO 返回 17 个关键点
    # 模拟“停止”手势：双手交叉于胸前（归一化后手腕位置靠近中线且较低）
    templates['stop'] = [np.random.rand(num_keypoints, 2)*0.2 - 0.1 for _ in range(num_frames)]
    # 模拟“左手挥动”：左手抬高（归一化后左手明显高于左肩）
    templates['left_wave'] = [np.random.rand(num_keypoints, 2)*0.2 + 0.1 for _ in range(num_frames)]
    # 模拟“右手挥动”：右手抬高（归一化后右手明显高于右肩）
    templates['right_wave'] = [np.random.rand(num_keypoints, 2)*0.2 - 0.1 for _ in range(num_frames)]
    # 模拟“T 型姿势”：双臂水平展开（左右手分别位于各自肩的左右侧，并且高度接近）
    templates['t_pose'] = [np.random.rand(num_keypoints, 2)*0.2 for _ in range(num_frames)]
    return templates

gesture_templates = load_gesture_templates()
joint_weights = [1.0] * 17  # 关节权重，可根据实际情况调优

# 如果 CSV 文件不存在，则写入表头
if not os.path.exists("frame_metrics.csv"):
    with open("frame_metrics.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "frame_time_sec", "fps"])
if not os.path.exists("gesture_metrics.csv"):
    with open("gesture_metrics.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "gesture_result", "dtw_distance", "dtw_time_sec"])

# ---------------- 关键函数定义 ----------------
def normalize_keypoints(kps):
    """
    以左右肩（索引5和6）的中点为原点，
    并以两肩距离作为尺度对关键点进行归一化。
    """
    left_shoulder = kps[5]
    right_shoulder = kps[6]
    origin = (left_shoulder + right_shoulder) / 2.0
    scale = np.linalg.norm(left_shoulder - right_shoulder)
    if scale == 0:
        scale = 1.0
    norm_kps = (kps - origin) / scale
    return norm_kps

def weighted_dtw(seq1, seq2, weights):
    """
    采用加权动态时间规整（DTW）计算两个手势序列之间的匹配距离。
    seq1, seq2: 列表形式的手势序列，每个元素为 (num_keypoints, 2) 的归一化关键点数组
    weights: 长度为 num_keypoints 的关节权重列表
    """
    n = len(seq1)
    m = len(seq2)
    DTW = np.full((n + 1, m + 1), np.inf)
    DTW[0, 0] = 0
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            d = 0
            for k in range(len(weights)):
                d += weights[k] * np.linalg.norm(seq1[i - 1][k] - seq2[j - 1][k])
            DTW[i, j] = d + min(DTW[i - 1, j], DTW[i, j - 1], DTW[i - 1, j - 1])
    return DTW[n, m]

def check_kinematic_constraints(norm_kps):
    """
    根据运动学约束判断当前帧是否满足触发条件。
    示例条件：若双手交叉于胸前（即两手腕水平距离小于肩宽一半，且均低于各自肩部），返回 True。
    """
    left_wrist = norm_kps[9]
    right_wrist = norm_kps[10]
    left_shoulder = norm_kps[5]
    right_shoulder = norm_kps[6]
    shoulder_dist = abs(left_shoulder[0] - right_shoulder[0])
    if abs(left_wrist[0] - right_wrist[0]) < 0.5 * shoulder_dist and \
       (left_wrist[1] > left_shoulder[1] and right_wrist[1] > right_shoulder[1]):
        return True
    return False

def baseline_recognition(norm_kps):
    """
    基于单帧静态规则进行判断，返回五种状态之一：
    stop, left_wave, right_wave, t_pose, no_gestures。
    定义条件如下：
      - 若运动学约束成立（双手交叉于胸前），返回 'stop'
      - 若左手明显高于左肩，而右手未高于右肩，返回 'left_wave'
      - 若右手明显高于右肩，而左手未高于左肩，返回 'right_wave'
      - 若两手都接近肩部水平（与肩部垂直距离小于阈值）且分别位于各自肩的外侧，则返回 't_pose'
      - 其他情况返回 'no_gestures'
    """
    left_wrist = norm_kps[9]
    right_wrist = norm_kps[10]
    left_shoulder = norm_kps[5]
    right_shoulder = norm_kps[6]
    # 条件1：stop
    if check_kinematic_constraints(norm_kps):
        return 'stop'
    # 条件2：left_wave
    if left_wrist[1] < left_shoulder[1] and not (right_wrist[1] < right_shoulder[1]):
        return 'left_wave'
    # 条件3：right_wave
    if right_wrist[1] < right_shoulder[1] and not (left_wrist[1] < left_shoulder[1]):
        return 'right_wave'
    # 条件4：t_pose：左右手与肩部高度接近且分别位于肩的外侧
    threshold = 1000
    #if abs(left_wrist[1] - left_shoulder[1]) < threshold and abs(right_wrist[1] - right_shoulder[1]) < threshold:
    if left_wrist[1] < left_shoulder[1] and right_wrist[1] > right_shoulder[1]:
        return 't_pose'
    # 若均不满足，则返回 no_gestures
    return 'no_gestures'

def gesture_recognition(gesture_seq):
    """
    利用加权 DTW 将采集到的手势序列与模板匹配，
    返回最匹配的手势名称及对应的 DTW 距离。
    """
    best_match = None
    best_distance = np.inf
    for gesture_name, template_seq in gesture_templates.items():
        dtw_distance = weighted_dtw(gesture_seq, template_seq, joint_weights)
        if dtw_distance < best_distance:
            best_distance = dtw_distance
            best_match = gesture_name
    return best_match, best_distance

def get_object_depth(depth, bounds):
    """
    计算给定 bounding box 区域内点云数据的中位数深度（x, y, z）。
    """
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
        x_median, y_median, z_median = -1, -1, -1
    return x_median, y_median, z_median

def publish_coordinates(point, publisher, rate):
    point_msg = Point()
    point_msg.x = point[0]
    point_msg.y = point[1]
    point_msg.z = point[2]
    publisher.publish(point_msg)
    rate.sleep()

# ---------------- 检测函数 ----------------
def detect(image, depth, publisher, rate):
    global gesture_buffer
    # 加载 YOLO-pose 模型
    model = YOLO('yolo11n-pose.pt')
    img = image[:, :, :3]
    results = model(img)
    coordinates = []
    # 初始状态设置为 no_gestures（即0.0）
    pose_state = 0.0
    recognized_gesture = "no_gestures"  # 显示的状态文字

    for r in results:
        im_array = r.plot()  # 获取带检测框的图像数组
        labels = r.names
        boxes = r.boxes  # 检测到的 bounding boxes
        keypoints = r.keypoints.xy  # 检测到的关键点信息
        keypoints_np = keypoints.cpu().numpy()  # shape: (num_detections, num_keypoints, 2)
        if len(boxes.xywh) == 0:
            continue

        for i in range(len(boxes.xywh)):
            bounds = tuple(boxes.xywh[i])
            x_cen, y_cen, z_cen = get_object_depth(depth, bounds)
            cls = boxes.cls[i].view(-1).tolist()
            origin = [0, 0, 0]
            target_position = [x_cen, y_cen, z_cen]
            distance = round(math.dist(target_position, origin))
            if distance < 200:
                print("The target is too close")
                coordinates.append(origin)
            elif distance > 9000:
                print("The target is too far away")
                coordinates.append(origin)
            else:
                coordinates.append(target_position)

            try:
                # 处理当前检测目标的关键点数据
                kps = keypoints_np[i]  # shape: (num_keypoints, 2)
                norm_kps = normalize_keypoints(kps)
                # 使用静态规则得到基线结果
                baseline_result = baseline_recognition(norm_kps)
                print("Baseline static recognition:", baseline_result)
                # 默认将静态识别结果作为当前状态
                if baseline_result in ['stop', 'left_wave', 'right_wave', 't_pose']:
                    recognized_gesture = baseline_result
                    if baseline_result == 'stop':
                        pose_state = 1.0
                    elif baseline_result == 'left_wave':
                        pose_state = 2.0
                    elif baseline_result == 'right_wave':
                        pose_state = 3.0
                    elif baseline_result == 't_pose':
                        pose_state = 4.0
                else:
                    recognized_gesture = "no_gestures"
                    pose_state = 0.0

                # 利用运动学约束作为触发条件采集手势序列
                if check_kinematic_constraints(norm_kps):
                    gesture_buffer.append(norm_kps)
                else:
                    if len(gesture_buffer) >= MIN_GESTURE_FRAMES:
                        dtw_start = time.time()
                        recog, dtw_distance = gesture_recognition(gesture_buffer)
                        dtw_time = time.time() - dtw_start
                        print("DTW-based recognition:", recog, "distance:", dtw_distance, "time:", dtw_time)
                        # 若DTW返回的识别属于有效状态，则更新状态；否则设为 no_gestures
                        if recog in ['stop', 'left_wave', 'right_wave', 't_pose']:
                            if recog == 'stop':
                                pose_state = 1.0
                            elif recog == 'left_wave':
                                pose_state = 2.0
                            elif recog == 'right_wave':
                                pose_state = 3.0
                            elif recog == 't_pose':
                                pose_state = 4.0
                            recognized_gesture = recog
                            # 记录DTW指标
                            with open("gesture_metrics.csv", "a", newline="") as f:
                                writer = csv.writer(f)
                                writer.writerow([time.time(), recog, dtw_distance, dtw_time])
                        else:
                            pose_state = 0.0
                            recognized_gesture = "no_gestures"
                            recog = "no_gestures"
                        gesture_buffer = []  # 清空手势缓存
            except Exception as e:
                print("Keypoints processing error:", e)
                continue

    # 对检测到的目标按 y 值排序（以便选取最近的目标）
    coordinates.sort(key=lambda elem: elem[1])
    nearest_position = [0.0, 0.0, 0.0]
    if coordinates:
        guide = coordinates[0]
        # 若状态为 no_gestures，则传输的pose部分保持0
        if pose_state == 0.0:
            nearest_position = [guide[0], 0.0, guide[2]]
        else:
            nearest_position = [guide[0], pose_state, guide[2]]
        print("Final position and pose:", nearest_position)
        # 只有识别到有效姿态时才发布坐标
        if pose_state != 0.0:
            try:
                publish_coordinates(nearest_position, publisher, rate)
            except rospy.ROSInterruptException:
                rospy.loginfo("ROS Interrupt Exception")

    # 在图像上显示识别到的状态
    cv2.putText(im_array, "Gesture: " + recognized_gesture, (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("ZED", im_array)
    key = cv2.waitKey(5)
    # 记录本帧处理时间与FPS
    frame_time = time.time() - start_time
    fps = 1.0 / frame_time if frame_time > 0 else 0
    with open("frame_metrics.csv", "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([time.time(), frame_time, fps])
    log.info("FPS: {}".format(fps))

# ---------------- 主程序 ----------------
if __name__ == '__main__':
    log = logging.getLogger(__name__)
    logging.basicConfig(level=logging.INFO)

    rospy.init_node('publish_coordinates', anonymous=True)
    publisher = rospy.Publisher('/target/coordinates', Point, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # 初始化 ZED 相机参数
    zed_id = 0
    input_type = sl.InputType()
    input_type.set_from_camera_id(zed_id)
    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.camera_fps = 60
    init.coordinate_units = sl.UNIT.MILLIMETER
    init.depth_minimum_distance = 300

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
    while not rospy.is_shutdown():
        start_time = time.time()  # 当前帧FPS计算起始时间
        err = cam.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            cam.retrieve_image(mat, sl.VIEW.LEFT)
            image = mat.get_data()
            cam.retrieve_measure(point_cloud_mat, sl.MEASURE.XYZRGBA)
            depth = point_cloud_mat.get_data()
            detect(image, depth, publisher, rate)
        else:
            key = cv2.waitKey(5)
    cv2.destroyAllWindows()
    cam.close()
    log.info("\nFINISH")
