#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import os
import time

BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode
drawing_utils = mp.tasks.vision.drawing_utils
drawing_styles = mp.tasks.vision.drawing_styles
vision = mp.tasks.vision

class ComeModeState:
    IDLE = 0
    ROTATING = 1
    ROTATING_STABILIZING = 2  # 回正释放扭力阶段
    MOVING = 3
    STOP = 4

# ------------------- 关键参数 -------------------
CENTER_CONFIRM_FRAMES = 3
FULL_ROTATION_SPEED = 0.18      # 降低一点，更稳
SLOW_ROTATION_SPEED = 0.06
KP_ROTATION = 0.7
MIN_ROTATION_SPEED = 0.07
SOFT_LANDING_OFFSET_THRESHOLD = 0.015
STABILIZE_OFFSET_THRESHOLD = 0.008

RELEASE_TORQUE_SPEED = 0.03     # 超柔顺向减速，释放轮子扭力
RELEASE_TIME = 0.3              # 顺向释放时间
STABILIZE_DELAY = 1.5           # 最终静止等待时间

def draw_pose_landmarks_on_image(rgb_image, detection_result):
    pose_landmarks_list = detection_result.pose_landmarks
    annotated_image = np.copy(rgb_image)
    pose_landmark_style = drawing_styles.get_default_pose_landmarks_style()
    pose_connection_style = drawing_utils.DrawingSpec(color=(0,255,0), thickness=2)
    for pose_landmarks in pose_landmarks_list:
        drawing_utils.draw_landmarks(
            image=annotated_image, landmark_list=pose_landmarks,
            connections=vision.PoseLandmarksConnections.POSE_LANDMARKS,
            landmark_drawing_spec=pose_landmark_style, connection_drawing_spec=pose_connection_style)
    return annotated_image

class ComeModeNode(Node):
    def __init__(self):
        super().__init__('come_mode_node')
        self.current_state = ComeModeState.IDLE
        self.come_triggered = False
        self.shoulder_mid_x = 0.0
        self.person_detected = False
        self.target_distance = 0.6
        self.person_distance = -1.0
        self.frame_timestamp = 0
        self.latest_frame = None
        self.bridge = CvBridge()
        self.center_confirm_count = 0

        # 【新增】人脸关键点是否检测到（用于旋转定位前提条件）
        # 只有同时检测到人脸和两肩中点，才判定为"正面人"，允许进入旋转锁定
        self.face_detected = False

        # 释放扭力专用
        self.release_start_time = 0.0
        self.releasing_torque = False
        self.stabilize_start_time = 0.0

        self.detection_enabled = False
        self.window_name = "Come Mode"
        self.window_open = False

        self.create_subscription(String, '/gesture_cmd', self.gesture_cb, 10)
        self.create_subscription(String, '/car/control', self.voice_cb, 10)
        self.create_subscription(Float32, '/person_distance', self.distance_cb, 10)
        self.create_subscription(Image, '/camera_frame', self.frame_cb, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.pose_model_path = "/home/yhs/mediapipe_models/pose_landmarker_lite.task"
        self.pose_options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=self.pose_model_path),
            running_mode=VisionRunningMode.VIDEO, num_poses=1)
        self.pose_landmarker = PoseLandmarker.create_from_options(self.pose_options)
        self.timer = self.create_timer(1/30, self.main_loop)

    def gesture_cb(self, msg):
        if msg.data == "come" and self.current_state == ComeModeState.IDLE:
            self.come_triggered = True
    def voice_cb(self, msg):
        if msg.data == "come" and self.current_state == ComeModeState.IDLE:
            self.come_triggered = True
    def distance_cb(self, msg):
        self.person_distance = msg.data
    def frame_cb(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            self.latest_frame = None

    def detect_and_draw(self, frame):
        if not self.detection_enabled:
            return frame
        frame = cv2.flip(frame, 1)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
        self.frame_timestamp += 1
        res = self.pose_landmarker.detect_for_video(mp_image, self.frame_timestamp)
        if res.pose_landmarks:
            anno_rgb = draw_pose_landmarks_on_image(frame_rgb, res)
            out = cv2.cvtColor(anno_rgb, cv2.COLOR_RGB2BGR)
            lm = res.pose_landmarks[0]

            # ── 主位置判断依据：两肩中点 ──────────────────────────────────────────
            # 左肩=lm[11]，右肩=lm[12]，取水平方向归一化中点
            self.shoulder_mid_x = (lm[11].x + lm[12].x) / 2
            self.person_detected = True

            # ── 前提条件：人脸关键点检测 ──────────────────────────────────────────
            # MediaPipe Pose 中面部关键点索引：
            #   0=鼻尖, 2=左眼, 5=右眼（以及其他面部点）
            # 此处仅检测鼻尖和双眼的可见度（visibility > 0.5），
            # 满足任一即视为"检测到人脸"，代表当前是正面或基本正面姿态
            # 用途：只有同时检测到人脸 + 两肩中点，才判定为正面人，允许旋转定位
            face_visible = (
                lm[0].visibility > 0.85 or   # 鼻尖
                lm[2].visibility > 0.85 or   # 左眼
                lm[5].visibility > 0.85      # 右眼
            )
            self.face_detected = face_visible

            h, w = frame.shape[:2]
            cx = int(self.shoulder_mid_x * w)
            cy = int((lm[11].y + lm[12].y) / 2 * h)
            cv2.circle(out, (cx, cy), 8, (0, 0, 255), -1)
            return out
        else:
            # 未检测到姿态：清除人体和人脸标志
            self.person_detected = False
            self.face_detected = False
            return frame

    def main_loop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        if self.current_state == ComeModeState.IDLE:
            if self.come_triggered:
                self.current_state = ComeModeState.ROTATING
                self.detection_enabled = True
                self.come_triggered = False
                self.window_open = True
                self.center_confirm_count =0
                self.releasing_torque = False

        elif self.current_state == ComeModeState.ROTATING:
            # ══════════════════════════════════════════════════════════════════
            # 旋转定位阶段：前提条件 = 同时检测到人脸关键点 + 两肩中点
            #   ① 满足前提（正面人）：用肩中点偏移量计算角速度，比例渐进对准
            #   ② 不满足前提（侧身/无人）：持续右转搜寻，直到检测到正面人
            # 设计意图：防止侧身/遮挡时肩部误判导致错误锁定，只有正面人才触发定位
            # ══════════════════════════════════════════════════════════════════
            if self.person_detected and self.face_detected:
                # 正面人已检测到（肩部 + 人脸同时存在），开始精确对准
                offset = self.shoulder_mid_x - 0.5
                if 5/12 <= self.shoulder_mid_x <= 7/12:
                    # 肩中点已在画面中央区间（5/12 ~ 7/12）
                    self.center_confirm_count += 1
                    if abs(offset) > SOFT_LANDING_OFFSET_THRESHOLD:
                        # 软着陆：仍有微小偏差，用慢速微调消除惯性
                        twist.angular.z = SLOW_ROTATION_SPEED if offset > 0 else -SLOW_ROTATION_SPEED
                    else:
                        # 偏差极小，停止旋转
                        twist.angular.z = 0.0
                    if self.center_confirm_count >= CENTER_CONFIRM_FRAMES:
                        # 连续多帧确认居中，进入稳定/释放扭力阶段
                        self.current_state = ComeModeState.ROTATING_STABILIZING
                        self.releasing_torque = True
                        self.release_start_time = time.time()
                        self.center_confirm_count = 0
                else:
                    # 肩中点偏离中心，重置计数，用比例控制角速度旋转对准
                    self.center_confirm_count = 0
                    speed = KP_ROTATION * abs(offset)
                    speed = max(speed, MIN_ROTATION_SPEED)
                    speed = min(speed, FULL_ROTATION_SPEED)
                    twist.angular.z = speed if offset > 0 else -speed
            else:
                # 未检测到正面人（无人脸 or 侧身 or 完全无人）
                # 持续右转搜寻，直到检测到正面人（人脸+肩部同时出现）再对准
                self.center_confirm_count = 0
                twist.angular.z = FULL_ROTATION_SPEED

        # ===================== 【核心：释放扭力 + 不打滑】 =====================
        elif self.current_state == ComeModeState.ROTATING_STABILIZING:
            if not self.person_detected:
                self.current_state = ComeModeState.ROTATING
                return

            offset = self.shoulder_mid_x - 0.5

            # 第一步：顺向超柔旋转，释放轮子扭力
            if self.releasing_torque:
                if time.time() - self.release_start_time < RELEASE_TIME:
                    twist.angular.z = RELEASE_TORQUE_SPEED if offset > 0 else -RELEASE_TORQUE_SPEED
                else:
                    self.releasing_torque = False
                    self.stabilize_start_time = time.time()
            # 第二步：完全静止等待1.5秒
            else:
                twist.angular.z = 0.0
                if time.time() - self.stabilize_start_time >= STABILIZE_DELAY:
                    self.current_state = ComeModeState.MOVING

        elif self.current_state == ComeModeState.MOVING:
            # 前进阶段：不再需要人脸，只用两肩中点判断是否仍检测到人体
            # 设计意图：前进时人可能侧身或部分遮挡，只要肩部可见就继续跟进
            if not self.person_detected:
                self.current_state = ComeModeState.ROTATING
            elif self.person_distance > self.target_distance:
                twist.linear.x = 0.13
            else:
                twist.linear.x =0.0
                self.current_state = ComeModeState.STOP

        elif self.current_state == ComeModeState.STOP:
            self.detection_enabled = False
            self.window_open = False
            self.current_state = ComeModeState.IDLE

        # -------------------- 绘制 --------------------
        if self.latest_frame is not None:
            display = self.latest_frame.copy()
            if self.detection_enabled:
                display = self.detect_and_draw(display)
            if self.window_open:
                state_map = {0:"IDLE",1:"ROTATE",2:"STABILIZING",3:"MOVING",4:"STOP"}
                s = state_map[self.current_state]
                cv2.putText(display,f"State:{s}",(20,40),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)
                cv2.imshow(self.window_name, display)
                cv2.waitKey(1)
            else:
                try:cv2.destroyWindow(self.window_name)
                except:pass

        if self.current_state != ComeModeState.IDLE:
            self.vel_pub.publish(twist)

    def destroy_node(self):
        cv2.destroyAllWindows()
        self.pose_landmarker.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ComeModeNode()
    try:rclpy.spin(node)
    except:pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
