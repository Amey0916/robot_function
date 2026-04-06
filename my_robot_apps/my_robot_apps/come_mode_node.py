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

# MediaPipe 工具
BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode
drawing_utils = mp.tasks.vision.drawing_utils
drawing_styles = mp.tasks.vision.drawing_styles
vision = mp.tasks.vision

# 状态定义
class ComeModeState:
    IDLE = 0
    ROTATING = 1
    MOVING = 2
    STOP = 3

# 人体骨架绘制函数
def draw_pose_landmarks_on_image(rgb_image, detection_result):
    pose_landmarks_list = detection_result.pose_landmarks
    annotated_image = np.copy(rgb_image)
    pose_landmark_style = drawing_styles.get_default_pose_landmarks_style()
    pose_connection_style = drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2)

    for pose_landmarks in pose_landmarks_list:
        drawing_utils.draw_landmarks(
            image=annotated_image,
            landmark_list=pose_landmarks,
            connections=vision.PoseLandmarksConnections.POSE_LANDMARKS,
            landmark_drawing_spec=pose_landmark_style,
            connection_drawing_spec=pose_connection_style)
    return annotated_image

class ComeModeNode(Node):
    def __init__(self):
        super().__init__('come_mode_node')
        
        # 基础状态
        self.current_state = ComeModeState.IDLE
        self.come_triggered = False
        self.shoulder_mid_x = 0.0
        self.person_detected = False
        self.target_distance = 0.6
        self.person_distance = -1.0
        self.frame_timestamp = 0
        self.latest_frame = None
        self.bridge = CvBridge()

        # 窗口控制
        self.detection_enabled = False
        self.window_name = "Come Mode - Body Detection"
        self.window_open = False

        # 订阅
        self.create_subscription(String, '/gesture_cmd', self.gesture_cmd_cb, 10)
        self.create_subscription(String, '/car/control', self.voice_cmd_cb, 10)
        self.create_subscription(Float32, '/person_distance', self.distance_cb, 10)
        self.create_subscription(Image, '/camera_frame', self.frame_cb, 10)
        
        # 发布
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 模型初始化
        self.pose_model_path = "/home/yhs/mediapipe_models/pose_landmarker_lite.task"
        if not os.path.exists(self.pose_model_path):
            self.get_logger().error(f"模型不存在：{self.pose_model_path}")
            rclpy.shutdown()
            return
        
        self.pose_options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=self.pose_model_path),
            running_mode=VisionRunningMode.VIDEO,
            num_poses=1
        )
        self.pose_landmarker = PoseLandmarker.create_from_options(self.pose_options)
        
        # 主循环
        self.timer = self.create_timer(1/30, self.main_loop)
        
        self.get_logger().info("✅ Come模式节点启动（空闲：关闭检测/窗口）")

    # 回调函数
    def gesture_cmd_cb(self, msg):
        if msg.data.lower() == "come" and self.current_state == ComeModeState.IDLE:
            self.come_triggered = True

    def voice_cmd_cb(self, msg):
        if msg.data.lower() == "come" and self.current_state == ComeModeState.IDLE:
            self.come_triggered = True

    def distance_cb(self, msg):
        self.person_distance = msg.data

    def frame_cb(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            self.latest_frame = None

    # 人体检测 + 绘制
    def detect_and_draw(self, frame):
        if not self.detection_enabled:
            return frame
        
        frame = cv2.flip(frame, 1)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
        self.frame_timestamp += 1
        
        result = self.pose_landmarker.detect_for_video(mp_image, self.frame_timestamp)
        
        if result.pose_landmarks:
            # 绘制骨架
            annotated_rgb = draw_pose_landmarks_on_image(frame_rgb, result)
            out_frame = cv2.cvtColor(annotated_rgb, cv2.COLOR_RGB2BGR)
            
            # 计算肩中点
            lm = result.pose_landmarks[0]
            self.shoulder_mid_x = (lm[11].x + lm[12].x) / 2
            self.person_detected = True
            
            # 绘制红点
            h, w = frame.shape[:2]
            cx = int(self.shoulder_mid_x * w)
            cy = int((lm[11].y + lm[12].y) / 2 * h)
            cv2.circle(out_frame, (cx, cy), 8, (0, 0, 255), -1)
            return out_frame
        else:
            self.shoulder_mid_x = 0.0
            self.person_detected = False
            return frame

    # 主状态机
    def main_loop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        # ===================== 状态机 =====================
        if self.current_state == ComeModeState.IDLE:
            self.detection_enabled = False
            if self.come_triggered:
                self.detection_enabled = True
                self.current_state = ComeModeState.ROTATING
                self.come_triggered = False
                self.window_open = True
                self.get_logger().info("▶️ 开启检测 + 显示窗口")

        elif self.current_state == ComeModeState.ROTATING:
            self.detection_enabled = True
            if self.person_detected and 5/12 <= self.shoulder_mid_x <= 7/12:
                self.current_state = ComeModeState.MOVING
            else:
                twist.angular.z = 0.2

        elif self.current_state == ComeModeState.MOVING:
            self.detection_enabled = True
            if self.person_distance <= 0.0:
                twist.linear.x = 0.0
            elif self.person_distance <= self.target_distance:
                twist.linear.x = 0.0
                self.current_state = ComeModeState.STOP
            else:
                twist.linear.x = 0.15

        elif self.current_state == ComeModeState.STOP:
            self.detection_enabled = False
            self.window_open = False
            self.current_state = ComeModeState.IDLE
            self.get_logger().info("✅ 完成！关闭检测 + 窗口")

        # ===================== 绘制与显示（主线程，无报错） =====================
        display_frame = None
        if self.latest_frame is not None:
            display_frame = self.latest_frame.copy()
            if self.detection_enabled:
                display_frame = self.detect_and_draw(display_frame)

        # 显示窗口
        if self.window_open and display_frame is not None:
            # 绘制状态文字
            cv2.putText(display_frame, f"State: {self.current_state}", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(display_frame, f"Dis: {self.person_distance:.2f}m", (20, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.imshow(self.window_name, display_frame)
            cv2.waitKey(1)
        else:
            # 关闭窗口
            if self.window_open is False:
                try:
                    cv2.destroyWindow(self.window_name)
                except:
                    pass

        # IDLE 状态下不发布任何速度指令，停止占用 /cmd_vel，
        # 避免与 turn_to_person_node / follow_by_distance_node 产生指令冲突。
        if self.current_state == ComeModeState.IDLE:
            return

        # 发布速度
        self.vel_pub.publish(twist)

    def destroy_node(self):
        cv2.destroyAllWindows()
        self.pose_landmarker.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ComeModeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
