#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import time

BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode
drawing_utils = mp.tasks.vision.drawing_utils
drawing_styles = mp.tasks.vision.drawing_styles
vision = mp.tasks.vision

# 停车保持帧数：进入 STOP 阶段后连续发多帧零速，防止惯性冲过目标
_STOP_HOLD_FRAMES = 10

# ✅【修复 1】补上缺失的状态类！
class ComeModeState:
    IDLE = 0
    ROTATING = 1
    ROTATING_STABILIZING = 2
    MOVING = 3
    STOP = 4

# ------------------- 关键参数 -------------------
CENTER_CONFIRM_FRAMES = 3
FULL_ROTATION_SPEED = 0.18
SLOW_ROTATION_SPEED = 0.06
KP_ROTATION = 0.7
MIN_ROTATION_SPEED = 0.07
SOFT_LANDING_OFFSET_THRESHOLD = 0.015
STABILIZE_OFFSET_THRESHOLD = 0.008

RELEASE_TORQUE_SPEED = 0.03
RELEASE_TIME = 0.3
STABILIZE_DELAY = 1.5

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

        self.face_detected = False
        self.follow_mode_active = False
        self._come_mode_was_active = False

        self.release_start_time = 0.0
        self.releasing_torque = False
        self.stabilize_start_time = 0.0
        self.stop_hold_count = 0

        self.detection_enabled = False
        self.window_name = "Come Mode"
        self.window_open = False

        self.create_subscription(String, '/gesture_cmd', self.gesture_cb, 10)
        self.create_subscription(String, '/car/control', self.voice_cb, 10)
        self.create_subscription(Float32, '/person_distance', self.distance_cb, 10)
        self.create_subscription(Image, '/camera_frame', self.frame_cb, 10)
        self.create_subscription(Bool, '/follow_mode_active', self.follow_mode_cb, 10)

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.come_mode_pub = self.create_publisher(Bool, '/come_mode_active', 10)

        self.pose_model_path = "/home/yhs/mediapipe_models/pose_landmarker_lite.task"
        self.pose_options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=self.pose_model_path),
            running_mode=VisionRunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.7,
            min_pose_presence_confidence=0.7,
            min_tracking_confidence=0.7)
        self.pose_landmarker = PoseLandmarker.create_from_options(self.pose_options)
        self.timer = self.create_timer(1/30, self.main_loop)

    def gesture_cb(self, msg):
        if msg.data == "come" and self.current_state == ComeModeState.IDLE:
            if self.follow_mode_active:
                self.get_logger().warn("🚫 互斥拒绝：当前处于 follow 模式，无法激活 come 模式！")
                return
            self.come_triggered = True

    def voice_cb(self, msg):
        if msg.data == "come" and self.current_state == ComeModeState.IDLE:
            if self.follow_mode_active:
                self.get_logger().warn("🚫 互斥拒绝：当前处于 follow 模式，无法激活 come 模式！")
                return
            self.come_triggered = True

    def follow_mode_cb(self, msg):
        was_inactive = not self.follow_mode_active
        self.follow_mode_active = msg.data
        if self.follow_mode_active and was_inactive and self.current_state != ComeModeState.IDLE:
            self.get_logger().warn("🚫 follow 模式激活，强制退出 come 模式！")
            self._stop_come_mode()

    def _stop_come_mode(self):
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.vel_pub.publish(stop_twist)
        self.current_state = ComeModeState.IDLE
        self.detection_enabled = False
        self.window_open = False
        self.releasing_torque = False
        self.center_confirm_count = 0
        self.get_logger().info("🛑 come 模式已强制停止，/cmd_vel 控制权归还 follow 节点。")

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

            self.shoulder_mid_x = (lm[11].x + lm[12].x) / 2
            self.person_detected = True

            face_visible = (
                lm[0].visibility > 0.70 or
                lm[2].visibility > 0.70 or
                lm[5].visibility > 0.70
            )
            self.face_detected = face_visible

            h, w = frame.shape[:2]
            cx = int(self.shoulder_mid_x * w)
            cy = int((lm[11].y + lm[12].y) / 2 * h)
            cv2.circle(out, (cx, cy), 8, (0, 0, 255), -1)
            return out
        else:
            self.person_detected = False
            self.face_detected = False
            return frame

    def main_loop(self):
        if self.follow_mode_active:
            if self.current_state != ComeModeState.IDLE:
                self._stop_come_mode()
            return

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        if self.current_state == ComeModeState.IDLE:
            if self.come_triggered:
                self.current_state = ComeModeState.ROTATING
                self.detection_enabled = True
                self.come_triggered = False
                self.window_open = True
                self.center_confirm_count = 0
                self.releasing_torque = False

        elif self.current_state == ComeModeState.ROTATING:
            if self.person_detected and self.face_detected:
                offset = self.shoulder_mid_x - 0.5
                if 5/12 <= self.shoulder_mid_x <= 7/12:
                    self.center_confirm_count += 1
                    if abs(offset) > SOFT_LANDING_OFFSET_THRESHOLD:
                        twist.angular.z = SLOW_ROTATION_SPEED if offset > 0 else -SLOW_ROTATION_SPEED
                    else:
                        twist.angular.z = 0.0
                    if self.center_confirm_count >= CENTER_CONFIRM_FRAMES:
                        self.current_state = ComeModeState.ROTATING_STABILIZING
                        self.releasing_torque = True
                        self.release_start_time = time.time()
                        self.center_confirm_count = 0
                else:
                    self.center_confirm_count = 0
                    speed = KP_ROTATION * abs(offset)
                    speed = max(speed, MIN_ROTATION_SPEED)
                    speed = min(speed, FULL_ROTATION_SPEED)
                    twist.angular.z = speed if offset > 0 else -speed
            else:
                self.center_confirm_count = 0
                twist.angular.z = FULL_ROTATION_SPEED

        elif self.current_state == ComeModeState.ROTATING_STABILIZING:
            if not self.person_detected:
                self.current_state = ComeModeState.ROTATING
                return

            offset = self.shoulder_mid_x - 0.5
            if self.releasing_torque:
                if time.time() - self.release_start_time < RELEASE_TIME:
                    twist.angular.z = RELEASE_TORQUE_SPEED if offset > 0 else -RELEASE_TORQUE_SPEED
                else:
                    self.releasing_torque = False
                    self.stabilize_start_time = time.time()
            else:
                twist.angular.z = 0.0
                if time.time() - self.stabilize_start_time >= STABILIZE_DELAY:
                    self.current_state = ComeModeState.MOVING

        elif self.current_state == ComeModeState.MOVING:
            if not self.person_detected:
                self.current_state = ComeModeState.ROTATING
            elif self.person_distance > self.target_distance:
                twist.linear.x = 0.13
            else:
                twist.linear.x = 0.0
                self.stop_hold_count = _STOP_HOLD_FRAMES
                self.current_state = ComeModeState.STOP

        elif self.current_state == ComeModeState.STOP:
            twist.linear.x = 0.0
            if self.stop_hold_count > 0:
                self.stop_hold_count -= 1
            else:
                self.detection_enabled = False
                self.window_open = False
                self.current_state = ComeModeState.IDLE

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

        come_active_now = (self.current_state != ComeModeState.IDLE)
        if come_active_now != self._come_mode_was_active:
            status_msg = Bool()
            status_msg.data = come_active_now
            self.come_mode_pub.publish(status_msg)
            self._come_mode_was_active = come_active_now
            self.get_logger().info(f"📢 come 模式状态变更：{'激活' if come_active_now else '已退出'}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        self.pose_landmarker.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ComeModeNode()
    try:
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
