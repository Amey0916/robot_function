#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import mediapipe as mp
import numpy as np
import os

# MediaPipe新版API
BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

# 手部骨架连线关系
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
    (5, 9), (9, 13), (13, 17)
]      

class GestureLocalNode(Node):
    def __init__(self):
        super().__init__('gesture_local_node')
        self.pub = self.create_publisher(String, '/cmd_orientation', 10)
        
        self.model_path = "/home/yhs/mediapipe_models/hand_landmarker.task"
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"模型文件不存在：{self.model_path}")
            rclpy.shutdown()
            return

        self.options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=self.model_path),
            running_mode=VisionRunningMode.VIDEO,
            num_hands=1
        )
        self.landmarker = HandLandmarker.create_from_options(self.options)

        # 只用摄像头1
        self.cap = cv2.VideoCapture(1)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头1！")
            rclpy.shutdown()
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.timer = self.create_timer(1/30, self.detect_which_side)
        self.frame_timestamp = 0
        self.last_direction = None
        self.get_logger().info("手势分区检测节点已启动（仅用摄像头1）")

    def detect_which_side(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("无法读取摄像头1帧")
            return

        direction = "none"
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
        self.frame_timestamp += 1
        detection_result = self.landmarker.detect_for_video(mp_image, self.frame_timestamp)

        h, w, _ = frame.shape
        if detection_result.hand_landmarks:
            for hand_landmarks in detection_result.hand_landmarks:
                # 用“食指指尖”作为参考点
                index_tip = hand_landmarks[8]
                x = int(index_tip.x * w)
                # 分区判断
                if x < w // 3:
                    direction = "left"
                elif x > 2 * w // 3:
                    direction = "right"
                else:
                    direction = "center"

                # 可选：可画骨架
                points = [(int(lm.x * w), int(lm.y * h)) for lm in hand_landmarks]
                for a, b in HAND_CONNECTIONS:
                    x1, y1 = points[a]
                    x2, y2 = points[b]
                    cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                for xpt, ypt in points:
                    cv2.circle(frame, (xpt, ypt), 5, (0, 255, 0), -1)

        # 只有识别结果变化时才发布
        if direction != self.last_direction:
            msg = String()
            msg.data = direction
            self.pub.publish(msg)
            self.get_logger().info(f"发送cmd_orientation: {direction}")
            self.last_direction = direction

        cv2.putText(frame, f"Direction: {direction}", (20, 40), 1, 2, (0, 0, 255), 2)
        cv2.imshow("Gesture Local Side", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        self.landmarker.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GestureLocalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()