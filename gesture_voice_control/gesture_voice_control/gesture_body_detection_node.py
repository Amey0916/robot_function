#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# 新增：导入图像消息类型和CV桥接工具
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import os
import time
import threading
import math
import sys


# -------------------------- 通用配置 --------------------------
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
DETECTION_FPS = 30  # 检测频率
CAMERA_FPS = 30     # 摄像头输出帧率
FOLLOW_DEBOUNCE_SECONDS = 2.0  # follow 手势防抖间隔（秒）
# 绿联相机关键词（可根据实际设备名调整）
UGREEN_CAM_KEYWORDS = ["ugreen", "Ugreen", "UGREEN", "绿联", "USB Camera"]
# 深度相机过滤关键词（重点新增）
DEPTH_CAM_KEYWORDS = ["depth", "Depth", "DEPTH", "ir", "IR", "realsense", "Realsense", "REALSONE"]

# -------------------------- 兼容OpenCV版本的工具函数 --------------------------
def get_cv_version():
    """获取OpenCV主版本号"""
    return int(cv2.__version__.split('.')[0])

def auto_find_ugreen_camera():
    """
    自动查找绿联相机设备（过滤深度相机，优先匹配绿联特征）
    返回：正确的设备路径（如/dev/video4）或None
    """
    # 方法1：遍历/dev/video*设备（Linux）- 优先推荐，精准度高
    if sys.platform.startswith('linux'):
        for i in range(20):  # 扩大遍历范围到20个设备
            dev_path = f"/dev/video{i}"
            if not os.path.exists(dev_path):
                continue
            
            try:
                # 读取设备名称（关键：过滤深度相机）
                info_path = f"/sys/class/video4linux/video{i}/name"
                if os.path.exists(info_path):
                    with open(info_path, 'r') as f:
                        dev_name = f.read().strip()
                    
                    # 第一步：过滤深度相机
                    if any(kw in dev_name for kw in DEPTH_CAM_KEYWORDS):
                        print(f"跳过深度相机：{dev_path} (名称: {dev_name})")
                        continue
                    
                    # 第二步：匹配绿联相机
                    if any(keyword in dev_name for keyword in UGREEN_CAM_KEYWORDS):
                        # 额外验证：打开并读取一帧RGB画面
                        cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
                        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # 强制MJPG格式
                        ret, frame = cap.read()
                        cap.release()
                        if ret and frame is not None and len(frame.shape) == 3:
                            print(f"✅ 找到绿联相机设备：{dev_path} (名称: {dev_name})")
                            return dev_path
            except Exception as e:
                print(f"检查设备{dev_path}失败：{e}")
                continue
    
    # 方法2：跨平台遍历（备用）
    print("Linux设备名称匹配失败，尝试跨平台模式...")
    for i in range(20):
        try:
            cap = cv2.VideoCapture(i, cv2.CAP_V4L2 if get_cv_version() >= 3 else cv2.CAP_ANY)
            if not cap.isOpened():
                cap.release()
                continue
            
            # 强制设置为RGB格式（关键：避免深度流）
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
            
            # 读取多帧以跳过缓存的深度数据
            for _ in range(3):
                ret, frame = cap.read()
            
            cap.release()
            if ret and frame is not None and len(frame.shape) == 3:
                # 验证不是深度帧（深度帧通常是单通道）
                dev_path = f"/dev/video{i}" if sys.platform.startswith('linux') else i
                print(f"✅ 跨平台模式找到有效RGB相机：{dev_path}")
                return dev_path
        except Exception as e:
            continue
    
    return None

# -------------------------- MediaPipe 新版API导入 --------------------------
# 手部检测相关
BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
# 人体姿态检测相关
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
# 通用
VisionRunningMode = mp.tasks.vision.RunningMode
drawing_utils = mp.tasks.vision.drawing_utils
drawing_styles = mp.tasks.vision.drawing_styles
vision = mp.tasks.vision

# -------------------------- 手部骨架连接（复用你的逻辑） --------------------------
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),            # 拇指
    (0, 5), (5, 6), (6, 7), (7, 8),            # 食指
    (0, 9), (9, 10), (10, 11), (11, 12),       # 中指
    (0, 13), (13, 14), (14, 15), (15, 16),     # 无名指
    (0, 17), (17, 18), (18, 19), (19, 20),     # 小指
    (5, 9), (9, 13), (13, 17)                  # 掌心横向连接
]      

# -------------------------- 人体检测绘制函数（复用官方逻辑） --------------------------
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

# -------------------------- 肩中点计算函数 --------------------------
def get_shoulder_center(pose_landmarks, width, height):
    LEFT_SHOULDER = vision.PoseLandmark.LEFT_SHOULDER
    RIGHT_SHOULDER = vision.PoseLandmark.RIGHT_SHOULDER
    
    l_shoulder = pose_landmarks[LEFT_SHOULDER]
    r_shoulder = pose_landmarks[RIGHT_SHOULDER]
    
    l_x = l_shoulder.x * width
    l_y = l_shoulder.y * height
    r_x = r_shoulder.x * width
    r_y = r_shoulder.y * height
    
    center_x = (l_x + r_x) / 2
    center_y = (l_y + r_y) / 2
    
    return center_x, center_y

# -------------------------- 合并后的核心节点 --------------------------
class GestureBodyDetectionNode(Node):
    def __init__(self):
        super().__init__('gesture_body_detection_node')
        
        # 1. 发布手势指令话题
        self.gesture_pub = self.create_publisher(String, '/gesture_cmd', 10)
        
        # 2. 发布方向话题（left/center/right）
        self.orientation_pub = self.create_publisher(String, '/cmd_orientation', 10)
        
        # 新增：发布摄像头帧话题（供come_mode_node订阅）
        self.frame_pub = self.create_publisher(Image, '/camera_frame', 10)
        self.bridge = CvBridge()  # CV桥接工具（转换OpenCV帧到ROS2图像消息）
        
        # 3. 核心状态变量
        self.is_follow_mode = False  # 是否启动人体检测
        self.last_gesture = None     # 上一次识别的手势
        self.frame_timestamp = 0     # MediaPipe Video模式时间戳
        self.follow_gesture_count = 0  # follow手势计数（奇数开，偶数关）
        self.last_follow_time = 0.0    # 上一次触发 follow 的时间
        
        # 4. 自动查找并初始化绿联摄像头（核心修改）
        self.cam_device = auto_find_ugreen_camera()
        if self.cam_device is None:
            self.get_logger().fatal("未找到绿联相机设备！请检查设备连接")
            rclpy.shutdown()
            return
        
        # 兼容不同OpenCV版本的摄像头打开方式
        self.cap = None
        cv_version = get_cv_version()
        try:
            if cv_version >= 4:
                # OpenCV4+ 推荐用法：强制V4L2并设置RGB格式
                self.cap = cv2.VideoCapture(self.cam_device, cv2.CAP_V4L2)
                # 关键设置：强制MJPG编码（避免YUV格式问题）、RGB转换
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
            else:
                # OpenCV3及以下兼容用法
                if isinstance(self.cam_device, str) and self.cam_device.startswith('/dev/'):
                    self.cap = cv2.VideoCapture(int(self.cam_device.split('/')[-1]))
                else:
                    self.cap = cv2.VideoCapture(self.cam_device)
        except Exception as e:
            # 终极兼容方案
            self.get_logger().warn(f"指定模式打开摄像头失败：{e}，尝试通用模式")
            self.cap = cv2.VideoCapture(self.cam_device)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
        
        # 设置摄像头参数（兼容所有OpenCV版本）
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        # 兼容不同OpenCV版本的缓冲区设置（关键：减少延迟和缓存）
        if cv_version >= 3:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # 额外设置：禁用自动曝光/自动白平衡（避免画面异常）
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25=手动模式
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 100)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4500)
        
        if not self.cap.isOpened():
            self.get_logger().fatal(f"无法打开绿联摄像头（设备：{self.cam_device}）！")
            rclpy.shutdown()
            return
        self.get_logger().info(
            f"绿联摄像头初始化成功：设备{self.cam_device}，分辨率{CAMERA_WIDTH}×{CAMERA_HEIGHT}，帧率{CAMERA_FPS}"
        )
        
        # 5. 初始化手部检测器（复用你的模型路径和参数）
        self.hand_model_path = "/home/yhs/mediapipe_models/hand_landmarker.task"
        if not os.path.exists(self.hand_model_path):
            self.get_logger().fatal(f"手部模型文件不存在：{self.hand_model_path}")
            self.release_resources()
            rclpy.shutdown()
            return
        
        hand_options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=self.hand_model_path),
            running_mode=VisionRunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.7,
            min_hand_presence_confidence=0.7,
            min_tracking_confidence=0.7
        )
        self.hand_detector = HandLandmarker.create_from_options(hand_options)
        self.get_logger().info("手部检测器初始化成功！")
        
        # 6. 初始化人体姿态检测器（复用你的模型路径）
        self.pose_model_path = "/home/yhs/mediapipe_models/pose_landmarker_lite.task"
        if not os.path.exists(self.pose_model_path):
            self.get_logger().warn(f"人体模型文件不存在：{self.pose_model_path}")
            self.get_logger().info("正在自动下载人体检测模型...")
            os.makedirs(os.path.dirname(self.pose_model_path), exist_ok=True)
            # 下载官方模型
            import urllib.request
            try:
                urllib.request.urlretrieve(
                    "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task",
                    self.pose_model_path
                )
                self.get_logger().info("人体模型下载成功！")
            except Exception as e:
                self.get_logger().fatal(f"模型下载失败：{e}")
                self.release_resources()
                rclpy.shutdown()
                return
        
        pose_options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=self.pose_model_path),
            running_mode=VisionRunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.7,
            min_pose_presence_confidence=0.7,
            min_tracking_confidence=0.7,
            output_segmentation_masks=False
        )
        self.pose_detector = PoseLandmarker.create_from_options(pose_options)
        self.get_logger().info("人体检测器初始化成功！")
        
        # 7. 线程共享数据
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_display_frame = None
        self.stop_event = threading.Event()
        
        # 8. 启动采集线程与显示线程
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.capture_thread.start()
        self.display_thread.start()
        
        # 9. 创建定时器（30Hz检测）
        self.timer = self.create_timer(1/DETECTION_FPS, self.detect_loop)
        self.get_logger().info("手势+人体检测节点启动成功！")

        # 新增：订阅语音follow模式控制话题
        self.voice_follow_sub = self.create_subscription(
            String,
            '/voice_follow_control',
            self.voice_follow_callback,
            10
        )
        
        self.get_logger().info("手势+人体检测节点启动成功！")
        self.get_logger().info("🗣️ 支持语音指令：跟随（开启follow）、停止跟随（关闭follow）")
        self.get_logger().info("📸 已发布摄像头帧话题：/camera_frame（供come_mode_node订阅）")

    def _publish_stop_orientation(self):
        """退出follow模式时立刻发布一次 none，清除残留的 center/left/right 指令"""
        stop_msg = String()
        stop_msg.data = "none"
        self.orientation_pub.publish(stop_msg)

    # 新增：语音follow模式回调函数
    def voice_follow_callback(self, msg):
        """处理语音的follow模式控制指令"""
        cmd = msg.data
        now = time.monotonic()
        # 防抖：和手势follow共用同一个时间戳，避免频繁切换
        if now - self.last_follow_time >= FOLLOW_DEBOUNCE_SECONDS:
            if cmd == "start_follow":
                self.is_follow_mode = True
                self.follow_gesture_count = 1  # 同步计数，确保手势切换逻辑正常
                self.last_follow_time = now
                self.get_logger().info("🗣️ 语音指令触发：进入follow模式，启动人体检测！")
            elif cmd == "stop_follow":
                self.is_follow_mode = False
                self.follow_gesture_count = 0  # 同步计数，确保手势切换逻辑正常
                self.last_follow_time = now
                self.get_logger().info("🗣️ 语音指令触发：退出follow模式，停止人体检测！")
                self._publish_stop_orientation()
            else:
                self.get_logger().warn(f"🗣️ 未知的follow模式指令：{cmd}")
        else:
            self.get_logger().warn(f"🗣️ 语音follow指令防抖中！需再等待 {FOLLOW_DEBOUNCE_SECONDS - round(now - self.last_follow_time, 1)} 秒")
            
    def capture_loop(self):
        """摄像头采集线程：持续读取最新帧（跳过无效帧）"""
        skip_frames = 0  # 跳过前几帧缓存
        while not self.stop_event.is_set():
            ret, frame = self.cap.read()
            if not ret or frame is None:
                continue
            
            # 跳过前3帧缓存（避免深度相机残留帧）
            if skip_frames < 3:
                skip_frames += 1
                continue
            
            # 验证是RGB帧（深度帧是单通道）
            if len(frame.shape) != 3:
                continue
            
            with self.frame_lock:
                self.latest_frame = frame
                # 新增：发布原始摄像头帧（未处理的帧）
                try:
                    frame_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    self.frame_pub.publish(frame_msg)
                except Exception as e:
                    self.get_logger().warn(f"发布帧失败：{e}")

    def display_loop(self):
        """显示线程：持续显示最新画面，避免阻塞推理"""
        while not self.stop_event.is_set():
            with self.frame_lock:
                frame = None
                if self.latest_display_frame is not None:
                    frame = self.latest_display_frame.copy()
                elif self.latest_frame is not None:
                    frame = self.latest_frame.copy()
            if frame is not None:
                cv2.imshow("Gesture + Body Detection (Q to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop_event.set()
                rclpy.shutdown()
                break

    def detect_loop(self):
        """核心检测循环：基于最新帧推理，不阻塞采集与显示"""
        with self.frame_lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame.copy()
        
        # 真实帧尺寸（姿态/手势都使用）
        h, w = frame.shape[:2]
        
        # 预处理：镜像翻转+RGB转换
        frame = cv2.flip(frame, 1)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
        
        # 更新时间戳（真实毫秒）
        self.frame_timestamp = int(time.monotonic() * 1000)
        
        # -------------------------- 第一步：手势识别（始终执行） --------------------------
        current_gesture = None
        hand_detection_result = self.hand_detector.detect_for_video(mp_image, self.frame_timestamp)
        
        if hand_detection_result.hand_landmarks:
            for hand_landmarks in hand_detection_result.hand_landmarks:
                # 绘制手部关键点和骨架（使用真实帧尺寸）
                points = [(int(lm.x * w), int(lm.y * h)) for lm in hand_landmarks]
                for x, y in points:
                    cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
                for a, b in HAND_CONNECTIONS:
                    x1, y1 = points[a]
                    x2, y2 = points[b]
                    cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                
                # 手势识别逻辑
                wrist = hand_landmarks[0]
                thumb_tip = hand_landmarks[4]
                index_tip = hand_landmarks[8]
                middle_tip = hand_landmarks[12]
                ring_tip = hand_landmarks[16]
                pinky_tip = hand_landmarks[20]

                thumb_ip = hand_landmarks[3]      
                index_pip = hand_landmarks[6]     
                middle_pip = hand_landmarks[10]
                ring_pip = hand_landmarks[14]
                pinky_pip = hand_landmarks[18]

                index_mcp = hand_landmarks[5]
                middle_mcp = hand_landmarks[9]
                ring_mcp = hand_landmarks[13]
                pinky_mcp = hand_landmarks[17]
                
                # 手指伸直判断
                index_straight = index_tip.y < index_pip.y
                middle_straight = middle_tip.y < middle_pip.y
                ring_straight = ring_tip.y < ring_pip.y
                pinky_straight = pinky_tip.y < pinky_pip.y
                thumb_straight = thumb_tip.y < thumb_ip.y

                # 手指弯曲判断
                ring_bent = ring_tip.y> ring_pip.y
                pinky_bent = pinky_tip.y > pinky_pip.y
                
                # “open”和“close”手势
                thumb_dist = abs(thumb_tip.y - wrist.y)
                index_dist = abs(index_tip.y - wrist.y)
                middle_dist = abs(middle_tip.y - wrist.y)
                ring_dist = abs(ring_tip.y - wrist.y)
                pinky_dist = abs(pinky_tip.y - wrist.y)

                # 计算各比值（避免除零错误）
                middle_dist_nonzero = middle_dist if middle_dist != 0 else 1e-6
                index_ratio = index_dist / middle_dist_nonzero
                ring_ratio = ring_dist / middle_dist_nonzero
                pinky_ratio = pinky_dist / middle_dist_nonzero
                thumb_ratio = thumb_dist / middle_dist_nonzero

                # 人体标准比例区间（宽松版，适配不同人手 + 远距离）
                index_ok = 0.80 < index_ratio < 1.50   # 食指/中指
                ring_ok  = 0.78 < ring_ratio  < 1.10   # 无名指/中指
                pinky_ok= 0.48 < pinky_ratio < 0.80    # 小指/中指
                thumb_ok= 0.29 < thumb_ratio < 0.73    # 拇指/中指

                # 最终 OPEN 判定（五指伸直 + 比例正常）
                is_palm_open = (
                    index_straight and middle_straight and ring_straight and pinky_straight and thumb_straight
                    and index_ok and ring_ok and pinky_ok and thumb_ok
                )

                def calculate_finger_angle(mcp1, tip1, mcp2, tip2):
                    """
                    计算 两根手指 之间的夹角
                    mcp1: 第一根手指根部 (如 index_mcp)
                    tip1: 第一根手指指尖 (如 index_tip)
                    mcp2: 第二根手指根部 (如 middle_mcp)
                    tip2: 第二根手指指尖 (如 middle_tip)
    
                    返回：两根手指的夹角（0~180度）
                    """
                    # 向量1：mcp1 → tip1
                    v1x = tip1.x - mcp1.x
                    v1y = tip1.y - mcp1.y
    
                    # 向量2：mcp2 → tip2
                    v2x = tip2.x - mcp2.x
                    v2y = tip2.y - mcp2.y
    
                    # 点积
                    dot = v1x * v2x + v1y * v2y
    
                    # 模长
                    mag1 = math.hypot(v1x, v1y)
                    mag2 = math.hypot(v2x, v2y)
    
                    if mag1 == 0 or mag2 == 0:
                        return 0.0
    
                    # 余弦值 → 角度
                    cos_theta = max(min(dot / (mag1 * mag2), 1.0), -1.0)
                    angle = math.degrees(math.acos(cos_theta))
    
                    return angle

                index_middle_angle = calculate_finger_angle(index_mcp, index_tip, middle_mcp, middle_tip)
                middle_pinky_angle = calculate_finger_angle(middle_mcp, middle_tip, pinky_mcp, pinky_tip)

                thumb_index_dist = np.sqrt((thumb_tip.x - index_tip.x)**2 + (thumb_tip.y - index_tip.y)**2)

                if is_palm_open:
                    current_gesture = "open"
                elif (index_pip.y > index_mcp.y and middle_pip.y > middle_mcp.y and 
                      ring_pip.y > ring_mcp.y and pinky_pip.y >pinky_mcp.y and thumb_dist < 0.15):
                    current_gesture = "close"
                else:
                    # 数字二: come手势
                    if index_straight and middle_straight and ring_bent and pinky_bent and index_ok and 20 < index_middle_angle < 60:
                        current_gesture = "come"
                    # OK手势: follow手势
                    elif (thumb_index_dist/middle_dist) < 0.10 and middle_straight and ring_straight and pinky_straight and ring_ok and 10 < middle_pinky_angle < 70:
                        current_gesture = "follow"
        
        # 发布手势指令（仅变化时发布）
        if current_gesture and current_gesture != self.last_gesture:
            msg = String()
            msg.data = current_gesture
            self.gesture_pub.publish(msg)
            self.get_logger().info(f"发布手势指令：{current_gesture}")
            self.last_gesture = current_gesture
        
        # follow 手势触发开/关逻辑（与 last_gesture 无关），并加 2 秒防抖
        if current_gesture == "follow":
            now = time.monotonic()
            if now - self.last_follow_time >= FOLLOW_DEBOUNCE_SECONDS:
                self.last_follow_time = now
                self.follow_gesture_count += 1
                if self.follow_gesture_count % 2 == 1:
                    self.is_follow_mode = True
                    self.get_logger().info("进入follow模式，启动人体检测！")
                else:
                    self.is_follow_mode = False
                    self.get_logger().info("退出follow模式，停止人体检测！")
                    self._publish_stop_orientation()

        # -------------------------- 第二步：人体检测（仅follow模式执行） --------------------------
        if self.is_follow_mode:
            pose_detection_result = self.pose_detector.detect_for_video(mp_image, self.frame_timestamp)
            pose_annotated_rgb = draw_pose_landmarks_on_image(frame_rgb, pose_detection_result)
            frame = cv2.cvtColor(pose_annotated_rgb, cv2.COLOR_RGB2BGR)
            
            if pose_detection_result.pose_landmarks:
                pose_landmarks = pose_detection_result.pose_landmarks[0]
                shoulder_center_x, shoulder_center_y = get_shoulder_center(
                    pose_landmarks, w, h
                )

                if shoulder_center_x < w * 5 / 12:
                    direction = "right"
                elif shoulder_center_x > w * 7 / 12:
                    direction = "left"
                else:
                    direction = "center"

                dir_msg = String()
                dir_msg.data = direction
                self.orientation_pub.publish(dir_msg)

                cv2.circle(frame, (int(shoulder_center_x), int(shoulder_center_y)), 
                           8, (0, 0, 255), -1)
                cv2.putText(frame, 
                           f"Direction: {direction}",
                           (20, 80), cv2.FONT_HERSHEY_SIMPLEX,
                           1.0, (255, 0, 0), 2, cv2.LINE_AA)
            
            cv2.putText(frame, "Mode: follow (body detection)",
                        (20, 120), cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (0, 255, 255), 2, cv2.LINE_AA)

        # 绘制当前手势
        cv2.putText(frame, f"Current Gesture: {current_gesture or 'None'}",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        with self.frame_lock:
            self.latest_display_frame = frame

    def release_resources(self):
        """统一释放所有资源（避免报错）"""
        self.stop_event.set()
        if hasattr(self, 'capture_thread') and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
        if hasattr(self, 'display_thread') and self.display_thread.is_alive():
            self.display_thread.join(timeout=1.0)
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        if hasattr(self, 'hand_detector'):
            self.hand_detector.close()
        if hasattr(self, 'pose_detector'):
            self.pose_detector.close()
        cv2.destroyAllWindows()
        self.get_logger().info("资源已释放")

    def destroy_node(self):
        """节点销毁时释放资源"""
        self.release_resources()
        super().destroy_node()

# -------------------------- 主函数 --------------------------
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GestureBodyDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("用户终止节点")
    except Exception as e:
        if node:
            node.get_logger().fatal(f"节点运行出错：{e}")
    finally:
        if node:
            node.destroy_node()
        # 避免重复shutdown报错
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
