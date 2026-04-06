#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time  # 新增：用于时间戳和冷却判断

class DeviceControlNode(Node):
    def __init__(self):
        super().__init__('device_control_node')
        
        # ========== 基础配置 ==========
        # 1. 串口初始化
        self.ser_hc06 = None
        try:
            self.ser_hc06 = serial.Serial(
                port='/dev/rfcomm0',
                baudrate=9600,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            self.get_logger().info(f"✅ 成功连接HC-06，串口：{self.ser_hc06.port}")
        except Exception as e:
            self.get_logger().error(f"❌ HC-06初始化失败：{e}")
            self.get_logger().warn("请检查：1.串口路径 2.蓝牙配对 3.串口权限")
        
        # 2. 冷却机制配置（核心）
        self.cooldown_seconds = 3  # 指令间隔至少3秒
        self.last_cmd_timestamp = 0  # 最后一次执行指令的时间戳（初始为0）
        
        # ========== 话题订阅 ==========
        # 语音控制（垃圾桶+小车）
        self.trash_bin_sub = self.create_subscription(String, 'trash_bin/control', self.voice_trash_callback, 10)
        self.car_sub = self.create_subscription(String, 'car/control', self.voice_car_callback, 10)
        # 手势控制（垃圾桶+小车）
        self.gesture_sub = self.create_subscription(String, '/gesture_cmd', self.gesture_callback, 10)
        
        self.get_logger().info("📌 设备控制节点已启动（带3秒冷却+或逻辑）")
        self.get_logger().info(f"🔧 规则：任意指令执行后，{self.cooldown_seconds}秒内忽略所有指令")
        self.get_logger().info("🔧 支持指令：open/close（垃圾桶）、follow/come（小车）")

    # ========== 核心：冷却检查函数 ==========
    def is_cooldown_over(self):
        """检查冷却期是否结束：当前时间 - 最后执行时间 ≥ 3秒"""
        current_time = time.time()
        time_diff = current_time - self.last_cmd_timestamp
        if time_diff >= self.cooldown_seconds:
            return True
        else:
            self.get_logger().warn(f"⏳ 冷却中！需再等待 {self.cooldown_seconds - round(time_diff, 1)} 秒才能执行下一个指令")
            return False

    def send_to_hc06(self, cmd):
        """通用HC-06指令发送函数"""
        if self.ser_hc06 and self.ser_hc06.is_open:
            try:
                self.ser_hc06.write(str(cmd).encode('utf-8'))
                self.get_logger().info(f"📤 向HC-06发送指令：{cmd}")
            except Exception as e:
                self.get_logger().error(f"❌ 发送指令失败：{e}")
        else:
            self.get_logger().error("❌ HC-06串口未就绪，无法发送指令")

    # ========== 垃圾桶指令统一处理（带冷却） ==========
    def handle_trash_bin_cmd(self, cmd, source):
        """处理垃圾桶指令（open/close），先检查冷却"""
        # 第一步：检查冷却
        if not self.is_cooldown_over():
            return  # 冷却中，直接忽略
        
        # 第二步：处理指令
        self.get_logger().info(f"🔗 收到{source}指令：{cmd}（触发垃圾桶控制）")
        if cmd == "open":
            self.get_logger().info(f"🗑️ 执行动作：垃圾桶开盖（{source}触发）")
            self.send_to_hc06(1)
        elif cmd == "close":
            self.get_logger().info(f"🗑️ 执行动作：垃圾桶关盖（{source}触发）")
            self.send_to_hc06(2)
        
        # 第三步：更新最后执行时间戳（核心，启动冷却）
        self.last_cmd_timestamp = time.time()

    # ========== 小车指令统一处理（带冷却，暂不发指令） ==========
    def handle_car_cmd(self, cmd, source):
        """处理小车指令（follow/come），先检查冷却"""
        # 第一步：检查冷却
        if not self.is_cooldown_over():
            return  # 冷却中，直接忽略
        
        # 第二步：处理指令（仅日志）
        self.get_logger().info(f"🔗 收到{source}指令：{cmd}（触发小车控制）")
        if cmd == "follow":
            self.get_logger().info(f"🚗 执行动作：小车开始跟随（{source}触发）")
        elif cmd == "come":
            self.get_logger().info(f"🚗 执行动作：小车向我移动（{source}触发，暂不发送指令）")
        
        # 第三步：更新最后执行时间戳（核心，启动冷却）
        self.last_cmd_timestamp = time.time()

    # ========== 语音指令回调 ==========
    def voice_trash_callback(self, msg):
        """语音垃圾桶指令（open/close）"""
        self.handle_trash_bin_cmd(msg.data, "语音")

    def voice_car_callback(self, msg):
        """语音小车指令（follow/come）"""
        self.handle_car_cmd(msg.data, "语音")

    # ========== 手势指令回调 ==========
    def gesture_callback(self, msg):
        """手势指令：区分垃圾桶/小车指令"""
        cmd = msg.data
        if cmd in ["open", "close"]:
            self.handle_trash_bin_cmd(cmd, "手势")
        elif cmd in ["follow", "come"]:
            self.handle_car_cmd(cmd, "手势")

    def destroy_node(self):
        """节点销毁时强制发送2，再关闭串口"""
        # 无视冷却，强制发送2
        if self.ser_hc06 and self.ser_hc06.is_open:
            try:
                self.ser_hc06.write(b'2')  # 强制发送2
                self.get_logger().info("📤 程序关闭，强制向HC-06发送2（关闭垃圾桶）")
                time.sleep(0.1)  # 确保指令发送完成
            except Exception as e:
                self.get_logger().error(f"❌ 强制发送2失败：{e}")
        # 关闭串口
        if self.ser_hc06 and self.ser_hc06.is_open:
            self.ser_hc06.close()
            self.get_logger().info("🔌 HC-06串口已关闭")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DeviceControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 节点被用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
