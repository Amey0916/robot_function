#!/usr/bin/env python3
import asyncio
import json
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import websockets


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

        # 2. 冷却机制配置（核心）- 已修��为 2 秒
        self.cooldown_seconds = 2
        self.last_cmd_timestamp = 0

        # 🟢 新增：记录当前小车是否处于跟随状态
        self.is_following = False

        # 3. WebSocket 上报配置（发给 web_chassis_control）
        self.bridge_ws_url = 'ws://127.0.0.1:8899'

        # ========== 话题订阅 ==========
        self.trash_bin_sub = self.create_subscription(String, 'trash_bin/control', self.voice_trash_callback, 10)
        self.car_sub = self.create_subscription(String, 'car/control', self.voice_car_callback, 10)
        self.gesture_sub = self.create_subscription(String, '/gesture_cmd', self.gesture_callback, 10)

        self.get_logger().info("📌 设备控制节点已启动（带2秒冷却+或逻辑）")
        self.get_logger().info(f"🔧 规则：任意指令执行后，{self.cooldown_seconds}秒内忽略所有指令")
        self.get_logger().info("🔧 支持指令：open/close/come/follow/stop_follow")

    def is_cooldown_over(self):
        current_time = time.time()
        time_diff = current_time - self.last_cmd_timestamp
        if time_diff >= self.cooldown_seconds:
            return True
        self.get_logger().warn(f"⏳ 冷却中！需再等待 {self.cooldown_seconds - round(time_diff, 1)} 秒才能执行下一个指令")
        return False

    def send_to_hc06(self, cmd):
        if self.ser_hc06 and self.ser_hc06.is_open:
            try:
                # 真正发射字符给 HC-06 蓝牙串口
                self.ser_hc06.write(str(cmd).encode('utf-8'))
                self.get_logger().info(f"📤 向HC-06发送指令：{cmd}")
            except Exception as e:
                self.get_logger().error(f"❌ 发送指令失败：{e}")
        else:
            self.get_logger().error("❌ HC-06串口未就绪，无法发送指令")

    def _normalize_cmd(self, cmd_raw: str) -> str:
        cmd = (cmd_raw or '').strip().lower()
        mapping = {
            'open': 'open',
            'close': 'close',
            'come': 'come',
            'follow': 'follow',
            'stop_follow': 'stop_follow',
            'stop follow': 'stop_follow',
            'stop': 'stop_follow',
        }
        return mapping.get(cmd, '')

    def _report_recognition(self, source: str, command: str) -> None:
        payload = {
            'type': 'recognition',
            'payload': {
                'source': source,      # voice | gesture
                'command': command,    # open | close | come | follow | stop_follow
                'timestamp': int(time.time() * 1000),
            }
        }
        threading.Thread(target=self._send_ws_message, args=(payload,), daemon=True).start()

    def _send_ws_message(self, payload: dict) -> None:
        async def _send_once():
            async with websockets.connect(self.bridge_ws_url, open_timeout=1, close_timeout=1, ping_interval=None) as ws:
                await ws.send(json.dumps(payload, ensure_ascii=False))

        try:
            asyncio.run(_send_once())
        except Exception as e:
            self.get_logger().warn(f"⚠️ 指令状态上报失败：{e}")

    def handle_trash_bin_cmd(self, cmd: str, source: str):
        # 1. 优先判断冷却时间，冷却中的冗余手势直接丢弃
        if not self.is_cooldown_over():
            return

        # 2. 确认通过冷却限制后，再上���给小程序端显示
        self._report_recognition(source, cmd)

        self.get_logger().info(f"🔗 收到{source}指令：{cmd}（触发垃圾桶控制）")
        if cmd == 'open':
            self.get_logger().info(f"🗑️ 执行动作：垃圾桶开盖（{source}触发）")
            self.send_to_hc06(1)
        elif cmd == 'close':
            self.get_logger().info(f"🗑️ 执行动作：垃圾桶关盖（{source}触发）")
            self.send_to_hc06(2)

        self.last_cmd_timestamp = time.time()

    def handle_car_cmd(self, cmd: str, source: str):
        # 🟢 核心修改：手势翻转逻辑 (Toggle)
        # 如果是手势发来的 follow，我们需要根据当前状态决定是“开始”还是“停止”
        if source == 'gesture' and cmd == 'follow':
            if self.is_following:
                cmd = 'stop_follow'  # 当前在跟随，再收到手势就是停止
            else:
                cmd = 'follow'       # 当前没跟随，收到手势就是开始

        # 1. 优先判断冷却时间，冷却中的冗余手势直接丢弃
        if not self.is_cooldown_over():
            return

        # 2. 确认通过冷却限制后，再上报给小程序端显示
        self._report_recognition(source, cmd)

        self.get_logger().info(f"🔗 收到{source}指令：{cmd}（触发小车控制）")
        
        # 🟢 执行动作，并同步更新状态标志位
        if cmd == 'follow':
            self.is_following = True   # 状态同步：进入跟随
            self.get_logger().info(f"🚗 执行动作：小车开始跟随（{source}触发）")
            # 可以在这里添加下发给底盘的具体控制代码，如 self.send_to_hc06(...) 等
            
        elif cmd == 'come':
            self.is_following = False  # 状态同步：执行其他动作打断跟随
            self.get_logger().info(f"🚗 执行动作：小车向我移动（{source}触发）")
            
        elif cmd == 'stop_follow':
            self.is_following = False  # 状态同步：退出跟随
            self.get_logger().info(f"🚗 执行动作：小车停止跟随（{source}触发）")

        self.last_cmd_timestamp = time.time()

    def voice_trash_callback(self, msg):
        cmd = self._normalize_cmd(msg.data)
        if cmd in ['open', 'close']:
            self.handle_trash_bin_cmd(cmd, 'voice')

    def voice_car_callback(self, msg):
        cmd = self._normalize_cmd(msg.data)
        if cmd in ['follow', 'come', 'stop_follow']:
            self.handle_car_cmd(cmd, 'voice')

    def gesture_callback(self, msg):
        cmd = self._normalize_cmd(msg.data)
        if cmd in ['open', 'close']:
            self.handle_trash_bin_cmd(cmd, 'gesture')
        elif cmd in ['follow', 'come', 'stop_follow']:
            self.handle_car_cmd(cmd, 'gesture')

    def destroy_node(self):
        if self.ser_hc06 and self.ser_hc06.is_open:
            try:
                self.ser_hc06.write(b'2')
                self.get_logger().info('📤 程序关闭，强制向HC-06发送2（关闭垃圾桶）')
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"❌ 强制发送2失败：{e}")

        if self.ser_hc06 and self.ser_hc06.is_open:
            self.ser_hc06.close()
            self.get_logger().info('🔌 HC-06串口已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DeviceControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()