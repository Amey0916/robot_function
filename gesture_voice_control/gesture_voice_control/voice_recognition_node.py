#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import os

os.environ.pop("ALSA_CONFIG_PATH", None)
os.environ.pop("CTL_CONFIG_PATH", None)

class VoiceRecognitionNode(Node):
    def __init__(self):
        super().__init__('voice_recognition_node')
        
        # 发布者
        self.trash_bin_pub = self.create_publisher(String, 'trash_bin/control', 10)
        self.car_pub = self.create_publisher(String, 'car/control', 10)
        self.follow_mode_pub = self.create_publisher(String, '/voice_follow_control', 10)
        self.get_logger().info("已创建follow模式发布者，话题：/voice_follow_control")

        # 语音识别配置
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 100
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.pause_threshold = 1.0

        # 自动选择 UGREEN 摄像头麦克风（按名称匹配）
        device_index = None
        try:
            mic_names = sr.Microphone.list_microphone_names()
            for i, name in enumerate(mic_names):
                if "UGREEN Camera 2K" in name or "UGREEN" in name:
                    device_index = i
                    break
        except Exception as e:
            self.get_logger().warn(f"枚举麦克风失败，使用默认设备: {e}")

        self.microphone = sr.Microphone(
            device_index=device_index,
            sample_rate=16000,
            chunk_size=2048
        )
        self.get_logger().info(f"当前使用麦克风索引: {device_index}")

        # 麦克风校准
        try:
            with self.microphone as source:
                self.get_logger().info("正在校准麦克风，请保持安静...")
                self.recognizer.adjust_for_ambient_noise(source, duration=0.8)
                self.get_logger().info("校准完成，麦克风已就绪！")
        except Exception as e:
            self.get_logger().error(f"校准失败: {str(e)}")

        # 定时器
        self.timer = self.create_timer(1.0, self.listen_voice)
        
        # ✅ 中英双语指令映射（中文+英文对应同一个指令）
        self.command_map = {
            # 停止跟随
            "停止跟随": ("car", "stop_follow"),
            "stop follow": ("car", "stop_follow"),
            "stop": ("car", "stop_follow"),
            
            # 跟随
            "跟随": ("car", "follow"),
            "follow": ("car", "follow"),
            
            # 打开
            "打开": ("trash_bin", "open"),
            "open": ("trash_bin", "open"),
            
            # 关闭
            "关闭": ("trash_bin", "close"),
            "close": ("trash_bin", "close"),
            
            # 过来
            "过来": ("car", "come"),
            "come": ("car", "come"),
            "come here": ("car", "come"),
        }

    def listen_voice(self):
        try:
            with self.microphone as source:
                self.get_logger().info("等待语音输入...")
                audio = self.recognizer.listen(
                    source,
                    timeout=8,
                    phrase_time_limit=6
                )
        
            audio_file = "/home/yhs/ros2_ws/test_audio.wav"
            with open(audio_file, "wb") as f:
                f.write(audio.get_wav_data())
            self.get_logger().info(f"录音已保存到: {audio_file}")
            
            # ✅ 双语识别：支持中文 + 英文
            text = self.recognizer.recognize_google(audio, language='zh-CN,en-US')
            text = text.strip().replace("，", "").replace("。", "")
            self.get_logger().info(f"识别到指令（清理后）: {text}")
            self.parse_command(text)
            
        except sr.WaitTimeoutError:
            pass
        except sr.UnknownValueError:
            # ✅ 双语提示
            self.get_logger().warn("无法识别语音！请清晰说：打开/关闭/跟随/过来/停止跟随 | open/close/follow/come/stop follow")
        except sr.RequestError as e:
            self.get_logger().error(f"识别失败（需外网）: {e}")
        except Exception as e:
            self.get_logger().error(f"监听异常: {str(e)}")

    def parse_command(self, text):
        matched = False
        self.get_logger().info(f"开始匹配指令：{text}")
        
        for keyword, (target, cmd) in self.command_map.items():
            if keyword in text:
                matched = True
                self.get_logger().info(f"匹配到关键词：{keyword} → 指令：{cmd}")
                
                msg = String()
                msg.data = cmd
                if target == "trash_bin":
                    self.trash_bin_pub.publish(msg)
                    self.get_logger().info(f"发布垃圾桶指令到 trash_bin/control: {cmd}")
                else:
                    self.car_pub.publish(msg)
                    self.get_logger().info(f"发布小车指令到 car/control: {cmd}")
                
                if cmd == "follow":
                    follow_msg = String()
                    follow_msg.data = "start_follow"
                    if self.follow_mode_pub.get_subscription_count() > 0:
                        self.follow_mode_pub.publish(follow_msg)
                        self.get_logger().info(f"成功发布start_follow到 /voice_follow_control")
                    else:
                        self.get_logger().warn(f"/voice_follow_control 暂无订阅者！")
                elif cmd == "stop_follow":
                    follow_msg = String()
                    follow_msg.data = "stop_follow"
                    if self.follow_mode_pub.get_subscription_count() > 0:
                        self.follow_mode_pub.publish(follow_msg)
                        self.get_logger().info(f"成功发布stop_follow到 /voice_follow_control")
                    else:
                        self.get_logger().warn(f"/voice_follow_control 暂无订阅者！")
                break
        
        if not matched:
            self.get_logger().warn(f"未匹配到任何指令：{text}")

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceRecognitionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("节点已停止")
    except Exception as e:
        print(f"启动失败: {str(e)}")
    finally:
        if node and rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
