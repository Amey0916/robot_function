import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

# 超时阈值：超过此时间未收到 /cmd_orientation 则视为跟随模式已退出，停止发布
_FOLLOW_TIMEOUT = 1.0  # 秒

class TurnToPersonNode(Node):
    def __init__(self):
        super().__init__('turn_to_person_node')
        self.direction = "center"          # 初始方向

        # active 标志：只有在 /cmd_orientation 持续到来（跟随模式激活）时才发布 /cmd_vel。
        # 当超过 _FOLLOW_TIMEOUT 秒未收到新消息时自动失活，停止向 /cmd_vel 写入，
        # 从而避免与 come_mode_node / follow_by_distance_node 产生指令冲突。
        self.active = False
        self.last_cmd_time = None          # 上次收到 /cmd_orientation 的时间戳

        self.create_subscription(String, '/cmd_orientation', self.cmd_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0/30.0, self.publish_twist)  # 30Hz，与检测帧率匹配，保证转向连贯

    def cmd_cb(self, msg):
        # 收到新方向：更新方向、激活节点并刷新计时器，同时立即发布一次保证实时响应
        self.direction = msg.data
        self.active = True
        self.last_cmd_time = time.monotonic()
        self._do_publish()

    def _do_publish(self):
        twist = Twist()
        if self.direction == 'left':
            twist.angular.z = 0.1
        elif self.direction == 'right':
            twist.angular.z = -0.1
        elif self.direction == 'center' or self.direction == "none":
            twist.angular.z = 0.0
        self.pub.publish(twist)

    def publish_twist(self):
        # 超时检测：若已超过 _FOLLOW_TIMEOUT 秒未收到方向指令，则认为跟随模式已关闭
        if self.last_cmd_time is not None and (time.monotonic() - self.last_cmd_time) > _FOLLOW_TIMEOUT:
            self.active = False

        # 仅在激活状态（跟随模式运行中）时才发布，其余时候直接 return 不占用 /cmd_vel
        if not self.active:
            return

        self._do_publish()

def main(args=None):
    rclpy.init(args=args)
    node = TurnToPersonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()