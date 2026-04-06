import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
import numpy as np
import time

# 超时阈值：超过此时间未收到 /cmd_orientation 则视为跟随模式已退出，停止发布
_FOLLOW_TIMEOUT = 1.0  # 秒

class FollowByDistanceNode(Node):
    def __init__(self):
        super().__init__('follow_by_distance_node')
        self.create_subscription(Float32, '/person_distance', self.cb_distance, 10)
        self.create_subscription(String, '/cmd_orientation', self.cb_orientation, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.orientation = "none"

        # active 标志：只有在 /cmd_orientation 持续到来（跟随模式激活）时才发布 /cmd_vel。
        # 当超过 _FOLLOW_TIMEOUT 秒未收到新消息时自动失活，停止向 /cmd_vel 写入，
        # 从而避免与 come_mode_node / turn_to_person_node 产生指令冲突。
        self.active = False
        self.last_orientation_time = None  # 上次收到 /cmd_orientation 的时间戳

        self.get_logger().info("仅在center时允许距离跟随小车移动。其它一律停止。")

    def cb_orientation(self, msg):
        self.orientation = msg.data
        # 收到方向指令：激活节点并刷新计时器
        self.active = True
        self.last_orientation_time = time.monotonic()
        self.get_logger().info(f"收到手势方向：{self.orientation}")

    def cb_distance(self, msg):
        # 超时检测：若已超过 _FOLLOW_TIMEOUT 秒未收到方向指令，则认为跟随模式已关闭
        if self.last_orientation_time is not None and (time.monotonic() - self.last_orientation_time) > _FOLLOW_TIMEOUT:
            self.active = False

        # 仅在激活状态（跟随模式运行中）时才发布，其余时候直接 return 不占用 /cmd_vel
        if not self.active:
            return

        dist = msg.data
        twist = Twist()
        twist.angular.z = 0.0

        # 只在center时跟随，否则强制停止
        if self.orientation != "center":
            twist.linear.x = 0.0
            self.pub.publish(twist)
            self.get_logger().info("非center小车停止。")
            return

        if np.isnan(dist) or dist == -1.0:
            twist.linear.x = 0.0
            self.get_logger().warn(f"接收无效距离：{dist}，小车停止")
        elif dist < 0.3:
            twist.linear.x = -0.1
            self.get_logger().info(f"距离:{dist:.2f}米 → 过近后退(-0.1m/s)")
        elif dist >= 1.0:
            twist.linear.x = 0.2
            self.get_logger().info(f"距离:{dist:.2f}米 → 距离较远前进(0.2m/s)")
        else:
            speed = (dist - 0.3) / (1.0 - 0.3) * 0.2
            twist.linear.x = speed
            self.get_logger().info(f"距离:{dist:.2f}米 → 渐进速度({speed:.2f}m/s)")

        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FollowByDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()