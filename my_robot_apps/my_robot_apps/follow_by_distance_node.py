import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool
from geometry_msgs.msg import Twist
import numpy as np
import time

# 超时阈值：超过此时间未收到 /cmd_orientation 则视为跟随模式已退出，停止发布
_FOLLOW_TIMEOUT = 1.0  # 秒

# 定时发布频率（Hz），高频持续发布保证前进指令连续，消除"走—停—走—停"现象
_PUBLISH_HZ = 20

# 最大跟随速度（m/s）。修改此值可直接改变跟随模式的最高前进速度。
MAX_FOLLOW_SPEED = 0.5

# 旋转制动等待时间（秒）
_ROTATION_HOLD_DELAY = 0.25


# ✅【修复 1】补上缺失的 class 定义！
class FollowByDistanceNode(Node):
    def __init__(self):
        super().__init__('follow_by_distance_node')
        self.create_subscription(Float32, '/person_distance', self.cb_distance, 10)
        self.create_subscription(String, '/cmd_orientation', self.cb_orientation, 10)

        self.come_mode_active = False
        self.create_subscription(Bool, '/come_mode_active', self._come_mode_cb, 10)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.orientation = "none"
        self.active = False
        self.last_orientation_time = None
        self._cached_twist = Twist()
        self.rotation_hold_until = 0.0

        self.create_timer(1.0 / _PUBLISH_HZ, self._timer_pub)

        self.get_logger().info(
            f"follow_by_distance_node 启动，最大速度={MAX_FOLLOW_SPEED}m/s，"
            "仅在 center 方向且 come 模式未激活时发布 /cmd_vel。")

    def _come_mode_cb(self, msg):
        self.come_mode_active = msg.data
        if self.come_mode_active:
            self._cached_twist = Twist()
            self.get_logger().info("🔒 come 模式激活，follow_by_distance_node 停止输出 /cmd_vel。")

    def cb_orientation(self, msg):
        prev_orientation = self.orientation
        self.orientation = msg.data
        self.active = True
        self.last_orientation_time = time.monotonic()

        if prev_orientation in ('left', 'right') and self.orientation in ('center', 'none'):
            self.rotation_hold_until = time.monotonic() + _ROTATION_HOLD_DELAY
            self.get_logger().info("旋转结束，等待 0.25s 制动后再接管 /cmd_vel")

        self.get_logger().info(f"收到手势方向：{self.orientation}")

    def _check_timeout(self):
        if self.last_orientation_time is not None and \
                (time.monotonic() - self.last_orientation_time) > _FOLLOW_TIMEOUT:
            self.active = False
            self._cached_twist = Twist()

    def cb_distance(self, msg):
        self._check_timeout()

        if not self.active:
            self._cached_twist = Twist()
            return

        if self.come_mode_active:
            return

        # ✅【优化】仅 center 方向允许前进，none 方向不前进（更安全）
        if self.orientation != "center":
            self._cached_twist = Twist()
            return

        dist = msg.data
        twist = Twist()
        twist.angular.z = 0.0

        if np.isnan(dist) or dist == -1.0:
            twist.linear.x = 0.0
            self.get_logger().warn(f"无效距离：{dist}，停止")
        elif dist < 0.3:
            twist.linear.x = -0.1
            self.get_logger().info(f"距离:{dist:.2f} → 后退")
        elif dist >= 1.0:
            twist.linear.x = MAX_FOLLOW_SPEED
            self.get_logger().info(f"距离:{dist:.2f} → 最大速度前进")
        else:
            speed = (dist - 0.3) / 0.7 * MAX_FOLLOW_SPEED
            twist.linear.x = speed
            self.get_logger().info(f"距离:{dist:.2f} → 渐进速度({speed:.2f})")

        self._cached_twist = twist

    def _timer_pub(self):
        self._check_timeout()

        if self.come_mode_active:
            return
        if not self.active:
            return
        if time.monotonic() < self.rotation_hold_until:
            return

        # ✅【严格安全】仅 center 方向发布
        if self.orientation != "center":
            return

        self.pub.publish(self._cached_twist)

def main(args=None):
    rclpy.init(args=args)
    node = FollowByDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
