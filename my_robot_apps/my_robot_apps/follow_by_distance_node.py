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
# 注意：原代码写死为 0.2，导致即使想设 0.5 也不起作用；此处统一用常量控制。
MAX_FOLLOW_SPEED = 0.5


class FollowByDistanceNode(Node):
    def __init__(self):
        super().__init__('follow_by_distance_node')
        self.create_subscription(Float32, '/person_distance', self.cb_distance, 10)
        self.create_subscription(String, '/cmd_orientation', self.cb_orientation, 10)

        # ── 互斥订阅：come 模式激活时本节点必须完全停止发布 /cmd_vel ──────────────
        # come_mode_node 发布 /come_mode_active（Bool），True 表示 come 模式正在运行。
        # 只要 come 模式激活，本节点立即清零缓存指令并停止向 /cmd_vel 写入，
        # 避免两个控制节点同时抢占 /cmd_vel 造成车轮抖动或指令相互覆盖。
        self.come_mode_active = False
        self.create_subscription(Bool, '/come_mode_active', self._come_mode_cb, 10)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.orientation = "none"

        # active 标志：只有在 /cmd_orientation 持续到来（跟随模式激活）时才发布 /cmd_vel。
        # 当超过 _FOLLOW_TIMEOUT 秒未收到新消息时自动失活，停止向 /cmd_vel 写入，
        # 从而避免与 come_mode_node / turn_to_person_node 产生指令冲突。
        self.active = False
        self.last_orientation_time = None  # 上次收到 /cmd_orientation 的时间戳

        # 缓存上一次计算出的速度指令，由定时器持续发布（提高频率）
        self._cached_twist = Twist()

        # 高频定时器：以 _PUBLISH_HZ Hz 持续向 /cmd_vel 发布缓存的指令，
        # 避免因距离消息间隔导致的"走—停—走—停"现象
        self.create_timer(1.0 / _PUBLISH_HZ, self._timer_pub)

        self.get_logger().info(
            f"follow_by_distance_node 启动，最大速度={MAX_FOLLOW_SPEED}m/s，"
            "仅在 center 方向且 come 模式未激活时发布 /cmd_vel。")

    # ──────────────────────────────────────────────────────────────────────────
    # 互斥回调：come 模式状态变更
    # ──────────────────────────────────────────────────────────────────────────
    def _come_mode_cb(self, msg):
        """come 模式激活/退出时更新互斥标志。
        come 模式激活时立即清零缓存速度，确保本节点不再向 /cmd_vel 写入。"""
        self.come_mode_active = msg.data
        if self.come_mode_active:
            # come 模式已激活 → 清零缓存，停止本节点所有输出
            self._cached_twist = Twist()
            self.get_logger().info("🔒 come 模式激活，follow_by_distance_node 停止输出 /cmd_vel。")

    def cb_orientation(self, msg):
        self.orientation = msg.data
        # 收到方向指令：激活节点并刷新计时器
        self.active = True
        self.last_orientation_time = time.monotonic()
        self.get_logger().info(f"收到手势方向：{self.orientation}")

    def _check_timeout(self):
        """检测跟随模式是否超时，超时则失活并清零缓存指令。"""
        if self.last_orientation_time is not None and \
                (time.monotonic() - self.last_orientation_time) > _FOLLOW_TIMEOUT:
            self.active = False
            self._cached_twist = Twist()

    def cb_distance(self, msg):
        # 超时检测：若已超过 _FOLLOW_TIMEOUT 秒未收到方向指令，则认为跟随模式已关闭
        self._check_timeout()

        if not self.active:
            # 非激活时确保缓存已清零，不占用 /cmd_vel（由定时器统一发布）
            self._cached_twist = Twist()
            return

        # ── 互斥检查：come 模式激活时本节点不计算也不缓存速度 ─────────────────────
        if self.come_mode_active:
            return

        # ── 仅在 center 方向下计算速度；左/右方向时不更新缓存 ────────────────────
        # 左/右方向由 turn_to_person_node 独占控制 /cmd_vel，本节点此时不发布任何指令，
        # 防止两个节点同时写 /cmd_vel 造成速度相互覆盖或车轮抖动。
        if self.orientation != "center":
            # 方向非 center：清零缓存但不发布（定时器看到 orientation!=center 也会跳过）
            self._cached_twist = Twist()
            return

        dist = msg.data
        twist = Twist()
        twist.angular.z = 0.0

        if np.isnan(dist) or dist == -1.0:
            # 距离无效：停车
            twist.linear.x = 0.0
            self.get_logger().warn(f"接收无效距离：{dist}，小车停止")
        elif dist < 0.3:
            # 过近：后退
            twist.linear.x = -0.1
            self.get_logger().info(f"距离:{dist:.2f}米 → 过近后退(-0.1m/s)")
        elif dist >= 1.0:
            # 距离较远：以最大速度持续前进
            # 原代码此处写死 0.2，与用户设置的 max_speed 不符；
            # 修复后直接使用 MAX_FOLLOW_SPEED，确保设置生效。
            twist.linear.x = MAX_FOLLOW_SPEED
            self.get_logger().info(
                f"距离:{dist:.2f}米 → 最大速度前进({MAX_FOLLOW_SPEED:.2f}m/s)")
        else:
            # 0.3 ~ 1.0 m 之间：按比例渐进，最终接近 MAX_FOLLOW_SPEED
            speed = (dist - 0.3) / (1.0 - 0.3) * MAX_FOLLOW_SPEED
            twist.linear.x = speed
            self.get_logger().info(f"距离:{dist:.2f}米 → 渐进速度({speed:.2f}m/s)")

        # 更新缓存；定时器将在每个周期持续发布，确保连续帧都有非零速度
        self._cached_twist = twist

    def _timer_pub(self):
        """高频定时发布缓存的速度指令。
        修复要点：
          1. come 模式激活时完全不发布（互斥）。
          2. 方向非 center 时不发布（让 turn_to_person_node 独占此时的 /cmd_vel），
             避免两节点同时写 /cmd_vel 导致的速度覆盖与抖动。
          3. 方向为 center 且激活时，每帧都发布缓存速度，保证前进指令连续，
             彻底消除"走一帧停两帧"的走走停停现象。
        """
        self._check_timeout()

        # 互斥：come 模式激活 → 本节点静默
        if self.come_mode_active:
            return

        if not self.active:
            return

        # 方向控制互斥：非 center 时让 turn_to_person_node 独占 /cmd_vel
        # 本节点不发布任何内容（包括零速），避免覆盖转向指令
        if self.orientation != "center":
            return

        # center 方向 + 激活状态：每帧持续发布缓存速度，机器人连贯前进
        self.pub.publish(self._cached_twist)

def main(args=None):
    rclpy.init(args=args)
    node = FollowByDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
