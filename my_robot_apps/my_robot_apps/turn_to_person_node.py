import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import time

# 超时阈值：超过此时间未收到 /cmd_orientation 则视为跟随模式已退出，停止发布
_FOLLOW_TIMEOUT = 1.0  # 秒

# 旋转制动帧数：方向切回 center/none 后连续下发零角速度的帧数
# 计算：5 帧 × (1/30s) ≈ 0.17s 制动时间；不提升定时器频率，仅通过多帧补偿实现制动
_BRAKE_FRAMES = 5


class TurnToPersonNode(Node):
    def __init__(self):
        super().__init__('turn_to_person_node')
        self.direction = "center"          # 初始方向

        # active 标志：只有在 /cmd_orientation 持续到来（跟随模式激活）时才发布 /cmd_vel。
        # 当超过 _FOLLOW_TIMEOUT 秒未收到新消息时自动失活，停止向 /cmd_vel 写入，
        # 从而避免与 come_mode_node / follow_by_distance_node 产生指令冲突。
        self.active = False
        self.last_cmd_time = None          # 上次收到 /cmd_orientation 的时间戳

        # ── 旋转制动计数 ──────────────────────────────────────────────────────────
        # 修复要点：方向从 left/right 切换到 center/none 时，连续多帧下发 angular.z=0，
        # 低频定时器下也能可靠刹稳，防止角速度惯性导致的打滑过冲。
        # 不提升定时器频率，仅通过多帧补偿实现制动。
        self.zero_count = 0  # 剩余需要下发的零角速度帧数

        # ── 互斥订阅：come 模式激活时本节点必须完全停止发布 /cmd_vel ──────────────
        # come_mode_node 发布 /come_mode_active（Bool），True 表示 come 模式正在运行。
        # 只要 come 模式激活，本节点立即停止向 /cmd_vel 写入，
        # 避免两个控制节点同时抢占 /cmd_vel 造成车轮抖动或转向失效。
        self.come_mode_active = False
        self.create_subscription(Bool, '/come_mode_active', self._come_mode_cb, 10)

        self.create_subscription(String, '/cmd_orientation', self.cmd_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 30Hz 定时发布，与检测帧率匹配，保证转向指令连贯
        self.timer = self.create_timer(1.0 / 30.0, self.publish_twist)

    # ──────────────────────────────────────────────────────────────────────────
    # 互斥回调：come 模式状态变更
    # ──────────────────────────────────────────────────────────────────────────
    def _come_mode_cb(self, msg):
        """come 模式激活/退出时更新互斥标志。
        come 模式激活后本节点立即停止发布转向指令。"""
        self.come_mode_active = msg.data
        if self.come_mode_active:
            self.get_logger().info("🔒 come 模式激活，turn_to_person_node 停止输出 /cmd_vel。")

    def cmd_cb(self, msg):
        # 记录上次方向，用于检测旋转→停止的切换时刻
        prev_direction = self.direction
        # 收到新方向：更新方向、激活节点并刷新计时器
        self.direction = msg.data
        self.active = True
        self.last_cmd_time = time.monotonic()

        # ── 旋转→停止过渡处理 ─────────────────────────────────────────────────────
        # 从旋转（left/right）切换到中心（center/none）时，重置制动计数，
        # 后续定时器将连续发多帧 angular.z=0，确保低频下也能刹稳，防止打滑过冲。
        if prev_direction in ('left', 'right') and self.direction in ('center', 'none'):
            self.zero_count = _BRAKE_FRAMES  # ~0.17s 制动零速
        elif self.direction in ('left', 'right'):
            # 重新进入旋转：清除制动计数
            self.zero_count = 0

        # 仅当方向为 left/right 时立即发布一次，保证实时转向响应
        # center/none 方向由 follow_by_distance_node 负责，本节点不发布
        if self.direction in ('left', 'right') and not self.come_mode_active:
            self._do_publish()

    def _do_publish(self):
        """仅处理左转/右转，center/none 时不发布。
        修复要点：
          原来当 direction='center' 时会发布全零 Twist，以 30Hz 覆盖了
          follow_by_distance_node 正在发布的前进速度（20Hz），导致 /cmd_vel
          呈现"有速度 → 被零速覆盖 → 有速度 → 被零速覆盖"的走走停停现象。
          修复方案：center/none 时完全不发布，把 /cmd_vel 控制权交还给
          follow_by_distance_node，两节点按方向状态互斥地独占 /cmd_vel。
        """
        twist = Twist()
        if self.direction == 'left':
            twist.angular.z = 0.1
            self.pub.publish(twist)
        elif self.direction == 'right':
            twist.angular.z = -0.1
            self.pub.publish(twist)
        # center / none：不发布任何指令，让 follow_by_distance_node 独占 /cmd_vel

    def publish_twist(self):
        """定时回调（30Hz）。
        超时检测 → 互斥检查 → 制动零帧检查 → 方向检查，四道关卡确保只在恰当条件下发布转向指令。"""
        # 超时检测：若已超过 _FOLLOW_TIMEOUT 秒未收到方向指令，则认为跟随模式已关闭
        if self.last_cmd_time is not None and \
                (time.monotonic() - self.last_cmd_time) > _FOLLOW_TIMEOUT:
            self.active = False

        # 互斥：come 模式激活时本节点静默
        if self.come_mode_active:
            return

        # 仅在激活状态（跟随模式运行中）时才发布，其余时候直接 return 不占用 /cmd_vel
        if not self.active:
            return

        # ── 旋转制动：连续多帧下发零角速度，防止打滑过冲 ───────────────────────
        # 修复要点：原来 center/none 时完全不发布，底盘依靠惯性滑行停止，有时打滑。
        # 现在在方向切回 center/none 后的若干帧内，持续发布 angular.z=0，
        # 主动制动消除残余角速度，即使定时器频率低（30Hz）也可刹稳，不允许过冲。
        if self.zero_count > 0 and self.direction in ('center', 'none'):
            twist = Twist()
            twist.angular.z = 0.0  # 主动清零角速度
            self.pub.publish(twist)
            self.zero_count -= 1
            return  # 制动帧发完前不做其他发布

        # center/none 方向时不发布，避免零速覆盖 follow_by_distance_node 的前进指令
        if self.direction not in ('left', 'right'):
            return

        self._do_publish()

def main(args=None):
    rclpy.init(args=args)
    node = TurnToPersonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
