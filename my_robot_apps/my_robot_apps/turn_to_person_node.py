import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TurnToPersonNode(Node):
    def __init__(self):
        super().__init__('turn_to_person_node')
        self.direction = "center"          # 初始状态（可设为none）
        self.create_subscription(String, '/cmd_orientation', self.cmd_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_twist)  # 每0.1秒发布一次

    def cmd_cb(self, msg):
        # 收到新方向，更新当前方向
        self.direction = msg.data

    def publish_twist(self):
        twist = Twist()
        if self.direction == 'left':
            twist.angular.z = 0.1
        elif self.direction == 'right':
            twist.angular.z = -0.1
        elif self.direction == 'center' or self.direction == "none":
            twist.angular.z = 0.0
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurnToPersonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()