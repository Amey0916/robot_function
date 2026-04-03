#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class OriginMarkerNode(Node):
    def __init__(self):
        super().__init__('origin_marker_node')
        # 创建 Marker 话题发布者
        self.publisher_ = self.create_publisher(Marker, 'origin_marker', 10)
        # 定时器，每秒发布一次
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        marker = Marker()
        # 必须绑定到 map 坐标系（不管建图还是导航，绝对参考系都是 map）
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "origin"
        marker.id = 0
        marker.type = Marker.SPHERE  # 使用球体
        marker.action = Marker.ADD
        
        # 强制将位置定在建图的原点
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        
        # 将大小设为 0.3米，方便在地图上看清
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        # 设为红色 (RGBA)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = OriginMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()