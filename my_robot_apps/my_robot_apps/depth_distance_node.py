import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import numpy as np
from cv_bridge import CvBridge

class DepthDistanceNode(Node):
    def __init__(self):
        super().__init__('depth_distance_node')
        self.create_subscription(
            Image, 
            '/ascamera_hp60c_2/depth/image_raw', 
            self.cb, 
            10
        )
        self.pub = self.create_publisher(Float32, '/person_distance', 10)
        self.bridge = CvBridge()
        self.get_logger().info("深度距离计算节点启动（相机输出：毫米→转米）！")

    def cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            h, w = img.shape
            roi = img[h//2-4:h//2+5, w//2-4:w//2+5]  # 中心9x9像素
            
            # 1. 过滤无效深度值（nan/0/超过4米的超大值）
            valid_roi = roi[np.isfinite(roi)]  # 去掉nan/inf
            valid_roi = valid_roi[(valid_roi > 200) & (valid_roi < 4000)]  # 200<毫米值<4000（即0.2~4米）
            
            if len(valid_roi) == 0:
                dist = -1.0  # 无有效数据，发特殊标记
                self.get_logger().warn("ROI无有效深度数据，发布-1.0")
            else:
                # 2. 核心：毫米(mm) → 米(m) 单位转换（已取消注释，直接用）
                dist_median_mm = np.median(valid_roi)
                dist = dist_median_mm / 1000.0  # 关键转换行！
                self.get_logger().info(f"原始毫米值：{dist_median_mm:.0f} → 实际距离：{dist:.2f}米")
            
            # 发布转换后的米单位距离
            out = Float32()
            out.data = dist
            self.pub.publish(out)
        
        except Exception as e:
            self.get_logger().error(f"处理深度图失败：{str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()