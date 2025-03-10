#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
import utm

class GpsToUtm(Node):
    def __init__(self):
        super().__init__('gps_to_utm')
        
        # 구독자와 발행자 설정
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix_mrp_2000',
            self.gps_callback,
            10)
        self.publisher = self.create_publisher(
            PointStamped,
            '/gps_abs',
            10)
    
    def gps_callback(self, msg):
        if not (msg.latitude == 0.0 and msg.longitude == 0.0):
            # UTM 변환
            utm_coords = utm.from_latlon(msg.latitude, msg.longitude)
            utm_x, utm_y = utm_coords[0], utm_coords[1]
            
            # PointStamped 메시지 생성 및 발행
            point_msg = PointStamped()
            point_msg.header = msg.header
            point_msg.header.frame_id = 'utm'
            point_msg.point.x = utm_x
            point_msg.point.y = utm_y
            point_msg.point.z = msg.altitude if not None else 0.0
            
            self.publisher.publish(point_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsToUtm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 