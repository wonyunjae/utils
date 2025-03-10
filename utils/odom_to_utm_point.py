#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer

class OdomToUtmPoint(Node):
    def __init__(self):
        super().__init__('odom_to_utm_point')
        
        # 초기 좌표 저장 변수
        self.initial_gps_utm = None
        self.first_data_received = False
        
        # 입력 데이터 구독자
        self.gps_utm_sub = Subscriber(self, PointStamped, '/gps_utm')
        self.glim_odom_sub = Subscriber(self, Odometry, '/glim_utm')
        self.liosam_odom_sub = Subscriber(self, Odometry, '/liosam_utm')
        
        # 타임 싱크로나이저 설정
        self.sync = ApproximateTimeSynchronizer(
            [self.gps_utm_sub, self.glim_odom_sub, self.liosam_odom_sub],
            queue_size=10, slop=0.1)
        self.sync.registerCallback(self.sync_callback)
        
        # 상대 좌표용 발행자
        self.gps_utm_rel_pub = self.create_publisher(PointStamped, '/gps_utm_rel', 10)
        self.glim_utm_point_pub = self.create_publisher(PointStamped, '/glim_utm_point', 10)
        self.liosam_utm_point_pub = self.create_publisher(PointStamped, '/liosam_utm_point', 10)
        
        # 절대 좌표용 발행자 (옵션)
        self.glim_utm_abs_pub = self.create_publisher(PointStamped, '/glim_abs', 10)
        self.liosam_utm_abs_pub = self.create_publisher(PointStamped, '/liosam_abs', 10)
        
        self.get_logger().info('Odom to UTM Point 변환기가 시작되었습니다.')
    
    # 수정된 sync_callback 함수
    def sync_callback(self, gps_utm, glim_odom, liosam_odom):
        # 첫 데이터 기록
        if not self.first_data_received:
            self.initial_gps_utm = (gps_utm.point.x, gps_utm.point.y)
            self.initial_glim = (glim_odom.pose.pose.position.x, glim_odom.pose.pose.position.y)
            self.initial_liosam = (liosam_odom.pose.pose.position.x, liosam_odom.pose.pose.position.y)
            self.first_data_received = True
            self.get_logger().info(f'첫 GPS UTM 좌표: ({self.initial_gps_utm[0]:.2f}, {self.initial_gps_utm[1]:.2f})')
            self.get_logger().info(f'첫 GLIM 좌표: ({self.initial_glim[0]:.2f}, {self.initial_glim[1]:.2f})')
            self.get_logger().info(f'첫 LIO-SAM 좌표: ({self.initial_liosam[0]:.2f}, {self.initial_liosam[1]:.2f})')
        
        # GPS UTM 상대좌표 계산 
        gps_rel = PointStamped()
        gps_rel.header = gps_utm.header
        gps_rel.point.x = gps_utm.point.x - self.initial_gps_utm[0]
        gps_rel.point.y = gps_utm.point.y - self.initial_gps_utm[1]
        gps_rel.point.z = gps_utm.point.z
        
        # GLIM Odometry 상대 좌표 (초기 위치로부터의 변화)
        glim_point = PointStamped()
        glim_point.header = glim_odom.header
        glim_point.point.x = glim_odom.pose.pose.position.x - self.initial_glim[0]
        glim_point.point.y = glim_odom.pose.pose.position.y - self.initial_glim[1]
        glim_point.point.z = glim_odom.pose.pose.position.z
        
        # LIO-SAM Odometry 상대 좌표 (초기 위치로부터의 변화)
        liosam_point = PointStamped()
        liosam_point.header = liosam_odom.header
        liosam_point.point.x = liosam_odom.pose.pose.position.x - self.initial_liosam[0]
        liosam_point.point.y = liosam_odom.pose.pose.position.y - self.initial_liosam[1]
        liosam_point.point.z = liosam_odom.pose.pose.position.z
        
        # GLIM 절대 UTM 좌표 계산 (초기 GPS에 상대 이동량 더하기)
        glim_abs = PointStamped()
        glim_abs.header = glim_odom.header
        glim_abs.point.x = self.initial_gps_utm[0] + (glim_odom.pose.pose.position.x - self.initial_glim[0])
        glim_abs.point.y = self.initial_gps_utm[1] + (glim_odom.pose.pose.position.y - self.initial_glim[1])
        glim_abs.point.z = glim_odom.pose.pose.position.z
        
        # LIO-SAM 절대 UTM 좌표 계산 (초기 GPS에 상대 이동량 더하기)
        liosam_abs = PointStamped()
        liosam_abs.header = liosam_odom.header
        liosam_abs.point.x = self.initial_gps_utm[0] + (liosam_odom.pose.pose.position.x - self.initial_liosam[0])
        liosam_abs.point.y = self.initial_gps_utm[1] + (liosam_odom.pose.pose.position.y - self.initial_liosam[1])
        liosam_abs.point.z = liosam_odom.pose.pose.position.z
        
        # 발행
        self.gps_utm_rel_pub.publish(gps_rel)
        self.glim_utm_point_pub.publish(glim_point)
        self.liosam_utm_point_pub.publish(liosam_point)
        self.glim_utm_abs_pub.publish(glim_abs)
        self.liosam_utm_abs_pub.publish(liosam_abs)
        
        # 로그 출력
        self.get_logger().info(f'GPS UTM(상대): ({gps_rel.point.x:.2f}, {gps_rel.point.y:.2f})')
        self.get_logger().info(f'GLIM UTM(상대): ({glim_point.point.x:.2f}, {glim_point.point.y:.2f})')
        self.get_logger().info(f'LIO-SAM UTM(상대): ({liosam_point.point.x:.2f}, {liosam_point.point.y:.2f})')
        self.get_logger().info(f'GLIM UTM(절대): ({glim_abs.point.x:.2f}, {glim_abs.point.y:.2f})')
        self.get_logger().info(f'LIO-SAM UTM(절대): ({liosam_abs.point.x:.2f}, {liosam_abs.point.y:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = OdomToUtmPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()