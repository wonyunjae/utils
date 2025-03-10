#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
import utm
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import math
from tf_transformations import euler_from_quaternion

class RelativeToAbsolute(Node):
    def __init__(self):
        super().__init__('relative_to_absolute')
        
        # QoS 프로필 설정
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # 초기 좌표 및 방향 저장 변수
        self.initial_gps_utm = None
        self.initial_heading = None
        self.initial_odom_yaw = None
        self.theta = None  # heading과 odom orientation의 차이
        self.rotation_matrix = None  # 2D 회전 행렬
        self.first_data_received = False
        
        # IMU 구독자 추가
        self.imu_sub = self.create_subscription(
            Imu, '/imu_xsens', self.imu_callback, qos)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/fix_mrp_2000', self.gps_callback, qos)
        self.glim_sub = self.create_subscription(
            Odometry, '/glim_ros/odom', self.glim_callback, qos)
        self.liosam_sub = self.create_subscription(
            Odometry, '/lio_sam/mapping/odometry', self.liosam_callback, qos)
        
        # 최근 데이터 저장
        self.latest_gps = None
        self.latest_glim = None
        self.latest_liosam = None
        self.latest_imu = None
        
        # 타이머 설정 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 발행자 설정
        self.gps_utm_pub = self.create_publisher(PointStamped, '/gps_utm', 10)
        self.glim_utm_pub = self.create_publisher(PointStamped, '/glim_utm', 10)
        self.liosam_utm_pub = self.create_publisher(PointStamped, '/liosam_utm', 10)
        self.gps_rel_pub = self.create_publisher(PointStamped, '/gps_rel', 10)
        
        self.get_logger().info('GPS와 Odometry를 UTM 좌표계로 변환하는 노드가 시작되었습니다.')
    
    def imu_callback(self, msg):
        self.latest_imu = msg
    
    def get_yaw_from_quaternion(self, quaternion):
        """쿼터니언에서 yaw 각도 추출"""
        _, _, yaw = euler_from_quaternion([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        ])
        return yaw
    
    def calculate_rotation_matrix(self, theta):
        """2D 회전 행렬 계산"""
        return np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
    
    def apply_rotation(self, x, y):
        """회전 행렬을 좌표에 적용"""
        if self.rotation_matrix is None:
            return x, y
        rotated = self.rotation_matrix @ np.array([x, y])
        return rotated[0], rotated[1]
    
    def gps_callback(self, msg):
        self.latest_gps = msg
    
    def glim_callback(self, msg):
        self.latest_glim = msg
    
    def liosam_callback(self, msg):
        self.latest_liosam = msg
    
    def timer_callback(self):
        # 모든 데이터가 수신되었는지 확인
        if not all([self.latest_gps, self.latest_glim, self.latest_liosam, self.latest_imu]):
            return
        
        # 유효한 GPS 데이터인지 확인
        if self.latest_gps.latitude == 0.0 and self.latest_gps.longitude == 0.0:
            return
        
        # GPS를 UTM 좌표로 변환
        utm_coords = utm.from_latlon(self.latest_gps.latitude, self.latest_gps.longitude)
        gps_utm_x, gps_utm_y = utm_coords[0], utm_coords[1]
        
        # GPS UTM 좌표 발행
        gps_utm_point = PointStamped()
        gps_utm_point.header = self.latest_gps.header
        gps_utm_point.header.frame_id = 'map'
        gps_utm_point.point.x = gps_utm_x
        gps_utm_point.point.y = gps_utm_y
        gps_utm_point.point.z = 0.0
        self.gps_utm_pub.publish(gps_utm_point)
        
        # 첫 번째 데이터인 경우 초기값 설정
        if not self.first_data_received:
            self.initial_gps_utm = (gps_utm_x, gps_utm_y)
            
            # IMU에서 초기 heading 계산
            self.initial_heading = self.get_yaw_from_quaternion(self.latest_imu.orientation)
            
            # GLIM odometry에서 초기 yaw 계산
            self.initial_odom_yaw = self.get_yaw_from_quaternion(
                self.latest_glim.pose.pose.orientation)
            
            # 수정안 1: 부호 반전
            self.theta = self.initial_odom_yaw - self.initial_heading - 0.125
            
            # 회전 행렬 계산
            self.rotation_matrix = self.calculate_rotation_matrix(self.theta)
            
            self.first_data_received = True
            self.get_logger().info(f'초기 GPS UTM 좌표: ({self.initial_gps_utm[0]:.2f}, {self.initial_gps_utm[1]:.2f})')
            self.get_logger().info(f'초기 Heading: {math.degrees(self.initial_heading):.2f}°')
            self.get_logger().info(f'초기 Odom Yaw: {math.degrees(self.initial_odom_yaw):.2f}°')
            self.get_logger().info(f'보정 각도 (theta): {math.degrees(self.theta):.2f}°')
        
        # GPS 상대 좌표 계산
        gps_rel_point = PointStamped()
        gps_rel_point.header.stamp = self.latest_gps.header.stamp
        gps_rel_point.header.frame_id = 'odom'
        gps_rel_point.point.x = gps_utm_x - self.initial_gps_utm[0]
        gps_rel_point.point.y = gps_utm_y - self.initial_gps_utm[1]
        gps_rel_point.point.z = 0.0
        self.gps_rel_pub.publish(gps_rel_point)
        
        # GLIM Odometry 데이터를 UTM 좌표로 변환 (회전 적용)
        glim_x = self.latest_glim.pose.pose.position.x
        glim_y = self.latest_glim.pose.pose.position.y
        rotated_glim_x, rotated_glim_y = self.apply_rotation(glim_x, glim_y)
        
        glim_utm_point = PointStamped()
        glim_utm_point.header = self.latest_glim.header
        glim_utm_point.header.frame_id = 'map'
        glim_utm_point.point.x = self.initial_gps_utm[0] + rotated_glim_x + 5
        glim_utm_point.point.y = self.initial_gps_utm[1] + rotated_glim_y - 2
        glim_utm_point.point.z = self.latest_glim.pose.pose.position.z
        self.glim_utm_pub.publish(glim_utm_point)
        
        # LIO-SAM Odometry 데이터를 UTM 좌표로 변환 (회전 적용)
        liosam_x = self.latest_liosam.pose.pose.position.x
        liosam_y = self.latest_liosam.pose.pose.position.y
        rotated_liosam_x, rotated_liosam_y = self.apply_rotation(liosam_x, liosam_y)
        
        liosam_utm_point = PointStamped()
        liosam_utm_point.header = self.latest_liosam.header
        liosam_utm_point.header.frame_id = 'map'
        liosam_utm_point.point.x = self.initial_gps_utm[0] + rotated_liosam_x + 5
        liosam_utm_point.point.y = self.initial_gps_utm[1] + rotated_liosam_y - 2
        liosam_utm_point.point.z = self.latest_liosam.pose.pose.position.z
        self.liosam_utm_pub.publish(liosam_utm_point)
        
        # 디버깅용 로그 출력
        self.get_logger().info(f'GPS UTM: ({gps_utm_x:.2f}, {gps_utm_y:.2f})')
        self.get_logger().info(f'GLIM UTM: ({glim_utm_point.point.x:.2f}, {glim_utm_point.point.y:.2f})')
        self.get_logger().info(f'LIO-SAM UTM: ({liosam_utm_point.point.x:.2f}, {liosam_utm_point.point.y:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = RelativeToAbsolute()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()