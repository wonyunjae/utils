#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import os
import numpy as np
from tf_transformations import quaternion_from_euler
import argparse

class TUMSaver(Node):
    def __init__(self, output_dir='.'):
        super().__init__('tum_saver')
        
        # 출력 디렉토리 생성
        self.output_dir = '/home/wynz/ros2_ws/src/utils/utils/tum/'
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 파일 오픈
        self.gps_file = open(os.path.join(self.output_dir, 'gps_utm.txt'), 'w')
        self.glim_file = open(os.path.join(self.output_dir, 'glim_utm.txt'), 'w')
        self.liosam_file = open(os.path.join(self.output_dir, 'liosam_utm.txt'), 'w')
        
        # 헤더 추가 (선택적)
        self.gps_file.write('# timestamp tx ty tz qx qy qz qw\n')
        self.glim_file.write('# timestamp tx ty tz qx qy qz qw\n')
        self.liosam_file.write('# timestamp tx ty tz qx qy qz qw\n')
        
        # 구독자 설정
        self.gps_sub = self.create_subscription(
            PointStamped, '/gps_utm', self.gps_callback, 10)
        self.glim_sub = self.create_subscription(
            PointStamped, '/glim_utm', self.glim_callback, 10)
        self.liosam_sub = self.create_subscription(
            PointStamped, '/liosam_utm', self.liosam_callback, 10)
        
        # 카운터 및 로깅
        self.gps_count = 0
        self.glim_count = 0
        self.liosam_count = 0
        self.get_logger().info(f'TUM 형식 저장기가 시작되었습니다. 출력 경로: {self.output_dir}')
    
    def write_tum_format(self, file, msg):
        # TUM 형식: timestamp tx ty tz qx qy qz qw
        # PointStamped는 방향 정보가 없으므로 기본값으로 설정 (단위 쿼터니언)
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        file.write(f"{timestamp:.9f} {msg.point.x:.6f} {msg.point.y:.6f} {msg.point.z:.6f} 0.0 0.0 0.0 1.0\n")
        file.flush()  # 즉시 디스크에 기록
    
    def gps_callback(self, msg):
        self.write_tum_format(self.gps_file, msg)
        self.gps_count += 1
        if self.gps_count % 100 == 0:
            self.get_logger().info(f'GPS UTM 데이터 {self.gps_count}개 저장됨')
    
    def glim_callback(self, msg):
        self.write_tum_format(self.glim_file, msg)
        self.glim_count += 1
        if self.glim_count % 100 == 0:
            self.get_logger().info(f'GLIM UTM 데이터 {self.glim_count}개 저장됨')
    
    def liosam_callback(self, msg):
        self.write_tum_format(self.liosam_file, msg)
        self.liosam_count += 1
        if self.liosam_count % 100 == 0:
            self.get_logger().info(f'LIO-SAM UTM 데이터 {self.liosam_count}개 저장됨')
    
    def close_files(self):
        self.gps_file.close()
        self.glim_file.close()
        self.liosam_file.close()
        self.get_logger().info(f'총 저장된 데이터: GPS={self.gps_count}, GLIM={self.glim_count}, LIO-SAM={self.liosam_count}')

def main(args=None):
    parser = argparse.ArgumentParser(description='ROS2 토픽을 TUM 형식으로 저장')
    parser.add_argument('--output_dir', '-o', type=str, default='./trajectories',
                       help='출력 디렉토리 경로 (기본값: ./trajectories)')
    
    rclpy.init(args=args)
    
    if args is not None:
        args = parser.parse_args(args)
    else:
        args = parser.parse_args()
    
    node = TUMSaver(output_dir=args.output_dir)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_files()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
