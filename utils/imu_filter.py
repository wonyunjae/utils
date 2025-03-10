#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from collections import deque

class ImuFilter(Node):
    def __init__(self):
        super().__init__('imu_filter')
        
        # 파라미터 설정
        self.declare_parameters(
            namespace='',
            parameters=[
                ('threshold', 5.0),     # 임계값 증가
                ('alpha', 0.2),         # 현재 값 반영 비율 증가
                ('buffer_size', 10)     # 버퍼 크기 감소
            ]
        )
        
        self.threshold = self.get_parameter('threshold').value
        self.alpha = self.get_parameter('alpha').value
        self.buffer_size = self.get_parameter('buffer_size').value

        # 이전 값들을 저장할 버퍼 초기화
        self.acc_x_buffer = deque(maxlen=self.buffer_size)
        self.acc_y_buffer = deque(maxlen=self.buffer_size)
        self.acc_z_buffer = deque(maxlen=self.buffer_size)
        self.yaw_buffer = deque(maxlen=self.buffer_size)
        
        # 필터링된 이전 값 저장
        self.prev_filtered = {
            'acc_x': 0.0,
            'acc_y': 0.0,
            'acc_z': 0.0,
            'yaw': 0.0
        }
        
        # Publisher & Subscriber QoS 설정 강화
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=100  # 버퍼 크기 증가
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            qos_profile)
        
        self.imu_pub = self.create_publisher(
            Imu,
            '/imu_filtered',
            qos_profile)
        
        self.get_logger().info('IMU Filter node has been started')

    def low_pass_filter(self, current_value, prev_filtered, buffer):
        """저주파 통과 필터 적용"""
        buffer.append(current_value)
        
        if len(buffer) < 2:
            return current_value
            
        # 최근 값들의 평균 계산
        recent_values = list(buffer)
        avg = np.mean(recent_values)
        
        # 급격한 변화 감지
        if abs(current_value - avg) > self.threshold:
            # 임계값을 넘는 변화 발생 시, 현재 값을 더 많이 반영ㄴ
            filtered = current_value * 0.7 + prev_filtered * 0.3
        else:
            # 정상적인 변화는 일반적인 필터링
            filtered = prev_filtered + self.alpha * (current_value - prev_filtered)
        
        return filtered

    def imu_callback(self, msg):
        # 원본 메시지 복사
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        
        # 가속도 필터링
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z
        
        filtered_msg.linear_acceleration.x = self.low_pass_filter(
            acc_x, self.prev_filtered['acc_x'], self.acc_x_buffer)
        filtered_msg.linear_acceleration.y = self.low_pass_filter(
            acc_y, self.prev_filtered['acc_y'], self.acc_y_buffer)
        filtered_msg.linear_acceleration.z = self.low_pass_filter(
            acc_z, self.prev_filtered['acc_z'], self.acc_z_buffer)
            
        # 각속도는 그대로 복사
        filtered_msg.angular_velocity = msg.angular_velocity
        
        # 쿼터니언 방향도 그대로 복사
        filtered_msg.orientation = msg.orientation
        
        # 필터링된 값을 이전 값으로 저장
        self.prev_filtered['acc_x'] = filtered_msg.linear_acceleration.x
        self.prev_filtered['acc_y'] = filtered_msg.linear_acceleration.y
        self.prev_filtered['acc_z'] = filtered_msg.linear_acceleration.z
        
        # 필터링된 메시지 발행
        self.imu_pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 