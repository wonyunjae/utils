#!/usr/bin/env python3

# navsat_transform.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 완전한 TF 트리 구성
        # map -> odom -> base_link -> [gps, imu_link, odom_mapping]
        
        # map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        
        # odom -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),
        
        # base_link -> gps
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_gps',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps']
        ),
        
        # base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),
        
        # odom -> odom_mapping (중요: 직접 연결)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_odom_mapping',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'odom_mapping']
        ),
        
        # odom_mapping -> gps (직접 연결)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_mapping_to_gps',
            arguments=['0', '0', '0', '0', '0', '0', 'odom_mapping', 'gps']
        ),
        
        # imu_link -> gps 변환
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_gps',
            arguments=['0', '0', '0', '0', '0', '0', 'imu_link', 'gps']
        ),
        
        # GLIM용 변환 노드
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_glim',
            output='screen',
            parameters=[{
                'frequency': 10.0,
                'yaw_offset': 0.0,
                'zero_altitude': True,
                'publish_filtered_gps': True,
                'use_odometry_yaw': True,
                'wait_for_datum': False,
                'datum': [0.0, 0.0, 0.0],
                'magnetic_declination_radians': 0.0,
                'broadcast_utm_transform': False,
                'publish_utm_odom': True,
                'use_local_cartesian': False,
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'base_link_frame_id': 'base_link',
                'gps_frame_id': 'gps',
                'imu_frame_id': 'imu_link'
            }],
            remappings=[
                ('/gps/fix', '/fix_mrp_2000'),
                ('/odometry/filtered', '/glim_ros/odom'),
                ('/odometry/gps', '/glim_utm')
            ]
        ),

        # LIO-SAM용 변환 노드
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_liosam',
            output='screen',
            parameters=[{
                'frequency': 10.0,
                'yaw_offset': 0.0,
                'zero_altitude': True,
                'publish_filtered_gps': True,
                'use_odometry_yaw': True,
                'wait_for_datum': False,
                'datum': [0.0, 0.0, 0.0],
                'magnetic_declination_radians': 0.0,
                'broadcast_utm_transform': False,
                'publish_utm_odom': True,
                'use_local_cartesian': False,
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'base_link_frame_id': 'base_link',
                'gps_frame_id': 'gps',
                'imu_frame_id': 'imu_link'
            }],
            remappings=[
                ('/gps/fix', '/fix_mrp_2000'),
                ('/odometry/filtered', '/lio_sam/mapping/odometry'),
                ('/odometry/gps', '/liosam_utm')
            ]
        ),

        # GPS를 UTM으로 변환하는 노드
        Node(
            package='utils',
            executable='gps_to_utm',
            name='gps_to_utm',
            output='screen'
        ),
        
        # Odometry를 PointStamped로 변환하는 노드 추가
        Node(
            package='utils',
            executable='odom_to_utm_point',
            name='odom_to_utm_point',
            output='screen'
        )
    ])