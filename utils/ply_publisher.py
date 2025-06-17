#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

def convert_to_pointcloud2_with_color(points, colors, frame_id="map"):
    header = Header()
    header.stamp = rclpy.time.Time().to_msg()
    header.frame_id = frame_id
    
    # RGB 정보를 포함한 포인트 클라우드 생성
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
    ]
    
    # RGB를 uint32로 변환
    rgb_uint32 = ((colors[:, 0].astype(np.uint32) << 16) | 
                  (colors[:, 1].astype(np.uint32) << 8) | 
                  (colors[:, 2].astype(np.uint32)))
    
    # 포인트와 컬러 결합
    cloud_data = []
    for i in range(len(points)):
        cloud_data.append([points[i][0], points[i][1], points[i][2], rgb_uint32[i]])
    
    point_cloud_msg = pc2.create_cloud(header, fields, cloud_data)
    return point_cloud_msg

class ColoredPlyPublisher(Node):
    def __init__(self, ply_file):
        super().__init__('colored_ply_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'foundation_point_cloud', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.points, self.colors = self.load_ply_with_color(ply_file)

    def load_ply_with_color(self, ply_file):
        pcd = o3d.io.read_point_cloud(ply_file)
        points = np.asarray(pcd.points, dtype=np.float32)
        colors = np.asarray(pcd.colors, dtype=np.float32) * 255  # 0-1 -> 0-255
        return points, colors

    def timer_callback(self):
        msg = convert_to_pointcloud2_with_color(self.points, self.colors)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing colored point cloud')

def main(args=None):
    rclpy.init(args=args)
    publisher = ColoredPlyPublisher('/home/smarthc/FoundationStereo/test_outputs/compare_pcl/cloud.ply')
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()