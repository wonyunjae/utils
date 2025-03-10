import numpy as np
import open3d as o3d
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
import os
from PIL import Image

class PointCloudToOccupancyGrid(Node):
    def __init__(self):
        super().__init__('pc_to_occupancy')
        
        # 입력/출력 경로 설정
        input_path = "warehouse/5.ply"
        self.output_dir = "/home/wynz/ros2_ws/src/map_converter/map"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # PLY 파일 로드
        pcd = o3d.io.read_point_cloud(input_path)
        points = np.asarray(pcd.points)
        
        # 2D로 투영하고 occupancy grid 생성
        resolution = 0.05  # 5cm per cell
        grid = self.points_to_occupancy(points, resolution)
        
        # 맵 저장
        self.save_map(grid, resolution)
        
        # occupancy grid 발행
        self.pub = self.create_publisher(OccupancyGrid, 'map', 10)
        self.publish_grid(grid, resolution)

    def points_to_occupancy(self, points, resolution):
        # 이전과 동일한 코드...
        pass

    def save_map(self, grid, resolution):
        # PGM 파일로 저장
        pgm_path = os.path.join(self.output_dir, "warehouse_map.pgm")
        image = Image.fromarray(grid.T.astype(np.uint8))
        image.save(pgm_path)
        
        # YAML 파일로 메타데이터 저장
        yaml_path = os.path.join(self.output_dir, "warehouse_map.yaml")
        with open(yaml_path, 'w') as f:
            f.write(f"image: {pgm_path}\n")
            f.write(f"resolution: {resolution}\n")
            f.write("origin: [0.0, 0.0, 0.0]\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.196\n")
            f.write("negate: 0\n")
        
        self.get_logger().info(f"Map saved to {self.output_dir}")

    def publish_grid(self, grid, resolution):
        # 이전과 동일한 코드...
        pass

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToOccupancyGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()