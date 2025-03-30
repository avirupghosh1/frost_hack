import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import open3d as o3d
import math

class ICPInitialPose(Node):
    def __init__(self):
        super().__init__('icp_initial_pose')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Load the known map as a point cloud (change the path if needed)
        self.map_cloud = o3d.io.read_point_cloud("/home/user/map.pcd")

    def scan_callback(self, msg):
        # Convert LaserScan to PointCloud
        scan_cloud = self.laserscan_to_pointcloud(msg)

        # Perform ICP scan matching
        icp_result = o3d.pipelines.registration.registration_icp(
            scan_cloud, self.map_cloud, 0.05, np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        # Extract estimated pose
        transform = icp_result.transformation
        x, y = transform[0, 3], transform[1, 3]
        theta = math.atan2(transform[1, 0], transform[0, 0])

        # Publish estimated initial pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.orientation.z = math.sin(theta / 2)
        pose_msg.pose.pose.orientation.w = math.cos(theta / 2)
        self.pub.publish(pose_msg)

        self.get_logger().info(f"Published initial pose: x={x}, y={y}, theta={theta}")

    def laserscan_to_pointcloud(self, scan_msg):
        """ Convert LaserScan to Open3D PointCloud """
        angle = scan_msg.angle_min
        points = []
        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y, 0])
            angle += scan_msg.angle_increment

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(np.array(points))
        return pc

def main():
    rclpy.init()
    node = ICPInitialPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
