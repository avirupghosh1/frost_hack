import cv2
import numpy as np
import open3d as o3d
import yaml

# Load map parameters from YAML
with open("/home/avirupghosh/ros2_ws/src/icp_localization/icp_localization/map.yaml", "r") as f:
    map_data = yaml.safe_load(f)

resolution = map_data["resolution"]  # meters per pixel
origin = map_data["origin"]  # [x, y, theta]

# Load occupancy grid map (PGM)
image = cv2.imread("/home/avirupghosh/ros2_ws/src/icp_localization/icp_localization/map.pgm", cv2.IMREAD_GRAYSCALE)
height, width = image.shape

# Threshold to extract occupied areas
threshold = 250
occupied_pixels = np.argwhere(image < threshold)

# Convert pixels to real-world coordinates
points = []
for y, x in occupied_pixels:
    wx = origin[0] + (x * resolution)
    wy = origin[1] + ((height - y) * resolution)  # Flip y-axis
    points.append([wx, wy, 0.0])  # 2D map, so z=0

# Convert to Open3D point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array(points))

# Save as PCD file
o3d.io.write_point_cloud("map.pcd", pcd)
print("Saved 2D map as map.pcd!")
