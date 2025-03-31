import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2
import yaml
import math
import matplotlib.pyplot as plt

class Aligner(Node):
    def __init__(self):
        super().__init__('aligner')

        # Subscribe to the LaserScan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Load map
        self.map_image, self.map_resolution, self.map_origin = self.load_map("/home/avirupghosh/ros2_ws/src/icp_localization/icp_localization/map.yaml")
        if self.map_image is None:
            self.get_logger().error("Failed to load map image.")
            return

        # Extract occupied map pixels
        self.occupied_pixels, self.x_range, self.y_range = self.get_occupied_pixels()
        if self.occupied_pixels is None:
            self.get_logger().error("No occupied pixels found in map!")
            return

        self.get_logger().info(f"Map Occupied Range: X={self.x_range}, Y={self.y_range}")

    def load_map(self, yaml_file):
        """ Load the map image and metadata """
        with open(yaml_file, 'r') as f:
            map_metadata = yaml.safe_load(f)

        pgm_path = map_metadata['image']
        resolution = map_metadata['resolution']
        origin = map_metadata['origin']  # [x, y, theta]

        map_image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        return map_image, resolution, origin

    def get_occupied_pixels(self):
      
        occupied_pixels = np.argwhere(self.map_image < 180)  # Assume obstacles are dark

        if occupied_pixels.size == 0:
            return None, 0, 0

        # Convert pixel indices to real-world coordinates
        world_coords = []
        for y, x in occupied_pixels:
            world_x = x * self.map_resolution + self.map_origin[0]
            world_y = y * self.map_resolution + self.map_origin[1]
            world_coords.append([world_x, world_y])

        world_coords = np.array(world_coords)

        # Compute x_range and y_range
        x_range = np.ptp(world_coords[:, 0])  # Max - Min in X
        y_range = np.ptp(world_coords[:, 1])  # Max - Min in Y

        return world_coords, x_range, y_range

    def scan_callback(self, msg):
        """ Callback function for LaserScan data """
        scan_points = self.laserscan_to_points(msg)

        if len(scan_points) == 0:
            return

        scan_points = np.array(scan_points)  # Convert to NumPy array

        # Find the best shift
        best_shift, min_distance = self.find_best_shift(scan_points)
        
        self.get_logger().info(f"Optimal Shift: {best_shift}, Min Sum Distance: {min_distance}")

    def laserscan_to_points(self, scan_msg):
        """ Convert LaserScan to 2D point list """
        angle = scan_msg.angle_min
        points = []

        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, -y])
            angle += scan_msg.angle_increment

        return points

    def sum_min_distances(self, shifted_map_points, scan_points):
        """ Compute sum of min distances from map points to scan points """
        total_distance = 0
        for map_point in shifted_map_points:
            distances = np.linalg.norm(scan_points - map_point, axis=1)
            total_distance += np.min(distances)
        return total_distance
    

    def plot_alignment(self, scan_points, shifted_map_points):
        """ Plot scan points and shifted occupied map points """
        if len(scan_points) == 0 or len(shifted_map_points) == 0:
            self.get_logger().warn("No valid points to plot.")
            return

        # Convert to NumPy arrays
        scan_points = np.array(scan_points)
        shifted_map_points = np.array(shifted_map_points)

        # Get min/max for normalization
        min_x = min(scan_points[:, 0].min(), shifted_map_points[:, 0].min()) - 1
        max_x = max(scan_points[:, 0].max(), shifted_map_points[:, 0].max()) + 1
        min_y = min(scan_points[:, 1].min(), shifted_map_points[:, 1].min()) - 1
        max_y = max(scan_points[:, 1].max(), shifted_map_points[:, 1].max()) + 1

        # Create plot
        plt.figure(figsize=(8, 8))
        plt.scatter(shifted_map_points[:, 0], shifted_map_points[:, 1], color='red', alpha=0.3, label="Shifted Map Cells", s=5)
        plt.scatter(scan_points[:, 0], scan_points[:, 1], color='black', label="Scan Points", s=5)

        # Set axis limits and labels
        plt.xlim(min_x, max_x)
        plt.ylim(min_y, max_y)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        plt.title("Alignment Visualization")
        plt.grid(True)

        # Save and show the plot
        
        plt.show()
    #GARADIENT DESCENT BASED APPROACH
    # def find_best_shift(self, scan_points, learning_rate=0.1, max_iters=50, tolerance=1e-4):
    # """ Gradient Descent-based optimization for best shift """
        # dx, dy = 0, 0  # Start with no shift
        # prev_distance = float('inf')

        # for i in range(max_iters):
        #     # Compute cost at current shift
        #     shifted_map = self.occupied_pixels + [dx, dy]
        #     current_distance = self.sum_min_distances(shifted_map, scan_points)

        #     # Compute numerical gradients
        #     epsilon = 1e-3
        #     shifted_map_x = self.occupied_pixels + [dx + epsilon, dy]
        #     shifted_map_y = self.occupied_pixels + [dx, dy + epsilon]

        #     gradient_x = (self.sum_min_distances(shifted_map_x, scan_points) - current_distance) / epsilon
        #     gradient_y = (self.sum_min_distances(shifted_map_y, scan_points) - current_distance) / epsilon

        #     # Update dx, dy using gradients
        #     dx -= learning_rate * gradient_x
        #     dy -= learning_rate * gradient_y

        #     self.get_logger().info(f"Iteration {i+1}: dx={dx:.4f}, dy={dy:.4f}, Cost={current_distance:.4f}")

        #     # Check for convergence
        #     if abs(prev_distance - current_distance) < tolerance:
        #         break
        #     prev_distance = current_distance

        # # Final optimal shift
        # best_shift = (dx, dy)
        # self.get_logger().info(f"Optimal shift found: {best_shift}, Min Distance: {prev_distance:.4f}")

        # # Visualize result
        # shifted_map = self.occupied_pixels + np.array(best_shift)
        # self.plot_alignment(scan_points, shifted_map)

        # return best_shift, prev_distance
    #brute-force approach while trying to find minimum correlation between a scan-data and 
    #map-data morphed in real world coordinates
    def find_best_shift(self, scan_points):
        """ Brute-force search for best shift in map points """
        best_shift = (0, 0)
        min_distance = float('inf')
        
        for dx in np.linspace(-self.x_range, self.x_range, num=30): 
            for dy in np.linspace(-self.y_range, self.y_range, num=30):
                shifted_map = self.occupied_pixels + [dx, dy]
                distance = self.sum_min_distances(shifted_map, scan_points)

                if distance < min_distance:
                    min_distance = distance
                    best_shift = (dx, dy)
        shifted_map = self.occupied_pixels + np.array(best_shift)  
        self.get_logger().info(f"Optimal pos of robot in (gazebo) world coordinates: {best_shift}, Min Sum Distance: {min_distance}")
        self.plot_alignment(scan_points, shifted_map) 
        
        return best_shift, min_distance

def main():
    rclpy.init()
    node = Aligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


