#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import pandas as pd
import matplotlib.pyplot as plt
import os

class PosePlotter(Node):
    def __init__(self):
        super().__init__('pose_plotter')
        self.data_rows = []
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.callback,
            10
        )
        self.message_count = 0
        self.get_logger().info('Listening for /amcl_pose messages...')

    def callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.data_rows.append({"t": t, "x": x, "y": y})
        self.message_count += 1
        if self.message_count % 100 == 0:
            self.get_logger().info(f'Processed {self.message_count} messages')
    
    def save_and_plot(self, output_dir):
        if not self.data_rows:
            self.get_logger().info('No data received')
            return
        
        # Save to CSV
        df = pd.DataFrame(self.data_rows)
        csv_path = os.path.join(output_dir, 'my_data.csv')
        df.to_csv(csv_path, index=False)
        self.get_logger().info(f'Wrote CSV: {csv_path}')

        # Plot trajectory
        plt.figure(figsize=(10, 6))
        plt.plot(df["x"], df["y"], 'bo-', label='Robot Path (amcl_pose)')
        waypoints = [(-6.25, 0.0), (0.0, 3.25), (6.25, 0.0), (0.0, 0.0)]
        for i, (wx, wy) in enumerate(waypoints, 1):
            for _, row in df.iterrows():
                if abs(row["x"] - wx) < 0.1 and abs(row["y"] - wy) < 0.1:
                    plt.text(row["x"], row["y"], f'WP{i}\n{row["t"]:.2f}s', fontsize=8, color='red')
                    break
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Robot Trajectory (from /amcl_pose)')
        plt.grid(True)
        plt.axis('equal')
        plt.legend()
        trajectory_path = os.path.join(output_dir, 'trajectory_xy.png')
        plt.savefig(trajectory_path, dpi=150)
        plt.show()
        self.get_logger().info(f'Saved plot: {trajectory_path}')

def main():
    rclpy.init()
    node = PosePlotter()
    output_dir = os.path.expanduser('~/autonomous-robot-logging-with-ROS2-jazzy/myros2_ws/my_experiment_bag_03')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'Processed {node.message_count} messages total')
        node.save_and_plot(output_dir)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()