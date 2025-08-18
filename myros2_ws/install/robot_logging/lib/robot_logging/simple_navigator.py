import math
import heapq
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

def a_star(grid, start, goal):
    # grid: 2D numpy array, 0 free, 1 occupied
    # start, goal: (x, y) tuples in grid indices
    if grid[start[1]][start[0]] == 1 or grid[goal[1]][goal[0]] == 1:
        return None

    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # 4 directions

    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []

    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            return data[::-1]  # reverse to start->goal

        close_set.add(current)

        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < grid.shape[1] and 0 <= neighbor[1] < grid.shape[0]:
                if grid[neighbor[1]][neighbor[0]] == 1 or neighbor in close_set:
                    continue

                tentative_g_score = gscore[current] + 1

                if tentative_g_score < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return None

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        self.map = None
        self.origin_x = None
        self.origin_y = None
        self.resolution = None
        self.current_pose = None  # (x, y, quaternion)
        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.subscription_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.targets = [(0.0, 0.0), (2.0, 2.0), (4.0, 0.0)]  # A, B, C in map coordinates - adjust as needed
        self.current_target_idx = 0
        self.path = []  # list of (wx, wy)
        self.wait_timer = None
        self.timer = self.create_timer(0.1, self.control_loop)

    def map_callback(self, msg):
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map = np.where(map_data < 0, 1, np.where(map_data > 0, 1, 0))  # unknown and occupied as 1, free as 0
        self.get_logger().info('Map received and processed.')

    def pose_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.current_pose = (pos.x, pos.y, ori)

    def control_loop(self):
        if self.map is None or self.current_pose is None:
            return

        if not self.path:
            target = self.targets[self.current_target_idx]
            start_grid = self.world_to_grid(self.current_pose[0], self.current_pose[1])
            goal_grid = self.world_to_grid(target[0], target[1])
            if start_grid is None or goal_grid is None:
                self.get_logger().error('Invalid start or goal position.')
                return
            path_grid = a_star(self.map, start_grid, goal_grid)
            if path_grid:
                self.path = [self.grid_to_world(gx, gy) for gx, gy in path_grid]
                self.get_logger().info(f'Path planned to target {target}')
            else:
                self.get_logger().error('No path found to target.')
                return

        if self.path:
            next_waypoint = self.path[0]
            dist = math.hypot(next_waypoint[0] - self.current_pose[0], next_waypoint[1] - self.current_pose[1])
            if dist < 0.1:  # waypoint tolerance
                self.path.pop(0)
                if not self.path:  # reached final target
                    self.stop_robot()
                    self.wait_timer = self.create_timer(5.0, self.next_target)
                    self.get_logger().info('Reached target, waiting 5 seconds.')
            else:
                # Simple proportional controller for heading
                angle_to_waypoint = math.atan2(next_waypoint[1] - self.current_pose[1], next_waypoint[0] - self.current_pose[0])
                q = self.current_pose[2]
                current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                angle_error = angle_to_waypoint - current_yaw
                angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi  # normalize to -pi to pi

                twist = Twist()
                if abs(angle_error) < 0.2:
                    twist.linear.x = 0.2  # forward speed
                twist.angular.z = 0.5 * angle_error  # proportional gain
                self.publisher_vel.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.publisher_vel.publish(twist)

    def next_target(self):
        self.wait_timer = None
        self.current_target_idx = (self.current_target_idx + 1) % len(self.targets)
        self.get_logger().info(f'Moving to next target: {self.targets[self.current_target_idx]}')

    def world_to_grid(self, wx, wy):
        if self.resolution == 0:
            return None
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        if 0 <= gx < self.map.shape[1] and 0 <= gy < self.map.shape[0]:
            return (gx, gy)
        return None

    def grid_to_world(self, gx, gy):
        wx = self.origin_x + (gx + 0.5) * self.resolution
        wy = self.origin_y + (gy + 0.5) * self.resolution
        return (wx, wy)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()