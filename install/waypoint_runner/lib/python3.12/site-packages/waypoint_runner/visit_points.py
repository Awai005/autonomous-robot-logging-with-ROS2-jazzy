import math
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from web3 import Web3

def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class WaypointRunner(Node):
    def __init__(self):
        super().__init__('waypoint_runner')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Edit your waypoints here (map frame):
        # (x, y, yaw_radians)
        self.waypoints = [
            (-6.25, 0.00, 0.0),   # A
            ( 0.00, 3.25, 1.57),  # B
            ( 6.25, 0.00, 3.14),  # C
            (0.0, 0.0, 0.0),   # Back to origin
        ]

        self.pause_sec = 1.0  # dwell 1s at each waypoint

        # Blockchain setup (Ganache)
        self.web3 = Web3(Web3.HTTPProvider('http://127.0.0.1:8545'))  # Use 7545 for Ganache GUI
        if not self.web3.is_connected():
            self.get_logger().error("Failed to connect to Ganache. Ensure Ganache is running.")
            rclpy.shutdown()
        self.account = self.web3.eth.accounts[0]
        self.web3.eth.default_account = self.account



    def make_goal(self, x, y, yaw) -> NavigateToPose.Goal:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quat(float(yaw))

        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal
    
    
    
    def log_data(self, position, message):
        data_str = f"Timestamp: {int(time.time())}, Position: {position}, Message: {message}"
        txn = {
            'to': self.account,
            'value': 0,
            'gas': 2000000,
            'gasPrice': self.web3.to_wei('1', 'gwei'),
            'nonce': self.web3.eth.get_transaction_count(self.account),
            'data': data_str.encode()
        }
        try:
            tx_hash = self.web3.eth.send_transaction(txn)
            self.get_logger().info(f"Transaction hash: {self.web3.to_hex(tx_hash)}")
            return tx_hash
        except Exception as e:
            self.get_logger().error(f"Failed to send transaction: {e}")
            return None

    def view_transaction(self, tx_hash):
        try:
            tx = self.web3.eth.get_transaction(tx_hash)
            if tx:
                data = tx['input'].decode('utf-8') if isinstance(tx['input'], bytes) else self.web3.to_text(tx['input'])
                self.get_logger().info(f"Transaction {self.web3.to_hex(tx_hash)} details:")
                self.get_logger().info(f"  From: {tx['from']}")
                self.get_logger().info(f"  To: {tx['to']}")
                self.get_logger().info(f"  Block: {tx['blockNumber']}")
                self.get_logger().info(f"  Data: {data}")
            else:
                self.get_logger().info(f"Transaction {self.web3.to_hex(tx_hash)} not found.")
        except Exception as e:
            self.get_logger().error(f"Error retrieving transaction: {e}")

    def run(self):
        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('No /navigate_to_pose action server. Is Nav2 up?')
            return

        for i, (x, y, yaw) in enumerate(self.waypoints, 1):
            goal = self.make_goal(x, y, yaw)
            self.get_logger().info(f'[{i}/{len(self.waypoints)}] Going to ({x:.2f},{y:.2f}) yaw={yaw:.2f}')
            send_future = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal was rejected by the server.')
                return

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()
            if result.status != 4:  # 4 == SUCCEEDED
                self.get_logger().warn(f'Goal finished with status={result.status} (not SUCCEEDED).')
            else:
                self.get_logger().info('Arrived. Resting...')
                time.sleep(self.pause_sec)
                position = f"x:{x:.2f} y:{y:.2f} z:0.00"
                message = f"Reached waypoint {i}"
                tx_hash = self.log_data(position, message)
                if tx_hash:
                    self.view_transaction(tx_hash)
                    
        self.get_logger().info('All waypoints complete ✅')

def main():
    rclpy.init()
    node = WaypointRunner()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
