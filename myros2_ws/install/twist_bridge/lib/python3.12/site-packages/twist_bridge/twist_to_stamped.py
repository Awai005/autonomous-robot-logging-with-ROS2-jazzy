import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class Bridge(Node):
    def __init__(self):
        super().__init__('twist_to_stamped')
        self.pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.sub = self.create_subscription(Twist, '/nav2/cmd_vel', self.cb, 10)

    def cb(self, msg: Twist):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'
        ts.twist = msg
        self.pub.publish(ts)

def main(args=None):
    rclpy.init(args=args)
    node = Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
