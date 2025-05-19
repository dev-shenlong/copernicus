import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistStampedToTwist(Node):
    def __init__(self):
        super().__init__('twist_stamped_to_twist')
        self.sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel_stamped',  # Input topic
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',  # Output topic (used by diffdrive plugin)
            10
        )

    def callback(self, msg: TwistStamped):
        twist_msg = Twist()
        twist_msg.linear = msg.twist.linear
        twist_msg.angular = msg.twist.angular
        self.pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistStampedToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
