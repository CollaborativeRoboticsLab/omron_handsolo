import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocityFilter(Node):
    def __init__(self):
        super().__init__('velocity_filter')
        self.pub = self.create_publisher(Twist, '/amr/cmd_vel', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)

    def cb(self, msg: Twist):
        out = Twist()
        # Zero translational components
        out.linear.x = 0.0
        out.linear.y = 0.0
        out.linear.z = 0.0
        # Keep rotational components
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z
        self.pub.publish(out)


def main():
    rclpy.init()
    node = VelocityFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
