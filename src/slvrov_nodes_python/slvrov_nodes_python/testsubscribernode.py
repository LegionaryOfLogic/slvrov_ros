import rclpy
from rclpy.node import Node
from slvrov_interfaces.msg import PCA9685Command


class TestSubscriberNode(Node):
    def __init__(self):
        super().__init__("test_subscriber_node")

        self.subscriber = self.create_subscription(PCA9685Command, "pca9685_command", self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f"ids: {msg.id}, pwm: {msg.pwm}")


def main(args=None):
    rclpy.init()
    node = TestSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()


