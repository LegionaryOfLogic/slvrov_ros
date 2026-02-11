import rclpy
from rclpy.node import Node
from slvrov_interfaces.msg import PCA9685Command
import slvrov_tools

class TestPublisherNode(Node):
    def __init__(self):
        super().__init__("test_publisher_node")
        self.publisher = self.create_publisher(PCA9685Command, "pca9685_command", 10)
        self.timer = self.create_timer(1.0, self.publish_msg)

    def publish_msg(self):
        msg = PCA9685Command()
        msg.id = ["F" for _ in range(16)]   # Make sure the list length matches expectation
        msg.pwm = [1500 for _ in range(16)]
        self.publisher.publish(msg)
        self.get_logger().info("Published PCA9685Command")

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

