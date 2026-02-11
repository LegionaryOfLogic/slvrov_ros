# TODO import exceptions

import rclpy
from rclpy.node import Node

from slvrov_tools.pca9685 import PCA9685, PCA9685_Pin_Config, get_pca9685_pin_configs
from slvrov_interfaces.msg import PCA9685Command

from underwater_global_setup import I2C_BUS1, PIN_CONFIG_PATH  # TODO make this file
# TODO determine what file system/setup is going to be runningo n surface and underwater


class PCA9685Node(Node):
    def __init__(self):
        global PIN_CONFIG_PATH

        super().__init__("pca9685_node")

        self.pca9685 = "PCA9685(I2C_BUS1)"  # TODO fill in argument
        self.pin_configs = get_pca9685_pin_configs(PIN_CONFIG_PATH)

        self.pca9685_command_subscription = self.create_subscription(PCA9685Command, "pca9685_command", self.pca9685_command_callback, 10)

    def pca9685_command_callback(self, msg):
        self.get_logger().info(f"Pins: {msg.id}, PWMs: {msg.pwm}")

        pins = (PIN_CONFIGS[pin_id] for pin_id in msd.id)
        pwms = (pins[pin] for pin in msd.pwm)
        

def main(args=None):
    rclpy.init()
    node = PCA9685Node()
    rclpy.spin(node)
    rclpy.shutdown()