#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class Relay_Node(Node):
    """
    A class representing a relay node that receives drive commands and relays them with modified values.

    This class inherits from the `Node` class and provides functionality to subscribe to a topic for drive commands,
    modify the received commands, and publish the modified commands to another topic.

    Attributes:
        subscriber_: A subscription object for receiving drive commands.
        publisher_: A publisher object for publishing modified drive commands.
    """

    def __init__(self):
        super().__init__("relay_node_by_s7our_squad")
        self.subscriber_ = self.create_subscription(
            AckermannDriveStamped, "drive", self.drive_callback, 10
        )
        self.publisher_ = self.create_publisher(
            AckermannDriveStamped, "drive_relay", 10
        )
        self.get_logger().info("Relay Node has been started, gog go go!")

    def drive_callback(self, msg):
        """
        Callback function for processing received drive commands.

        This function is called whenever a new drive command is received. It modifies the received command by
        multiplying the speed and steering angle by 3, and publishes the modified command to the "drive_relay" topic.

        Args:
            msg: The received drive command message.
        """
        new_msg = AckermannDriveStamped()
        new_msg.drive.speed = msg.drive.speed * 3
        new_msg.drive.steering_angle = msg.drive.steering_angle * 3
        self.publisher_.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)

    relay_node = Relay_Node()

    rclpy.spin(relay_node)

    relay_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
