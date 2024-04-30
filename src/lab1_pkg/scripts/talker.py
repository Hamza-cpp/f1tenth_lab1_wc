#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class Talker_Node(Node):
    """
    A class representing a talker node that publishes AckermannDriveStamped messages.

    This class inherits from the `Node` class and provides functionality to publish
    drive messages with specified speed and steering angle.

    Attributes:
        d_param (float): The value of the 'd' parameter.
        v_param (float): The value of the 'v' parameter.
        publisher_ (Publisher): The publisher object for publishing drive messages.
        timer_period (float): The period of the timer in seconds.
        timer_ (Timer): The timer object for periodically publishing drive messages.
    """

    def __init__(self):
        super().__init__("talker_node_by_s7our_squad")
        self.d_param = self.declare_parameter("d", rclpy.Parameter.Type.DOUBLE).value
        self.v_param = self.declare_parameter("v", rclpy.Parameter.Type.DOUBLE).value
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "drive", 10)
        self.timer_period = 0.001  # 1 milisecond
        self.timer_ = self.create_timer(self.timer_period, self.publish_drive)
        self.get_logger().info("Talker Node has been started, gog go go!")
        self.get_logger().info(
            f"Parameters received: v={self.v_param}, d={self.d_param}"
        )

    def publish_drive(self):
        """
        Publishes the drive message with the specified speed and steering angle.

        This method is called by the timer at regular intervals to publish the drive message.
        The speed and steering angle are obtained from the 'v_param' and 'd_param' attributes.
        If the parameters are not set, default values of 0.0 are used.
        """
        msg = AckermannDriveStamped()
        msg.drive.speed = self.v_param if self.v_param else 0.0
        msg.drive.steering_angle = self.d_param if self.d_param else 0.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    talker_node = Talker_Node()

    rclpy.spin(talker_node)

    talker_node.destroy(talker_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
