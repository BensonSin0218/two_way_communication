import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import time

class TestingPublishNode(Node):
    def __init__(self) -> None:
        super().__init__("testing_publish_node")
        self.__publisher = self.create_publisher(
            String, "talker", 10)
        self.create_timer(
            0.5, self.__timer_callback)

    def __timer_callback(self) -> None:
        msg_time = f"{time.localtime().tm_sec}"

        self.get_logger().info(f"{msg_time}")
        self.__publisher.publish(String(data=f"{msg_time}"))


def main(args=None):
    rclpy.init(args=args)

    node = TestingPublishNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()