import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

import time

class TestingServiceNode(Node):
    def __init__(self) -> None:
        super().__init__("testing_service_node")

        self.create_service(
            AddTwoInts, "testing_service",
            self.service_callback)

    def service_callback(self, req, res) -> any:
        time.sleep(5)

        res.sum = req.a + req.b

        return res


def main(args=None):
    rclpy.init(args=args)

    node = TestingServiceNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()