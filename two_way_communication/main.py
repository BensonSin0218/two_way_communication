import rclpy
from .two_way_communication_node import TwoWayCommunicationNode

def main(args=None) -> None:
    rclpy.init(args=args)

    node = TwoWayCommunicationNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()