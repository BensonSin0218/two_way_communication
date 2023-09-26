from .models.two_way_communication_socket_node import TwoWayCommunicationSocketNode

import rclpy

def main(args=None) -> None:
    try:
        rclpy.init(args=args)

        node = TwoWayCommunicationSocketNode()

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
