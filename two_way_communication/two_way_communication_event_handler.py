from rclpy.node import Node

from socketio import AsyncNamespace

class TwoWayCommunicationEventHandler(AsyncNamespace):
    def __init__(self, node: Node, namespace: str) -> None:
        super().__init__(namespace)

        self.__node = node

    def on_connect(self, sid: str, _: dict) -> None:
        self.__node.get_logger().info(f"{sid} is connected!")

        if not self.__node.get_operator() is None:
            self.__node.get_logger().warn(
                f"Already have operator! {sid} will not have the control!")
            return

        self.__node.set_operator(sid)

        self.__node.get_logger().info(f"{sid} is the operator now!")

    def on_disconnect(self, sid: str) -> None:
        self.__node.get_logger().info(f"{sid} is disconnected!")

        if not self.__node.get_operator() == sid:
            return

        self.__node.set_operator(None)

        self.__node.get_logger().warn(
            f"Operator is now disconnected! No operator now!")

    def on_publish(self, sid:str, data: dict) -> None:
        self.__node.get_logger().debug(
            f"Event: publisher | sid: {sid} | data: {data}")

        if not "topic" in data or not "message" in data:
            self.__node.get_logger().error(
                f"Expected to get topic and message in data! Got {data}! Ignoring this command!")
            return

        self.__node.publish_message_to_topic(data["topic"], data["message"])

    def on_subscribe(self, sid: str, data: dict) -> None:
        self.__node.get_logger().debug(
            f"Event: subscribe | sid: {sid} | data: {data}")

        if not "topic" in data:
            self.__node.get_logger().error(
                f"Expected to get topic in data! Got {data}! Ignoring this command!")
            return

        elif data["topic"] in self.rooms(sid):
            self.__node.get_logger().warn(
                f"Already subscribed to {data['topic']}! Ignoring this command!")
            return

        self.enter_room(sid, data["topic"])

        self.__node.get_logger().info(f"{sid} is now subscribed to {data['topic']}")

    def on_unsubscribe(self, sid: str, data: dict) -> None:
        self.__node.get_logger().debug(
            f"Event: unsubscribe | sid: {sid} | data: {data}")

        if not "topic" in data:
            self.__node.get_logger().error(
                f"Expected to get topic in data! Got {data}! Ignoring this command!")
            return

        elif not data["topic"] in self.rooms(sid):
            self.__node.get_logger().warn(
                f"{sid} does not subscribe to {data['topic']}! Ignoring this command!")
            return

        self.leave_room(sid, data["topic"])

        self.__node.get_logger().info(f"{sid} is now unsubscribed from {data['topic']}")

    async def on_service(self, sid: str, data: dict) -> None:
        self.__node.get_logger().debug(
            f"Event: service | sid: {sid} | data: {data}")

        if not "service" in data or not "request" in data:
            self.__node.get_logger().error(
                f"Expected to get service and request in data! Got {data}! Ignoring this command!")
            return

        return await self.__node.request_service(data["service"], data["request"])

