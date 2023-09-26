from .two_way_communication_socket_node import TwoWayCommunicationSocketNode
from rclpy.impl.rcutils_logger import RcutilsLogger
from socketio import AsyncNamespace

import time

class TwoWayCommunicationEventHandler(AsyncNamespace):
    """ Event handler for two way communication socket"""
    def __init__(self, node: TwoWayCommunicationSocketNode, namespace: str) -> None:
        super().__init__(namespace)

        self.__node: TwoWayCommunicationSocketNode = node
        self.__logger: RcutilsLogger = self.__node.get_logger()

    async def on_connect(self, sid: str, _: dict) -> None:
        self.__logger.info(f"{sid} is connected!")

        self.__node.add_connection(sid)
        operator_sid: str | None = self.__node.get_operator()
        if sid == operator_sid:
            self.__logger.info(f"{sid} is the operator now!")

        await self.emit("connection/info", {
            "operator_sid": operator_sid,
            "connections_count": len(self.__node.get_connections()),
            "timestamp": time.time()
        })

    def on_disconnect(self, sid: str) -> None:
        self.__logger.info(f"{sid} is disconnected!")

        self.__node.remove_connection(sid)
        operator_sid: str | None = self.__node.get_operator()
        if operator_sid is None:
            self.__logger.info("No operator now!")
        else:
            self.__logger.info(f"{operator_sid} is the operator now!")

    def on_publish(self, sid:str, data: dict) -> None:
        self.__logger.debug(
            f"Event: publisher | sid: {sid} | data: {data}")

        if not "topic" in data or not "message" in data or not "timestamp" in data:
            self.__logger.error(
                f"Expected to get neither topic, message nor timestamp in data! Got {data}! Ignoring this command!")

            return

        if abs(data["timestamp"] - time.time()) > 1.0:
            self.__logger.error(
                f"message timestamp difference is greater that 1.0! Ignoring this message!")

            return

        self.__node.publish_message_to_topic(data["topic"], data["message"])

    async def on_subscribe(self, sid: str, data: dict) -> None:
        self.__logger.debug(
            f"Event: subscribe | sid: {sid} | data: {data}")

        if not "topic" in data:
            self.__logger.error(
                f"Expected to get topic in data! Got {data}! Ignoring this command!")
            return

        elif data["topic"] in self.rooms(sid):
            self.__logger.warn(
                f"Already subscribed to {data['topic']}! Ignoring this command!")
            return

        await self.enter_room(sid, data["topic"])

        self.__logger.info(f"{sid} is now subscribed to {data['topic']}")

    def on_unsubscribe(self, sid: str, data: dict) -> None:
        self.__logger.debug(
            f"Event: unsubscribe | sid: {sid} | data: {data}")

        if not "topic" in data:
            self.__logger.error(
                f"Expected to get topic in data! Got {data}! Ignoring this command!")
            return

        elif not data["topic"] in self.rooms(sid):
            self.__logger.warn(
                f"{sid} does not subscribe to {data['topic']}! Ignoring this command!")
            return

        self.leave_room(sid, data["topic"])

        self.__logger.info(f"{sid} is now unsubscribed from {data['topic']}")
