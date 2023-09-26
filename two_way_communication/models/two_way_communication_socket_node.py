from .enums.topic_direction_type import TopicDirectionType
from .topic_info import TopicInfo
from asyncio.events import AbstractEventLoop
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rclpy_message_converter import message_converter
from socketio import ASGIApp, AsyncServer
from threading import Thread
from utils_package.utils import get_hostname_as_namespace, import_ROS_module

import asyncio
import nest_asyncio
import uvicorn

nest_asyncio.apply()

class TwoWayCommunicationSocketNode(Node):
    def __init__(self, node_name: str = "two_way_communication_socket_node") -> None:
        super().__init__(
            namespace=get_hostname_as_namespace(),
            node_name=node_name,
            allow_undeclared_parameters=True)

        self.__logger: RcutilsLogger = self.get_logger()
        self.__topics: dict[TopicInfo] = {}

        self.__sio_thread: Thread = None
        self.__sio: AsyncServer = None
        self.__sio_app: ASGIApp = None


        # Only the first connected will be the operator
        # Also, it should be unique
        self.__connections: list[str] = []

        self.__declare_ros_parameters()

        self.__setup_sio()

        self.__start_sio_thread()
        self.__setup_ros()

        self.__logger.info(f"{node_name} initialized!")

    def __def__(self) -> None:
        if not self.__sio_thread is None:
            self.__sio_thread.join()

    def __declare_ros_parameters(self) -> None:
        ''' Declare ROS topic related parameters '''
        topics_list: list[str] = self.declare_parameter(
            "topics",
            descriptor=ParameterDescriptor(type=Parameter.Type.STRING_ARRAY.value,
                                           dynamic_typing=True)).value

        if not topics_list is None and len(topics_list):
            # TODO: Add operator_only
            for topic in topics_list:
                prefix: str = f"topics_setting.{topic}"
                topic_direction: str = self.declare_parameter(
                    f"{prefix}.direction",
                    descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value)).value
                topic_name: str = self.declare_parameter(
                    f"{prefix}.topic_name",
                    descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value)).value
                topic_message_type_name: str = self.declare_parameter(
                    f"{prefix}.message_type",
                    descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value)).value
                topic_qos: int = self.declare_parameter(
                    f"{prefix}.qos", 1,
                    descriptor=ParameterDescriptor(type=Parameter.Type.INTEGER.value)).value

                topic_message_type: any = import_ROS_module(topic_message_type_name,
                                                            self.__logger)

                if not topic_direction.lower() in [type.value.lower() for type in TopicDirectionType]:
                    self.__logger.error(f"Failed to setup {topic} as {topic_direction}, excepted value in {[type.value.lower() for type in TopicDirectionType]}")

                    continue

                elif not any([topic_direction, topic_name,
                              topic_message_type_name, topic_message_type]):
                    self.__logger.error(f"Failed to setup {topic}, properly one of the arguments below is none!")
                    self.__logger.error(f"type: {topic_direction} | topic_name: {topic_name} | message_type_name: {topic_message_type_name} | message_type: {topic_message_type}")

                    continue

                self.__topics[topic] = TopicInfo(
                    topic_name, TopicDirectionType(topic_direction), topic_message_type, topic_qos)

    def __setup_sio(self) -> None:
        from .two_way_communication_event_handler import TwoWayCommunicationEventHandler

        self.__sio = AsyncServer(async_mode='asgi', async_handlers=True)
        self.__sio_app = ASGIApp(self.__sio)

        self.__sio.register_namespace(TwoWayCommunicationEventHandler(self, "/"))

    def __setup_ros(self) -> None:
        for topic, topic_info in self.__topics.items():
            self.__logger.info(f"Setting up {topic} with direction: {topic_info.direction}")
            match topic_info.direction:
                case TopicDirectionType.PUBLISHER:
                    self.__topics[topic].publisher = self.create_publisher(
                        topic_info.message_type,
                        topic_info.topic,
                        topic_info.qos)

                    self.__logger.info(f"Created publisher for {topic} with topic name: {topic_info.topic}")

                case TopicDirectionType.SUBSCRIBER:
                    self.__topics[topic].subscriber = self.create_subscription(
                        topic_info.message_type,
                        topic_info.topic,
                        self.__get_msg_callback(topic_info.topic),
                        topic_info.qos)

                    self.__logger.info(f"Created subscriber for {topic} with topic name: {topic_info.topic}")

                case _:
                    self.__logger.error(f"Failed to setup {topic} with unknown direction: {topic_info.direction}!")

    def __start_sio_thread(self) -> None:
        host: str =  self.declare_parameter(
            "network.host", "0.0.0.0",
            descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value)).value
        port: int = self.declare_parameter(
            "network.port", 1234,
            descriptor=ParameterDescriptor(type=Parameter.Type.INTEGER.value)).value

        self.__sio_thread = Thread(target=lambda: uvicorn.run(self.__sio_app, host=host, port=port, log_level='error'), daemon=True)
        self.__sio_thread.start()

    def __get_msg_callback(self, topic: str) -> callable:
        loop: AbstractEventLoop = asyncio.get_event_loop()

        def __emit_data_callback(data: dict) -> None:
            async def __emit_callback() -> None:
                await self.__sio.emit(topic, data, room=topic, namespace="/")

            asyncio.set_event_loop(loop)
            loop.run_until_complete(__emit_callback())

        def __msg_callback(msg: any) -> None:
            self.__logger.debug(f"{topic}: {msg}")

            __emit_data_callback(message_converter.convert_ros_message_to_dictionary(msg))

        return __msg_callback

    def add_connection(self, sid: str) -> None:
        if  sid in self.__connections:
            self.__logger.warn(f"{sid} is already connected!")

            return

        self.__connections.append(sid)

    def remove_connection(self, sid: str) -> None:
        if not sid in self.__connections:
            self.__logger.warn(f"{sid} is not connected!")

            return

        self.__connections.remove(sid)

    def get_connections(self) -> list[str]:
        return self.__connections

    def get_operator(self) -> str | None:
        return self.__connections[0] if len(self.__connections) else None

    def get_publisher_topics(self) -> list[str]:
        return [topic_name for topic_name in self.__topics.keys() if self.__topics[topic_name].direction == TopicDirectionType.PUBLISHER.value]

    def publish_message_to_topic(self, topic: str, message: dict) -> None:
        if (not topic in self.__topics) or \
            (self.__topics[topic].direction != TopicDirectionType.PUBLISHER):
            self.__logger.error(
                f"{topic} is neither in node topics nor from network to ros! Ignoring this message!")

            return

        try:
            msg: any = message_converter.convert_dictionary_to_ros_message(
                self.__topics[topic].message_type, message)
            self.__topics[topic].publisher.publish(msg)

        except Exception as e:
            self.__logger.error(
                f"Failed to publish data to topic - {topic}! Exception:")
            self.__logger.error(f"{e}")
