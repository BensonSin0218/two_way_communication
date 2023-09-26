from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node, Client
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rclpy_message_converter import message_converter
from utils_package.utils import import_ROS_module

from .two_way_communication_event_handler import TwoWayCommunicationEventHandler
from threading import Thread
from socketio import AsyncServer, ASGIApp

import asyncio
import nest_asyncio
import uvicorn

nest_asyncio.apply()

class TwoWayCommunicationNode(Node):
    PUBLISHER: str = "network_to_ros"
    SUBSCRIBER: str = "ros_to_network"

    def __init__(self, node_name: str = "two_way_communication_node") -> None:
        super().__init__(node_name,
                         allow_undeclared_parameters=True)

        self.__topics: dict = {}
        self.__services: dict = {}
        self.__network_settings: dict = {}
        self.__sio: AsyncServer = None
        self.__app: ASGIApp = None
        self.__network_thread: Thread = None
        self.__operator: str = None

        self.__declare_parameters()
        self.__start_ros()
        self.__setup_socketIO_callback()
        self.__start_socketIO()

        self.get_logger().info(f"{node_name} initialized!")

    def __declare_parameters(self) -> None:
        topics_list: list[str] = self.declare_parameter(
            "topics_list",
            descriptor=ParameterDescriptor(type=Parameter.Type.STRING_ARRAY.value,
                                           dynamic_typing=True)).value

        if not topics_list is None and len(topics_list):
            # TODO: Add operator_only
            for topic in topics_list:
                prefix: str = f"topics.{topic}"
                topic_type: str = self.declare_parameter(
                    f"{prefix}.type",
                    descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value)).value
                topic_name: str = self.declare_parameter(
                    f"{prefix}.topic_name",
                    descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value)).value
                topic_message_type_name: str = self.declare_parameter(
                    f"{prefix}.message_type",
                    descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value)).value
                topic_message_type: any = import_ROS_module(topic_message_type_name,
                                                            self.get_logger())
                topic_qos: int = self.declare_parameter(
                    f"{prefix}.qos", 1,
                    descriptor=ParameterDescriptor(type=Parameter.Type.INTEGER.value)).value

                if not topic_type.lower() in [self.PUBLISHER, self.SUBSCRIBER]:
                    self.get_logger().error(f"Failed to setup {topic} as {topic_type}, excepted value in {[self.PUBLISHER, self.SUBSCRIBER]}")

                    continue

                elif not any([topic_type, topic_name, topic_message_type_name, topic_message_type]):
                    self.get_logger().error(f"Failed to setup {topic}, properly one of the arguments below is none!")
                    self.get_logger().error(f"type: {topic_type} | topic_name: {topic_name} | message_type_name: {topic_message_type_name} | message_type: {topic_message_type}")

                    continue

                self.__topics[topic]: dict = {
                    "type": topic_type,
                    "topic_name": topic_name,
                    "message_type": topic_message_type,
                    "qos": topic_qos
                }

        services_list: list[str] = self.declare_parameter(
            "services_list",
            descriptor=ParameterDescriptor(type=Parameter.Type.STRING_ARRAY.value,
                                           dynamic_typing=True)).value

        if not services_list is None and len(services_list):
            # TODO: Add operator_only
            for service in services_list:
                prefix:str = f"services.{service}"
                service_name: str = self.declare_parameter(
                    f"{prefix}.service_name",
                    descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value)).value
                srv_name: str = self.declare_parameter(
                    f"{prefix}.srv",
                    descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value)).value
                srv: any = import_ROS_module(srv_name,
                                             self.get_logger())

                if not any([service_name, srv_name, srv]):
                    self.get_logger().error(f"Failed to setup {service}, properly one of the arguments below is none!")
                    self.get_logger().error(f"service_name: {service_name} | srv_name: {srv_name} | srv: {srv}")

                    continue

                self.__services[service]: dict = {
                    "service_name": service_name,
                    "srv": srv
                }

        self.__network_settings["address"]: str = self.declare_parameter(
            "network.address", "0.0.0.0",
            descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value)).value
        self.__network_settings["port"]: int = self.declare_parameter(
            "network.port", 1234,
            descriptor=ParameterDescriptor(type=Parameter.Type.INTEGER.value)).value

        self.get_logger().info("Parameters:")
        self.get_logger().info("-> topics:")
        for topic_name, topic in self.__topics.items():
            self.get_logger().info(f"--> {topic_name}:")
            self.get_logger().info(f"---> type: {topic['type']}")
            self.get_logger().info(f"---> topic_name: {topic['topic_name']}")
            self.get_logger().info(f"---> message_type: {topic['message_type']}")
            self.get_logger().info(f"---> qos: {topic['qos']}")
        self.get_logger().info("-> services:")
        for service_name, service in self.__services.items():
            self.get_logger().info(f"--> {service_name}")
            self.get_logger().info(f"---> service_name: {service['service_name']}")
            self.get_logger().info(f"---> srv: {service['srv']}")
        self.get_logger().info("-> network:")
        self.get_logger().info(f"--> address: {self.__network_settings['address']}")
        self.get_logger().info(f"--> port: {self.__network_settings['port']}")

    def __start_ros(self) -> None:
        for topic, topic_info in self.__topics.items():
            match topic_info["type"]:
                case self.PUBLISHER:
                    self.__topics[topic][self.PUBLISHER] = self.create_publisher(
                        topic_info["message_type"],
                        topic_info["topic_name"],
                        topic_info["qos"])

                case self.SUBSCRIBER:
                    self.__topics[topic][self.SUBSCRIBER] = self.create_subscription(
                        topic_info["message_type"],
                        topic_info["topic_name"],
                        self.__get_msg_callback(topic),
                        topic_info["qos"])

    def __get_msg_callback(self, topic: str) -> callable:
        def __emit_data_to_socketIO(event: str, data: dict) -> None:
            async def __emit_data() -> None:
                await self.__sio.emit(event, data, room=event, namespace="/")

            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(__emit_data())
            loop.close()

        def __msg_callback(msg: any) -> None:
            self.get_logger().debug(f"{topic} get {msg}")

            __emit_data_to_socketIO(
                topic,
                message_converter.convert_ros_message_to_dictionary(msg))

        return __msg_callback

    def __setup_socketIO_callback(self) -> None:
        self.__sio = AsyncServer(async_mode="asgi", async_handlers=True)
        self.__app = ASGIApp(self.__sio)

        self.__sio.register_namespace(TwoWayCommunicationEventHandler(self, "/"))

    def __start_socketIO(self) -> None:
        self.__network_thread = Thread(target=self.__uvicorn_thread,
                                       daemon=True)
        self.__network_thread.start()

    def __uvicorn_thread(self) -> None:
        uvicorn.run(self.__app,
                    host=self.__network_settings["address"],
                    port=self.__network_settings["port"],
                    log_level="error")

    # Public functions
    def get_operator(self) -> str:
        return self.__operator

    def set_operator(self, sid: str) -> None:
        self.__operator = sid

    def publish_message_to_topic(self, topic: str, message: dict) -> None:
        if not topic in self.__topics:
            self.get_logger().error(
                f"{topic} is not in node topics! Ignoring this command!")
            return

        elif self.__topics[topic]["type"] != self.PUBLISHER:
            self.get_logger().error(
                f"{topic} is not from network to ros! Ignoring this command!")
            return

        try:
            msg: any = message_converter.convert_dictionary_to_ros_message(
                self.__topics[topic]["message_type"], message)
            self.__topics[topic][self.PUBLISHER].publish(msg)

        except Exception as e:
            self.get_logger().error(
                f"Failed to publish data to topic - {topic}! Exception:")
            self.get_logger().error(f"{e}")

    async def request_service(self, service: str, message: dict) -> dict:
        if not service in self.__services:
            self.get_logger().error(
                f"{service} is not in node services! Ignoring this command!")
            return {
                "status": 1,
                "error": f"{service} not found"
            }

        try:
            cb_group: ReentrantCallbackGroup = ReentrantCallbackGroup()
            service_client: Client = self.create_client(
                self.__services[service]["srv"],
                self.__services[service]["service_name"],
                callback_group=cb_group)

            if not service_client.wait_for_service(timeout_sec=0.1):
                self.destroy_client(service_client)

                self.get_logger().error(
                    f"{service} is not available! Ignoring this command!")

                return {
                    "status": 1,
                    "error": f"{service} is not available"
                }

            request: any = self.__services[service]["srv"].Request(**message)
            response: any = await service_client.call_async(request)

            return {
                "status": 0,
                "response": f"{response}"
            }

        except Exception as e:
            self.get_logger().error(
                f"Failed to request for service - {service}! Exception:")
            self.get_logger().error(f"{e}")

            return {
                "status": 1,
                "error": f"{e}"
            }
