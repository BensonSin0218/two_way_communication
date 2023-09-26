from .enums.topic_direction_type import TopicDirectionType
from rclpy.node import Publisher, Subscription

class TopicInfo:
    def __init__(self, topic: str, direction: TopicDirectionType, message_type: any, qos: int) -> None:
        self.topic: str = topic
        self.direction: TopicDirectionType = direction
        self.message_type: any = message_type
        self.qos: int = qos

        self.publisher: Publisher = None
        self.subscriber: Subscription = None