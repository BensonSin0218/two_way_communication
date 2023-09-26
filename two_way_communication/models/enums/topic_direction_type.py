from enum import Enum

class TopicDirectionType(Enum):
    PUBLISHER = 'network_to_ros'
    SUBSCRIBER = 'ros_to_network'