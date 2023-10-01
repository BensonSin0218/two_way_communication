two_way_communication
===

A `ROS2` package that subscribe transmit `ROS2` topic and service between network and `ROS2`

This package is developed based on `ROS2 Humble`, and `Socket-IO 5.9.0`

---

# How To Build

1. Git clone the repository to your `ROS2` workspace
    ```bash
    cd <your workspace>/src
    git clone https://github.com/BensonSin0218/two_way_communication.git
    ```

2. Build the package
    ```bash
    cd ../
    colcon build --packages-select two_way_communication
    ```

# How To Launch

1. Source `ROS2` and your workspace
    ```bash
    source /opt/ros/humble/setup.bash
    source <your workspace>/install/setup.bash
    ```

2. Launch the package
    ```bash
    ros2 launch two_way_communication two_way_communication_node.launch.py
    ```

    > If every thing goes right, you should see the following output (Or something similar to this based on your params):
    > ```bash
    > Parameters:
    > -> topics:
    > --> talker:
    > ---> type: ros_to_network
    > ---> topic_name: /talker
    > ---> message_type: <class 'std_msgs.msg._string.String'>
    > ---> qos: 1
    > --> listener:
    > ---> type: network_to_ros
    > ---> topic_name: /listener
    > ---> message_type: <class 'std_msgs.msg._string.String'>
    > ---> qos: 1
    > -> services:
    > --> testing_service
    > ---> service_name: /testing_service
    > ---> srv: <class 'example_interfaces.srv._add_two_ints.AddTwoInts'>
    > -> network:
    > --> address: 0.0.0.0
    > --> port: 1234
    > two_way_communication_node initialized!
    > ```

---

# Configuration

Once you build the package, you can change parameters in `config/params.yaml` to setup new streaming or change other settings
> Full path: `<your workspace>/install/two_way_communication/share/two_way_communication/config/params.yaml`

> Then you may use the `ros2 launch` command to re-launch the package

## Params

`topics_list`

- Name to enable transmit between network and `ROS2`, this name should be equal to the sections in `topics`

`topics`

- Section to set topic to transmit from `ROS2` to network, vice versa
- Format:
    ```yaml
    <name in topics_list>:
      type: ros_to_network  # Or network_to_ros
      topic_name: /talker
      message_type: std_msgs/msg/String
      qos: 1
    ```

`services_list`

- Name to enable transmit between network and `ROS2`, this name should be equal to the sections in `services`

`services`

- Section to set service to transmit from `ROS2` to network
- Format:
    ```yaml
    <name in services_list>:
      service_name: /testing_service
      srv: example_interfaces/srv/AddTwoInts
    ```

`network`

- Section to configure `network` settings
- Parameters:
    - `address`: Incoming request address, **default: 0.0.0.0**
    - `port`: Incoming request port, **default: 1234**
