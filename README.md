# `two_way_communication`
===

A `ROS2` package that subscribe transmit `ROS2` topic and service between network outside and `ROS2`
> This requires [utils_package](https://github.com/BensonSin0218/utils_package)

---

# How To Build

1. Git clone the repository to your `ROS2` workspace
    ```bash
    cd <your ROS2 workspace>/src
    git clone https://github.com/BensonSin0218/two_way_communication.git
    ```

2. Install related packages
    ```bash
    python3 -m pip install -r requirements.txt
    sudo apt install ros-humble-rclpy-message-converter
    ```

2. Build the package
    ```bash
    cd ../
    colcon build --packages-select two_way_communication
    ```

# How To Launch

1. Source `ROS2` and your workspace
    ```bash
    source /opt/ros/humble/setup.zsh
    source <your workspace>/install/setup.zsh
    ```

2. Launch the package
    ```bash
    ros2 launch two_way_communication two_way_communication_node.launch.py
    ```

    > If every thing goes right, you should see the following output (Or something similar to this based on your params):
    > ```bash
    > [main-1] [INFO] [1720537860.057337568] [two_way_communication_node]: two_way_communication_node initialized!
    > ```

---

# Configuration

Once you build the package, you can change parameters in `config/params.yaml` to setup the package
> Full path: `<your workspace>/install/two_way_communication/share/two_way_communication/config/params.yaml`

> Then you may use the `ros2 launch` command to re-run the package

## Params

`topics`

- Name to enable transmit between network and `ROS2`, this name should be equal to the sections in `topics`

`topics_setting`

- Section to set each topics' setting, i.e. topic message type / to transmit from `ROS2` to network, vice versa
- Format:
    ```yaml
    <name in topics>:
      direction: ros_to_network  # Or network_to_ros
      topic_name: talker
      message_type: std_msgs/msg/String
    ```

`services`

- **Deprecated for now**

`services_setting`

- **Deprecated for now**

`network`

- Section to configure `network` settings
- Parameters:
    - `address`: Incoming request address, **default: 0.0.0.0**
    - `port`: Incoming request port, **default: 1234**
