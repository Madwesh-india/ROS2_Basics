# Nodes.md

## Overview of Nodes and `rclpy` in ROS 2

This document provides an in-depth overview of nodes in ROS 2 using the `rclpy` Python library. Nodes are fundamental building blocks in ROS 2, representing individual processes that perform computation. This guide covers the `Node` class and its methods, providing references to specific topics like publishers, subscribers, and custom node implementations.

### Table of Contents

- [Introduction to Nodes](#introduction-to-nodes)
- [The `Node` Class in ROS 2](#the-node-class-in-ros-2)
- [Core Methods of the `Node` Class](#core-methods-of-the-node-class)
- [Creating a Custom Node](#creating-a-custom-node)
- [Redirects to Important Topics](#redirects-to-important-topics)
- [Advanced Node Concepts](#advanced-node-concepts)
- [Further Reading](#further-reading)

## Introduction to Nodes

In ROS 2, a **node** is an executable that uses ROS 2 to communicate with other nodes. Nodes can publish or subscribe to topics, provide or use services, and engage in other ROS 2 communication patterns. This modular design makes it easier to develop complex robotic systems in a flexible and reusable way.

## The `Node` Class in ROS 2

The `Node` class in `rclpy` provides a convenient interface for creating ROS 2 nodes in Python. When you create a new node, you define its behavior by extending the `Node` class and utilizing its methods to interact with the ROS 2 environment.

### Basic Node Creation

To create a basic ROS 2 node in Python, you start by importing the necessary libraries and extending the `Node` class:

```python
import rclpy
from rclpy.node import Node

def main(args=args):
  mynode = Node("my_node")
  #add subscribers, publishers etc
  rclpy.spin(mynode)
  rclpy.shutdown()
```

## Core Methods of the `Node` Class

The `Node` class in ROS 2 provides a variety of methods to handle different functionalities. Below are some of the core methods:

### 1. **Node Initialization and Shutdown**

- **`__init__(self, name: str, *, namespace: str = '')`**: Constructor method to initialize a new node with a given name.
  
- **`destroy_node(self)`**: Cleanly destroy the node and release resources.

### 2. **Logging**

- **`get_logger(self) -> Logger`**: Returns a logger instance that you can use to log messages. 

    ```python
    self.get_logger().info('This is an informational message.')
    ```
> **For detailed examples on services and clients, see the [Logging Guide](python/Logging.md).**

### 3. **Publishing and Subscribing to Topics**

- **`create_publisher(self, msg_type, topic_name, qos_profile)`**: Creates a publisher to a specific topic.
  
- **`create_subscription(self, msg_type, topic_name, callback, qos_profile)`**: Creates a subscription to a specific topic and assigns a callback function.

> **For detailed examples on publishers and subscribers, see the [Publisher and Subscriber Guide](python/Subscriber-Publisher.md).**

### 4. **Creating Timers**

- **`create_timer(self, timer_period_sec: float, callback)`**: Creates a timer that triggers a callback function at a specified interval.

    ```python
    self.create_timer(1.0, self.timer_callback)  # Triggers every second
    ```
> **For detailed examples on services and clients, see the [Timers Guide](python/Timers.md).**

### 5. **Parameter Handling**

- **`declare_parameter(self, name: str, value: ParameterValue)`**: Declares a parameter for the node.
  
- **`get_parameter(self, name: str)`**: Retrieves the value of a parameter.
  
- **`set_parameters(self, parameters: List[Parameter])`**: Sets multiple parameters for the node.

### 6. **Service and Client Creation**

- **`create_service(self, srv_type, srv_name, callback)`**: Creates a service with a specified type and callback function.
  
- **`create_client(self, srv_type, srv_name)`**: Creates a client that can request a service.

> **For detailed examples on services and clients, see the [Service and Client Guide](python/Service-Client.md).**

### 7. **Callbacks**

- **`add_callback(self, callback)`**: Adds a callback function to be called at a specific time or event.

### 8. **Node Utilities**
1. **Getting Node Names and Namespaces**

   - **`get_fully_qualified_name(self) -> str`**: Returns the fully qualified name of the node (includes the namespace).

     ```python
     fully_qualified_name = self.get_fully_qualified_name()
     self.get_logger().info(f'Node fully qualified name: {fully_qualified_name}')
     ```

2. **Working with Clocks and Time**

   - **`get_clock(self) -> Clock`**: Returns the clock associated with the node. This can be used to get the current time or to manage time-based operations.

     ```python
     clock = self.get_clock()
     current_time = clock.now()
     self.get_logger().info(f'Current time: {current_time.to_msg()}')
     ```

   - **`now(self) -> Time`**: A convenience method to get the current time using the node's default clock.

     ```python
     current_time = self.now()
     self.get_logger().info(f'Current time: {current_time.to_msg()}')
     ```

3. **Node Graph and Discovery Utilities**

   - **`get_node_names(self) -> List[str]`**: Returns a list of all node names in the ROS graph.

     ```python
     node_names = self.get_node_names()
     self.get_logger().info(f'Available nodes: {node_names}')
     ```

   - **`get_node_names_and_namespaces(self) -> List[Tuple[str, str]]`**: Returns a list of all node names and their namespaces.

     ```python
     nodes_and_namespaces = self.get_node_names_and_namespaces()
     self.get_logger().info(f'Nodes and namespaces: {nodes_and_namespaces}')
     ```

4. **Node Options and Context**

   - **`get_node_options(self) -> NodeOptions`**: Returns the options used to create the node, such as the name, namespace, and parameters.

     ```python
     node_options = self.get_node_options()
     self.get_logger().info(f'Node options: {node_options}')
     ```

   - **`get_context(self) -> Context`**: Returns the context associated with the node. The context represents the environment in which the node operates, including the global state and ROS middleware settings.

     ```python
     context = self.get_context()
     self.get_logger().info(f'Node context: {context}')
     ```

5. **Handling Parameters and Parameter Events**

   - **`get_parameter_or(self, name: str, alternative_value: ParameterValue) -> ParameterValue`**: Gets a parameter value or returns an alternative value if the parameter is not set.

     ```python
     my_param = self.get_parameter_or('my_param', ParameterValue(type_=ParameterType.PARAMETER_INTEGER, integer_value=0))
     self.get_logger().info(f'Parameter value: {my_param.value}')
     ```

   - **`add_on_set_parameters_callback(self, callback: Callable[[List[Parameter]], SetParametersResult])`**: Registers a callback function that is called whenever parameters are set on the node.

     ```python
     def parameters_callback(parameters):
         for param in parameters:
             self.get_logger().info(f'Parameter set: {param.name} = {param.value}')
         return SetParametersResult(successful=True)

     self.add_on_set_parameters_callback(parameters_callback)
     ```

6. **Handling Subscription and Publisher Count**

   - **`count_publishers(self, topic_name: str) -> int`**: Returns the number of publishers currently publishing to a specific topic.

     ```python
     publisher_count = self.count_publishers('topic_name')
     self.get_logger().info(f'Number of publishers: {publisher_count}')
     ```

   - **`count_subscribers(self, topic_name: str) -> int`**: Returns the number of subscribers currently subscribed to a specific topic.

     ```python
     subscriber_count = self.count_subscribers('topic_name')
     self.get_logger().info(f'Number of subscribers: {subscriber_count}')
     ```

7. **QoS (Quality of Service) Utilities**

   - **`get_publisher_qos(self, topic_name: str) -> QoSProfile`**: Returns the Quality of Service (QoS) settings for a specific publisher topic.

     ```python
     qos_profile = self.get_publisher_qos('topic_name')
     self.get_logger().info(f'Publisher QoS: {qos_profile}')
     ```

   - **`get_subscription_qos(self, topic_name: str) -> QoSProfile`**: Returns the QoS settings for a specific subscriber topic.

     ```python
     qos_profile = self.get_subscription_qos('topic_name')
     self.get_logger().info(f'Subscriber QoS: {qos_profile}')
     ```

8. **Node State and Lifecycle Management**

   - **`get_lifecycle_state(self)`**: Returns the current state of a lifecycle node (if the node is a lifecycle node).

     ```python
     lifecycle_state = self.get_lifecycle_state()
     self.get_logger().info(f'Node lifecycle state: {lifecycle_state}')
     ```

## Creating a Custom Node

To create a custom node, extend the `Node` class and utilize its methods to implement the desired functionality. Here's an example of a custom node that publishes messages and logs information:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CustomNode(Node):
    def __init__(self):
        super().__init__('custom_node')
        self.publisher_ = self.create_publisher(String, 'custom_topic', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = CustomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Redirects to Important Topics

For detailed information on specific topics, please refer to the following guides:

- **Publisher and Subscriber Guide**: Learn how to create publishers and subscribers in ROS 2 and manage topic communication. [Read more](python/Subscriber-Publisher.md)

- **Service and Client Guide**: Understand how to create services and clients for synchronous and asynchronous communication. [Read more](python/Service-Client.md)

- **Timers Guide**: Explore how to create and manage timers in ROS 2 for executing periodic tasks. Learn how to set up timers, handle timed callbacks, and manage timer intervals. [Read more](python/Timers.md)

- **Logging Guide**: Learn about the logging capabilities in ROS 2, including how to log different levels of information, format log messages, and configure logging settings for better debugging and monitoring. [Read more](python/Logging.md)



## Advanced Node Concepts

### 1. **Lifecycle Nodes**

Lifecycle nodes provide more control over the state of a node, offering a managed node lifecycle to help write more predictable and safer software. The lifecycle node class includes additional methods for managing state transitions and lifecycle events.

### 2. **Multi-threaded and Multi-process Nodes**

To improve performance or manage complex applications, ROS 2 nodes can be run in a multi-threaded or multi-process setup. The `rclpy` library provides support for these advanced concepts to handle concurrent tasks and processes.

### 3. **Custom Callback Groups**

Custom callback groups can be used to control the execution of callbacks within a node, enabling fine-grained concurrency management.

## Further Reading

For more detailed explanations and examples, consider the following resources:

- [ROS 2 Python API Documentation](https://docs.ros.org/en/humble/Guides.html)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

By understanding the capabilities of the `Node` class and how to use `rclpy`, you'll be well-equipped to develop robust and efficient nodes for your ROS 2 applications.
