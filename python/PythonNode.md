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
- **For detailed examples on logging, see the [Logging Guide](python/Logging.md).**

### 3. **Creating a Timer**

To create a timed event that repeatedly calls a callback function:

```python
self.create_timer(1.0, self.timer_callback)  # Calls `timer_callback` every 1 second
```
    
- **In-depth Explanation:[Timers](python/Timers.md)**

### 4. **Creating a Publisher**

To create a publisher for a specific message type:

```python
self.publisher = self.create_publisher(String, "topic_name", 10)
```

- **Publishing a Message:**

    ```python
    msg = String()
    msg.data = "Hello, ROS 2!"
    self.publisher.publish(msg)
    ```
    
- **In-depth Explanation:[Publishers](python/Subscriber-Publisher.md)**

### 5. **Creating a Subscriber**

To create a subscriber that listens to a specific topic:

```python
self.subscription = self.create_subscription(String, "topic_name", self.subscription_callback, 10)
```

- **Callback Function for Subscriber:**

    ```python
    def subscription_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
    ```
    
- **In-depth Explanation:[Subscribers](python/Subscriber-Publisher.md)**

### 6. **Creating a Service Client**

To create a client for calling a ROS 2 service:

```python
self.client = self.create_client(ServiceType, 'service_name')
```

- **Waiting for the Service:**

    ```python
    while not self.client.wait_for_service(timeout_sec=1.0):
        self.get_logger().warn("Service not available, waiting...")
    ```

- **Creating a Request and Calling the Service:**

    ```python
    request = ServiceType.Request()
    future = self.client.call_async(request)
    future.add_done_callback(self.service_response_callback)
    ```

- **Callback Function for Service Response:**

    ```python
    def service_response_callback(self, future):
        response = future.result()
        self.get_logger().info(f"Service response: {response}")
    ```
    
- **In-depth Explanation:[Service/Client](python/Service-Client.md)**

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
