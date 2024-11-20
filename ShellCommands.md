# ShellCommands

## Basic Shell Commands for ROS 2

This document provides a list of essential shell commands for working with ROS 2 (Robot Operating System 2). These commands are commonly used for running nodes, building packages, and interacting with topics, services, and other components within the ROS 2 environment.

### 1. Managing Nodes

To run a node from a specific package:

```bash
ros2 run <package_name> <executable_name>
```
> Note: Use **ctrl+c** to terminate any terminal execution

To list all active nodes:

```bash
ros2 node list
```

To get detailed information about a specific node:

```bash
ros2 node info /<node_name>
```

### 2. Working with Topics

To list all active topics:

```bash
ros2 topic list
```

To get information about a specific topic, including its datatype and publisher/subscriber count:

```bash
ros2 topic info /<topic_name>
```

To view the data being published to a topic:

```bash
ros2 topic echo /<topic_name>
```

To display information about the datatype of a topic:

```bash
ros2 interface show <Type>
```

### 3. Working with Services

To list all active services:

```bash
ros2 service list
```

To get the type of a specific service:

```bash
ros2 service type /<service_name>
```

To call a service:

```bash
ros2 service call /<service_name> <service_type> '{<args>}'
```

### 4. Building ROS 2 Packages

To build all packages in your workspace:

```bash
colcon build
```

For incremental builds (useful when working on files without creating new ones):

```bash
colcon build --symlink-install
```
---

> 🚨 **Note:** `--symlink-install` cannot be used after adding custom messages

---

### 5. Visualizing ROS 2 Graph

To visualize all active nodes and their connections in a graph format:

```bash
rqt_graph
```

### 6. Converting Xacro to URDF

To convert a Xacro file to URDF:

```bash
ros2 run xacro xacro <xacro_path> > <urdf_path>
```

### Summary

These commands provide the foundation for interacting with ROS 2 systems, enabling you to run nodes, build and manage packages, and interact with topics and services. For more detailed information on each command, use the `--help` option with the command, e.g., `ros2 topic --help`.
