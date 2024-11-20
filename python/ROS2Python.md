# ROSPython.md

## Basic Python Workflow and Code Examples in ROS 2

This document provides an overview of essential Python operations, methods, and coding practices for developing with ROS 2 (Robot Operating System 2). 
It includes examples of creating nodes, managing publishers and subscribers, handling services, and setting up your package with Python scripts.

### Table of Contents

- [Introduction](#introduction)
- [Important Python Operations](#important-python-operations)
- [Node Methods and Functions](#node-methods-and-functions)
- [Setting Up `setup.py`](#setting-up-setuppy)

## Introduction

In ROS 2, Python is a popular choice for writing nodes and handling various functionalities like publishers, subscribers, and services. This guide provides you with the foundational Python code snippets and workflows needed to get started with ROS 2 development.


## ROS 2 Python Workspace Structure

This outlines the typical file structure for a ROS 2 workspace using Python. It includes directories and files essential for building and running ROS 2 nodes, services, and other components.

## Workspace Structure

```
ros2_ws/                           # Root of the ROS 2 workspace
├── src/                           # Source directory for ROS 2 packages
│   ├── my_package/                # Example ROS 2 Python package
│   │   ├── CMakeLists.txt         # CMake file for colcon build (can be optional for Python packages)
│   │   ├── package.xml            # Package manifest file with package information
│   │   ├── setup.py               # Python setup script for installing the package
│   │   ├── resource/              # Resource directory for additional files
│   │   ├── launch/                # Launch files directory
│   │   │   └── example_launch.py  # Example launch file for starting nodes
│   │   ├── my_package/            # Python module directory (must match package name)
│   │   │   ├── __init__.py        # Python package initializer
│   │   │   ├── node_script.py     # Python script for a ROS 2 node
│   │   ├── test/                  # Directory for test files
│   │   │   └── test_example.py    # Example test script for unit testing
│   │   └── README.md              # README file for package description and usage
├── install/                       # Directory where built packages are installed (created after build)
├── build/                         # Directory where packages are built (created after build)
├── log/                           # Directory containing build and run logs (created after build)
└── colcon.meta                    # Metadata for colcon build system (optional)
```
  
> **Refer:[Setting-up Ros Workspace](Workspace.md)**

## Important Python Operations

### 1. **Importing the rclpy Module**

To work with ROS 2 in Python, you must import the `rclpy` module:

```python
import rclpy
from rclpy.node import Node
```

### 2. **Creating a ROS 2 Node Class**

Define a class that inherits from `Node`:

```python
class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")  # 'my_node' is the name displayed in the ROS 2 graph
```

### 3. **Main Function for Node Execution**

The `main()` function initializes the ROS 2 client library, creates an instance of the node, and starts spinning it to keep it alive:

```python
def main(args=None):
    rclpy.init(args=args)                    # Initialize ROS 2 Python client library
    node = MyNode()                          # Create an instance of the node
    rclpy.spin(node)                         # Keep the node running and processing callbacks
    rclpy.shutdown()                         # Shutdown ROS 2 Python client library

if __name__ == "__main__":
    main()                                    # Entry point for the node script
```
> **Note:** For further information [PythonNode.md](PythonNode.md).
## Setting Up `setup.py`

To ensure your Python nodes are executable, you must modify the `setup.py` file in your ROS 2 package:

```python
from setuptools import setup

package_name = 'your_package_name'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'nodeScriptName_node'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nodeCallName = packageName.nodeScriptName_node:main'
        ],
    },
)
```
    
- **In-depth Explanation:[Setup](python/Setup.md)**

> **Note:** `nodeCallName`, `nodeScriptName`, and `nodeDisplayedName` can be the same or different depending on your preference and project requirements.
