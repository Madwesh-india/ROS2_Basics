# 🐢 Getting Started with Turtlesim  

Turtlesim is a simple and fun ROS 2 simulator, perfect for learning the foundational concepts of ROS, such as nodes, topics, services, actions, and parameters. This guide provides an overview of Turtlesim, installation instructions, and a walkthrough on running and interacting with it.

---

## 🐢 What is Turtlesim?  

Turtlesim is a lightweight graphical simulator included with ROS 2. It allows you to interact with a turtle in a virtual environment while exploring core ROS 2 concepts, including:  
- **[Nodes](nodes.md)**: Independent programs that communicate in a ROS 2 system.  
- **[Topics](topics.md)**: Publish-subscribe communication channels for message exchange.  
- **[Services](services.md)**: Request-response communication for one-time tasks.  
- **[Actions](actions.md)**: Asynchronous communication for goal-oriented tasks.  
- **[Parameters](parameters.md)**: Configurable settings for nodes, modifiable at runtime.

---

## ⚙️ Installation  

### Prerequisites  
Ensure **ROS 2 Humble** is installed and sourced in your terminal. If not, follow the [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html).  

### Installation Command  

Install Turtlesim using the following commands:  

```bash
sudo apt update  
sudo apt install ros-humble-turtlesim  
```

---

## 🚀 Running Turtlesim  

### Step 1: Start the Turtlesim Node  

Launch the Turtlesim simulator with:  

```bash
ros2 run turtlesim turtlesim_node  
```  

A graphical window with a turtle in the center of a virtual environment will appear.

#### Output:  
- **Turtlesim Window**  
  ![Turtlesim Window](/gettingStarted/TurtlesimImages/turtlesim_window.png "TurtleSimWindow")
- **Terminal Output**  
  ![Turtlesim Terminal](/gettingStarted/TurtlesimImages/turtlesim_terminal.png "TurtleSimTerminal")

> Note: Use **ctrl+c** to terminate the execution
---

### Step 2: Control the Turtle  

Open a new terminal (ensure ROS 2 is sourced) and run:  

```bash
ros2 run turtlesim turtle_teleop_key  
```  

Use the **arrow keys** to move the turtle:  
- **Up**: Move forward  
- **Down**: Move backward  
- **Left/Right**: Turn  

#### Output:  
- **Moved Turtle**  
  ![Moved Turtle](/gettingStarted/TurtlesimImages/moved_turtle.png "Moved Turtle")  
- **Teleop Terminal Output**  
  ![Turtle Teleop Keyboard](/gettingStarted/TurtlesimImages/turtle_teleop_keybord.png "Turtle Teleop Keyboard")

> Note: The conventon of the direction for control is about the **local axis** , not the fixed **global axis**

---

## 🌟 Key ROS 2 Concepts  

### 🧩 Nodes  

Nodes are independent programs in ROS 2 that perform specific tasks and communicate with each other. In Turtlesim:  
- **`turtlesim_node`**: Manages the simulation window and turtle behavior.  
- **`turtle_teleop_key`**: Sends movement commands to the turtle.  

List active nodes:  
```bash
ros2 node list
```

---

### 🔗 Topics  

Topics are publish-subscribe communication channels. Nodes either publish messages to topics or subscribe to receive messages.  

#### Example Topics in Turtlesim:  
- `/turtle1/cmd_vel`: Controls the turtle’s movement.  
- `/turtle1/pose`: Reports the turtle’s position and orientation.  

List active topics:  
```bash
ros2 topic list
```

Inspect messages on a topic:  
```bash
ros2 topic echo /turtle1/pose
```
> Note: Use **ctrl+c** to terminate the execution

### Visualizing Topics with `rqt_graph`  

#### Install `rqt`:  
```bash
sudo apt install ros-humble-rqt*
```

#### Launch `rqt_graph`:  
```bash
rqt_graph
```

The graph will display nodes and topics, showing their interactions.

---

### ⚙️ Services  

Services enable request-response communication. For example:  
- **`/clear`**: Clears the turtle’s trail.  
- **`/spawn`**: Spawns a new turtle at a specified location.  

List available services:  
```bash
ros2 service list
```

Call a service to clear the screen:  
```bash
ros2 service call /clear std_srvs/srv/Empty
```

---

### 🎯 Actions  

Actions enable asynchronous tasks with feedback and cancel options. While Turtlesim doesn't have built-in actions, they are used in tasks like robot navigation.  

Explore actions in ROS 2:  
[ROS 2 Actions Guide](https://docs.ros.org/en/humble/Tutorials/Actions/Understanding-Actions.html).

---

### ⚙️ Parameters  

Parameters are runtime configurations for nodes. Modify parameters like the background color in Turtlesim.  

#### List Parameters:  
```bash
ros2 param list /turtlesim
```

#### Get Parameter Value:  
```bash
ros2 param get /turtlesim background_r
```

#### Set Parameter:  
```bash
ros2 param set /turtlesim background_b 255
```

Apply changes with:  
```bash
ros2 service call /clear std_srvs/srv/Empty
```

---

## 🛠️ Using `rqt`  

Launch the `rqt` GUI to monitor topics, call services, or edit parameters:  
```bash
rqt
```

#### Example: Call the `/clear` service  
1. Navigate to **Plugins → Services → Service Caller**.  
2. Select `/clear`.  
3. Click **Call**.  

![rqt Example](/gettingStarted/TurtlesimImages/rqt_example.png "rqt Example")

---

## 🌟 Next Steps  

**Controling Turtle through terminal commands: [TerminalTurtle.md](TerminalTurtle.md)**

Dive deeper into ROS 2 concepts: 
- Explore [Nodes](nodes.md) and [Topics](topics.md).  
- Work with [Services](services.md) and [Actions](actions.md).  
- Experiment with dynamic [Parameters](parameters.md).  

Happy learning! 🐢