# 🐢 Getting Started with Turtlesim  

Turtlesim is a simple and fun ROS 2 simulator, perfect for learning the foundational concepts of ROS, such as nodes, topics, services, actions, and parameters. This guide provides an overview of Turtlesim, installation instructions, and a walkthrough on running and interacting with it.

---

## 🐢 What is Turtlesim?  

Turtlesim is a lightweight graphical simulator included with ROS 2. It allows you to interact with a turtle in a virtual environment using ROS concepts. Through Turtlesim, you can learn about:  
- **[Nodes](nodes.md)**: Independent programs that communicate in a ROS 2 system.  
- **[Topics](topics.md)**: Publish-subscribe communication channels.  
- **[Services](services.md)**: Request-response communication for one-time tasks.  
- **[Actions](actions.md)**: Goal-oriented asynchronous communication.  
- **[Parameters](parameters.md)**: Configurations for nodes that can be dynamically modified.

---

## ⚙️ Installation  

### Prerequisites  
Ensure that **ROS 2 Humble** is installed and sourced in your terminal. If not, follow the [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html).  

### Installation Command  

Install Turtlesim using the following commands:  

```bash
sudo apt update  
sudo apt install ros-humble-turtlesim  
```

---

## 🚀 Running Turtlesim  

### Step 1: Start the Turtlesim Node  

Run the Turtlesim simulator by executing:  

```bash
ros2 run turtlesim turtlesim_node  
```  

This opens a graphical window with a turtle in the center of a virtual environment.

#### Output:  
- **Turtlesim Window**  
  ![Turtlesim Window](/gettingStarted/TurtlesimImages/turtlesim_window.png "TurtleSimWindow")
- **Terminal Output**  
  ![Turtlesim Terminal](/gettingStarted/TurtlesimImages/turtlesim_terminal.png "TurtleSimTerminal")

---

### Step 2: Control the Turtle  

In a new terminal (remember to source ROS 2), run the following command to control the turtle using your keyboard:  

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

---

## 🌟 Understanding Nodes, Topics, Services, and Actions  

### 🧩 Nodes  

Nodes are independent programs that communicate with each other in ROS 2. In Turtlesim:  
- **`turtlesim_node`**: Manages the Turtlesim window and updates the turtle’s state.  
- **`turtle_teleop_key`**: Sends velocity commands to control the turtle’s movement.  

To list all active nodes, run:  
```bash
ros2 node list
```

---

### 🔗 Topics  

Topics are used for publish-subscribe communication. Nodes publish messages to topics or subscribe to them to receive messages.  

#### Example Topics in Turtlesim:  
- `/turtle1/cmd_vel`: Receives velocity commands to control the turtle.  
- `/turtle1/pose`: Publishes the turtle's position and orientation.  

List all active topics:  
```bash
ros2 topic list
```

#### Example Output:  
```bash
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

You can inspect messages on a topic:  
```bash
ros2 topic echo /turtle1/pose
```

---

### ⚙️ Services  

Services enable one-to-one request-response communication for tasks. Turtlesim provides several services to control its behavior, such as:  
- **`/clear`**: Clears the turtle's trail.  
- **`/spawn`**: Creates a new turtle at a specified position.  

List all available services:  
```bash
ros2 service list
```

Call a service (e.g., clearing the screen):  
```bash
ros2 service call /clear std_srvs/srv/Empty
```

---

### 🎯 Actions  

Actions enable goal-oriented, asynchronous tasks that can provide feedback and be canceled mid-execution. While Turtlesim does not directly include actions, they are common in advanced robotics (e.g., moving a robot to a location).  

To explore actions in other examples, check out:  
[ROS 2 Humble Actions Guide](https://docs.ros.org/en/humble/Tutorials/Actions/Understanding-Actions.html).

---

## 🔗 Exploring Topics with `rqt_graph`  

### Step 1: Install `rqt`  

To visualize ROS communication, install `rqt` and its plugins:  

```bash
sudo apt install ros-humble-rqt*
```

### Step 2: Launch `rqt_graph`  

Run the following command to visualize active nodes and topics:  

```bash
rqt_graph
```

The graph will show how nodes interact through topics like `/turtle1/cmd_vel` and `/turtle1/pose`.

#### Example Graph:  
![rqt_graph Example](/gettingStarted/TurtlesimImages/rqt_graph_example.png "rqt_graph Example")

---

## 🛠️ Using `rqt`  

`rqt` is a GUI-based toolset for interacting with ROS 2. It provides features such as topic monitoring, parameter editing, and node introspection.  

### Launch `rqt`  

Run the following command to start the `rqt` GUI:  
```bash
rqt
```

You can explore plugins such as:  
- **Topics**: Monitor topic data.  
- **Parameters**: View and edit parameters dynamically.  

**Call a service in rqt:**  
1) Open rqt and Navigate to **`Plugin -> Services -> Service Caller`**  in the top bar
2) Select the service `/clear`
3) Press `Call` button

#### Example Graph:  
![rqt Example](/gettingStarted/TurtlesimImages/rqt_example.png "rqt Example")

> Note: Equivalent command line
> ```bash
> ros2 service call /clear std_srvs/srv/Empty
> ```

---

## ⚙️ Parameters  

Parameters in ROS 2 allow you to configure nodes at runtime. For example, in Turtlesim, you can modify the background color dynamically.

### Step 1: List Available Parameters  

To see all parameters for the `turtlesim_node`:  
```bash
ros2 param list /turtlesim
```

### Step 2: Get a Parameter Value  

Check the current background color:  
```bash
ros2 param get /turtlesim background_r
```

### Step 3: Set a Parameter  

Change the background color to blue:  
```bash
ros2 param set /turtlesim background_b 255
```

To apply changes, call the `/clear` service:  
```bash
ros2 service call /clear std_srvs/srv/Empty
```

---

## 🌟 Next Steps  

Now that you’ve explored Turtlesim, dive deeper into the core ROS 2 concepts:  
1. Learn how to write custom nodes in the [Nodes Guide](nodes.md).  
2. Publish and subscribe to topics with the [Topics Guide](topics.md).  
3. Call and create services in the [Services Guide](services.md).  
4. Explore goal-oriented tasks with the [Actions Guide](actions.md).  

Happy learning! 🐢