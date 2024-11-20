# 🖥️ Exploring Turtlesim Through the Terminal  

This guide covers essential terminal commands for interacting with Turtlesim. Learn how to observe, control, and debug the Turtlesim node through ROS 2 commands.
You can go though [**ShellCommands.md**](ShellCommands.md) for understanding commands.

---
## 🚀 Starting the Turtlesim Node  

Launch the Turtlesim simulator in a terminal:  

```bash
ros2 run turtlesim turtlesim_node
```

## 🔗 Monitoring Topics  

List all active topics in a new terminal:  

```bash
ros2 topic list
```

### Example Output:  

```bash
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

---

## 📡 Inspecting Topic Data  

You can observe the data being published to a specific topic using the `ros2 topic echo` command.  

### Example: Observe Turtle's Pose  

```bash
ros2 topic echo /turtle1/pose
```
> Note: Use **ctrl+c** to terminate the execution

#### Example Output:  
```yaml
x: 5.544445
y: 5.544445
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
```

---

## 🔧 Debugging with `ros2 info`  

### Get Topic Information  

To inspect a topic and see its type, use:  

```bash
ros2 topic info /turtle1/cmd_vel
```

#### Example Output:  

```bash
Type: geometry_msgs/msg/Twist
Publisher count: 0
Subscription count: 1
```

### Get Message Definition  

To understand the structure of messages on a topic:  

```bash
ros2 interface show geometry_msgs/msg/Twist
```

#### Example Output:  

```yaml
Vector3  linear
	float64 x
	float64 y
	float64 z
Vector3  angular
	float64 x
	float64 y
	float64 z
```

---

## ⚙️ Exploring Parameters  

List all parameters for the `turtlesim_node`:  

```bash
ros2 param list /turtlesim
```

### Get a Parameter Value  

Check the background color’s red component:  

```bash
ros2 param get /turtlesim background_r
```

### Set a Parameter Value  

Change the red component of the background color:  

```bash
ros2 param set /turtlesim background_r 255
```

Apply changes by clearing the screen:  

```bash
ros2 service call /clear std_srvs/srv/Empty
```

---

## 🛠️ Call a Service  

List all available services:  

```bash
ros2 service list
```

### Call the `/clear` Service  

Clear the turtle's trail with:  

```bash
ros2 service call /clear std_srvs/srv/Empty
```

### Spawn a New Turtle  

Add a new turtle at position (2, 2):  

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: ''}"
```

#### Example Output:  

```yaml
name: turtle2
```

---

## 🌟 Advanced: Inspecting Nodes  

### Inspect a Node  

View detailed information about a node:  

```bash
ros2 node info /turtlesim
```

---

## 🌀 Exercise: Drawing Circles with Multiple Turtles  

This exercise will guide you to draw two circles using two turtles and observe their poses.

---

### 🐢 Step 1: Launch Turtlesim  

Start the Turtlesim simulator in one terminal:  
```bash
ros2 run turtlesim turtlesim_node
```

---

### 🐢 Step 2: Spawn a Second Turtle  

Spawn a new turtle at position (2, 2):  
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

This will create a second turtle, named `turtle2`.

---

### 🐢 Step 3: Move Turtles to Draw Circles  

Use the following commands to make each turtle draw a circle:  

#### **Control `turtle1`**  

Publish velocity commands for `turtle1`:  
```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}" -r 10
```

#### **Control `turtle2`**  

Publish velocity commands for `turtle2`:  
```bash
ros2 topic pub /turtle2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.2}}" -r 10
```

---

### 🐢 Step 4: Observe Turtle Poses  

Monitor the poses of both turtles in separate terminals:  

- **Turtle 1 Pose**  
  ```bash
  ros2 topic echo /turtle1/pose
  ```

- **Turtle 2 Pose**  
  ```bash
  ros2 topic echo /turtle2/pose
  ```
- **Output**

    ![Two Circle](/gettingStarted/TurtlesimImages/turtle_circle.png "Two Circle")
---

### 🐢 Step 5: Experiment with Circle Sizes  

Control the size of the circles by adjusting the `linear.x` (speed) and `angular.z` (rotation rate) values in the `ros2 topic pub` commands. For example:  
- Increase `linear.x` to make the circle larger.  
- Increase `angular.z` to make the circle tighter.

---

## 🌟 Next Steps  

**Controling Turtle through python commands: [PythonTurtle.md](PythonTurtle.md)**

## 🎯 Summary  

By completing this exercise, you’ve learned how to:  
1. Spawn and control multiple turtles.  
2. Draw patterns using velocity commands.  
3. Inspect turtle poses and modify behavior dynamically.  

Now try creating other shapes or experimenting with different speeds and angles! 🐢