# 🐍 Interacting with Turtlesim Using Python  

This guide teaches you to create Python-based ROS 2 nodes for interacting with Turtlesim. It includes publishing, subscribing, service calls, and advanced tasks like conditional execution and drawing patterns. Each script is configured with entry points and can be executed using `ros2 run` after building the workspace.  

---

## 🏗️ Setting Up a ROS 2 Workspace  

Refer to [**Workspace.md**](/python/Workspace.md) for setting up a workspace and package.  
**Workspace Name**: `turtle_ws`  
**Package Name**: `turtlesim_python`

---

## ⚙️ Configuration  

### Update `setup.py`  

Include Python scripts in the [`setup.py`](/python/ROS2Python.md#setting-up-setuppy) file:  

```python
'console_scripts': [
    # add here
]
```

### Build and Source  

1. **Build the Package**:  
   Run the following from **[your workspace directory](/BasicLinuxCommands.md#-file-and-directory-management)**:  
   ```bash
   colcon build --symlink-install
   ```
    > Colcon Build everytime there are **changes** if `--symlink-install` is not used

2. **Source the Workspace**:  
   ```bash
   source install/setup.bash
   ```
    > Source everytime a **new terminal** is opened

---

## 🟢 Simple Publisher  

This node publishes velocity commands to move the turtle.  

**Script: `publisher.py`**  

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_velocity)

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 2.0  # Forward speed
        msg.angular.z = 1.0  # Rotational speed
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: Linear=%.2f Angular=%.2f' % (msg.linear.x, msg.angular.z))

def main():
    rclpy.init()
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Description:  
- Publishes `geometry_msgs/msg/Twist` messages to `/turtle1/cmd_vel` at 0.5-second intervals.
- Moves the turtle in a circular trajectory.  

**Adding entrypoint in setup.py**
```python
'console_scripts': [
    ...
    'publisher = turtlesim_python.publisher:main',
    ...
]
```

**Run using:**  
```bash
ros2 run turtlesim_python publisher
```

---

## 🟡 Simple Subscriber  

This node subscribes to the `/turtle1/pose` topic to get the turtle's position.  

**Script: `subscriber.py`**  

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        self.get_logger().info(f'Turtle Pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}')

def main():
    rclpy.init()
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
 
**Adding entrypoint in setup.py**
```python
'console_scripts': [
    ...
    'subscriber = turtlesim_python.subscriber:main',
    ...
]
```

### Description:  
- Subscribes to `turtlesim/msg/Pose` messages from `/turtle1/pose`.  
- Logs the turtle’s current position and orientation.  

**Run using:**  
```bash
ros2 run turtlesim_python subscriber
```

---

## 🔵 Simple Service Client  

This node calls the `/spawn` service to create a new turtle.  

**Script: `service_client.py`**  

```python
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class SimpleClient(Node):
    def __init__(self):
        super().__init__('simple_client')
        self.client = self.create_client(Spawn, '/spawn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Spawn service...')
        self.request = Spawn.Request()
        self.request.x = 2.0
        self.request.y = 2.0
        self.request.theta = 0.0

    def send_request(self):
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Spawned Turtle: {future.result().name}')
        else:
            self.get_logger().info('Service call failed.')

def main():
    rclpy.init()
    node = SimpleClient()
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

 
**Adding entrypoint in setup.py**
```python
'console_scripts': [
    ...
    'service_client = turtlesim_python.service_client:main',
    ...
]
```

### Description:  
- Calls the `/spawn` service to add a new turtle at position `(2.0, 2.0)`.  

**Run using:**  
```bash
ros2 run turtlesim_python service_client
```

---## 🛠️ Conditional Execution  

This script moves the turtle until its `x` position is greater than 7.  

**Script: `conditional_movement.py`**  

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class ConditionalMovement(Node):
    def __init__(self):
        super().__init__('conditional_movement')
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.stop_moving = False

    def pose_callback(self, msg):
        if msg.x < 7.0 and not self.stop_moving:
            cmd = Twist()
            cmd.linear.x = 1.0
            self.publisher.publish(cmd)
            self.get_logger().info(f'Moving Turtle: x={msg.x:.2f}')
        else:
            self.stop_moving = True
            self.get_logger().info('Turtle stopped.')

def main():
    rclpy.init()
    node = ConditionalMovement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Adding entrypoint in setup.py**
```python
'console_scripts': [
    ...
    'conditional_movement = turtlesim_python.conditional_movement:main',
    ...
]
```

Execute the script:  
```bash
ros2 run turtlesim_python conditional_movement
```

---


## 🌀 Drawing Two Circles  

This node moves two turtles to draw circular patterns simultaneously.  

**Script: `two_circles.py`**  

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwoCircles(Node):
    def __init__(self):
        super().__init__('two_circles')
        self.pub_turtle1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_turtle2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_turtles)

    def move_turtles(self):
        cmd1 = Twist()
        cmd1.linear.x = 2.0
        cmd1.angular.z = 1.0
        self.pub_turtle1.publish(cmd1)

        cmd2 = Twist()
        cmd2.linear.x = 1.5
        cmd2.angular.z = 1.2
        self.pub_turtle2.publish(cmd2)

def main():
    rclpy.init()
    node = TwoCircles()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Adding entrypoint in setup.py**
```python
'console_scripts': [
    ...
    'two_circles = turtlesim_python.two_circles:main',
    ...
]
```

### Description:  
- Publishes `geometry_msgs/msg/Twist` commands to `/turtle1/cmd_vel` and `/turtle2/cmd_vel` to create circular trajectories for both turtles.  
- Adjusts `linear.x` and `angular.z` values for different circle sizes.  

**Run using:**  
1. Spawn the second turtle:  
   ```bash
   ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
   ```  

2. Execute the script:  
   ```bash
   ros2 run turtlesim_python two_circles
   ```

---

## 🎯 Summary  

With these scripts, you’ve learned to:  
1. Publish and subscribe to topics.  
2. Call ROS 2 services.  
3. Implement multi-turtle motion and dynamic control.  

Use `rqt_graph` to visualize the nodes and topics while they run. Happy coding! 🐢

