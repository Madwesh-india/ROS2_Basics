# 🖥️ Basic Linux Commands for ROS 2  

Working with ROS 2 often involves using Linux commands for navigation, file management, and interaction with ROS 2 nodes. This guide covers the essential Linux commands useful for ROS 2 development.

---

## 📂 File and Directory Management  

### Navigate Directories  

- **Change Directory (`cd`)**:  
  ```bash
  cd /path/to/directory
  ```
  Example:  
  ```bash
  cd ~/ros2_ws/src
  ```

- **Go Back One Directory (`cd ..`)**:  
  ```bash
  cd ..
  ```

- **Go to the Home Directory**:  
  ```bash
  cd ~
  ```

### View Directory Contents  

- **List Files and Folders (`ls`)**:  
  ```bash
  ls
  ```
  Options:  
  - `-l`: Detailed view.  
  - `-a`: Show hidden files.  
  ```bash
  ls -la
  ```

### Create Directories  

- **Make Directory (`mkdir`)**:  
  ```bash
  mkdir folder_name
  ```
  Example:  
  ```bash
  mkdir ros2_ws
  ```

### Remove Files or Directories  

- **Remove File (`rm`)**:  
  ```bash
  rm file_name
  ```

- **Remove Directory (`rm -r`)**:  
  ```bash
  rm -r directory_name
  ```

> ⚠️ Use `rm -r` with caution as it deletes the directory and its contents.

---

## 🛠️ File Operations  

### Copy Files  

- **Copy (`cp`)**:  
  ```bash
  cp source destination
  ```
  Example:  
  ```bash
  cp ~/ros2_ws/src/file.py ~/ros2_ws/src/copy_of_file.py
  ```

### Move or Rename Files  

- **Move (`mv`)**:  
  ```bash
  mv source destination
  ```
  Example:  
  ```bash
  mv old_name.py new_name.py
  ```

---

## 🔍 Searching and Viewing  

### Search for Files  

- **Find Files (`find`)**:  
  ```bash
  find /path -name "file_name"
  ```
  Example:  
  ```bash
  find ~/ros2_ws -name "*.py"
  ```

### View File Contents  

- **Display Entire File (`cat`)**:  
  ```bash
  cat file_name
  ```

- **View File with Paging (`less`)**:  
  ```bash
  less file_name
  ```

---

## 🔗 ROS 2-Specific Commands  

### Build a ROS 2 Workspace  

- Navigate to the root of the workspace:  
  ```bash
  cd ~/ros2_ws
  ```
- Build the workspace:  
  ```bash
  colcon build
  ```

### Source the Workspace  

- Source the ROS 2 environment:  
  ```bash
  source /opt/ros/humble/setup.bash
  ```
- Source your custom workspace:  
  ```bash
  source ~/ros2_ws/install/setup.bash
  ```

### ROS 2 Package Management  

- **Create a ROS 2 Package**:  
  ```bash
  ros2 pkg create package_name --build-type ament_cmake --dependencies rclcpp
  ```
  Example:  
  ```bash
  ros2 pkg create turtlesim_extension --build-type ament_cmake --dependencies turtlesim
  ```

- **List ROS 2 Packages**:  
  ```bash
  ros2 pkg list
  ```

### Running ROS 2 Nodes  

- Run a node using `ros2 run`:  
  ```bash
  ros2 run package_name node_name
  ```
  Example:  
  ```bash
  ros2 run turtlesim turtlesim_node
  ```

---

## 📊 Monitoring System and Logs  

### System Resource Usage  

- **Check Disk Usage (`df`)**:  
  ```bash
  df -h
  ```

- **Check Free Memory (`free`)**:  
  ```bash
  free -h
  ```

### ROS 2 Logging  

- Check log files for troubleshooting:  
  ```bash
  cd ~/.ros/log
  ls
  ```

---

## 🔑 Permissions  

### Change File Permissions  

- **Make a File Executable**:  
  ```bash
  chmod +x file_name
  ```
  Example:  
  ```bash
  chmod +x script.py
  ```

### Switch to Root User  

- **Run a Command as Root (`sudo`)**:  
  ```bash
  sudo command
  ```
  Example:  
  ```bash
  sudo apt update
  ```

---

## 🌟 Tips for Efficient ROS 2 Workflow  

1. **Use Aliases**: Add shortcuts in your `~/.bashrc` file for frequent commands.  
   Example:  
   ```bash
   alias cw='cd ~/ros2_ws'
   alias cb='colcon build && source ~/ros2_ws/install/setup.bash'
   ```

2. **View Workspace Paths**: Verify the source paths with:  
   ```bash
   echo $ROS_PACKAGE_PATH
   ```

3. **Check Python Version**: ROS 2 requires Python 3. Check your version:  
   ```bash
   python3 --version
   ```

---

## 📚 Next Steps  

Familiarize yourself with:  
- [ROS 2 Basics](https://docs.ros.org/en/rolling/Tutorials.html)  
- [Turtlesim Exercises](turtlesim_tasks.md)  

Happy ROS 2 learning! 🚀