# 🛠️ Troubleshooting: `ros2` Command Not Found  

This guide provides solutions to common issues when the `ros2` command is not recognized.

---

## ⚠️ Common Mistake 1: Not Sourcing ROS 2  

If you encounter the error:  
```bash
ros2: command not found
```
It means the ROS 2 environment has not been sourced in the terminal session.

### Temporary Fix: Source ROS 2 in the Current Terminal  

Run the following command to source your ROS 2 installation:  

```bash
source /opt/ros/humble/setup.bash
```

> **Note:** This command needs to be run every time you open a new terminal unless automated.

---

## ✅ Permanent Solution: Add ROS 2 to `.bashrc`  

To avoid manually sourcing ROS 2 every time, follow these steps:  

### Step 1: Open the `.bashrc` File  

Run the following command:  
```bash
gedit ~/.bashrc
```

### Step 2: Add the ROS 2 Source Command  

Scroll to the end of the file and add:  
```bash
source /opt/ros/humble/setup.bash
```

### Step 3: Apply the Changes  

Run this command to reload the updated `.bashrc` file:  
```bash
source ~/.bashrc
```

---

## ⚠️ Common Mistake 2: `source` Command Returns "No Such File or Directory"  

If you see the error:  
```bash
bash: /opt/ros/humble/setup.bash: No such file or directory
```
This likely means:  
1. **ROS 2 is not installed** or  
2. **A different ROS 2 distribution is installed**  

### Step 1: Check Installed ROS 2 Distributions  

Verify which ROS 2 distributions are installed by listing the directories under `/opt/ros`:  
```bash
ls /opt/ros
```

If `humble` is not listed, you likely have a different distribution installed or no installation at all.  

### Step 2: Install or Reinstall ROS 2  

If ROS 2 is missing or incorrect, reinstall the desired distribution (`humble` in this case). Follow the official installation guide for ROS 2 Humble:  
[ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)

### Step 3: Verify Installation  

After reinstalling, ensure the correct setup file exists:  
```bash
ls /opt/ros/humble/setup.bash
```

If the file is present, source it again:  
```bash
source /opt/ros/humble/setup.bash
```

---