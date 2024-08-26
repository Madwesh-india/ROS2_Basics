# Creating a custom workspace

## Getting Started
To create and build this ROS 2 workspace:

1. Create the ROS 2 workspace directory:

   ```bash
   mkdir -p ros2_ws/src
   cd ros2_ws
   ```

2. Clone or create your ROS 2 packages in the `src` directory.
   ### To create a new python ROS 2 packages
   > **Note:** You should be in ros2_ws in the shell
   ```bash
   cd src/
   ```
   
   ```bash
   ros2 pkg create <pakageName> --build-type ament_python --dependencies rclpy
   ```
   > **Note:** Follow [ROSPython.md](ROSPython.md) to create the executable node_script
   
   ```bash
   cd ..
   ```
   

4. Build the workspace:

   ```bash
   colcon build
   ```

5. Source the workspace:

   ```bash
   source install/setup.bash
   ```

6. Run your ROS 2 nodes using `ros2 run`:

   ```bash
   ros2 run pakageName node_script
   ```
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

## Description of Key Files and Directories

- **`src/`**: The source directory containing all ROS 2 packages. Each package is typically a self-contained project.

- **`my_package/`**: An example Python package that contains ROS 2 nodes, services, and other resources.

  - **`CMakeLists.txt`**: (Optional for Python packages) The CMake file required for building C++ packages or any additional CMake configuration. Not commonly used for pure Python packages.

  - **`package.xml`**: The package manifest file, which contains metadata about the package such as name, version, authors, dependencies, and license information.

  - **`setup.py`**: The Python setup script used for installing the package. It defines the package name, version, dependencies, and entry points for nodes.

  - **`resource/`**: A directory for storing additional non-code files needed by the package, such as configuration files or launch files.

  - **`launch/`**: Contains launch files that are used to start multiple nodes and set parameters. Launch files are written in Python for ROS 2.

  - **`my_package/`**: The actual Python module directory. It contains the Python scripts that define the ROS 2 nodes, publishers, subscribers, services, and other components.

  - **`test/`**: A directory for test scripts, including unit tests and integration tests for the package.

  - **`README.md`**: A README file providing an overview of the package, how to build it, and how to use it.

- **`install/`**, **`build/`**, **`log/`**: Directories created by the build system (`colcon`) after building the workspace. These directories contain the installed files, intermediate build files, and logs, respectively.

- **`colcon.meta`**: Optional metadata file for the `colcon` build system, used for configuration and managing build options.

## Notes

- The directory names and structure must adhere to ROS 2 conventions for colcon to recognize and build them properly.
- Python scripts (nodes, publishers, subscribers, etc.) should have executable permissions and correct shebangs (e.g., `#!/usr/bin/env python3`).

