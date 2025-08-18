# Robot Navigation with Blockchain Logging

## Overview
This package integrates a ROS2 Jazzy node with a local Ethereum blockchain (using Ganache) to log robot navigation data as transactions. Each time the robot reaches a navigation goal, it saves a transaction containing a timestamp, position, and status message to the blockchain.

## Prerequisites
- **Ubuntu 24.04** with **ROS2 Jazzy** installed.
- **Python 3.12** (default with Ubuntu 24.04).
- **Node.js and npm** for Ganache installation.
- **Python `web3` library**: Install with `pip install web3`.
- A working ROS2 navigation stack (e.g., Nav2) with Gazebo Harmonic.

## Blockchain Setup with Ganache
The node uses Ganache to simulate a local Ethereum blockchain for logging transactions.

### Installing Ganache
1. Install Node.js and npm:
   ```bash
   sudo apt update
   sudo apt install nodejs npm
   ```
   Alternatively, for a newer Node.js version, use Node Version Manager (nvm):
   ```bash
   curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
   source ~/.bashrc
   nvm install 20
   ```

2. Install Ganache CLI:
   ```bash
   npm install ganache --global
   ```

3. Alternatively, for the Ganache GUI:
   - Download the latest Linux AppImage from [Ganache releases](https://github.com/trufflesuite/ganache/releases) (e.g., `ganache-2.7.1-linux-x86_64.AppImage`).
   - Navigate to the download directory: `cd ~/Downloads`.
   - Make it executable: `chmod a+x ganache-*.AppImage`.
   - Install fuse if needed: `sudo apt install libfuse2`.
   - Run it: `./ganache-*.AppImage`.

### Running Ganache
- **CLI**: Start Ganache to run a local blockchain on `http://127.0.0.1:8545`:
  ```bash
  ganache
  ```
  Note the first account address (e.g., `0x...`) displayed in the terminal.

- **GUI**: Launch the AppImage, create a quick workspace, and note the RPC server URL (default: `http://127.0.0.1:7545`). Update the `ganache_url` in `nav_goal_logger.py` to `7545` if using the GUI.

## ROS2 Node for Blockchain Logging
The `nav_goal_logger` node logs navigation data to the blockchain as raw transactions, including a timestamp, robot position, and status message (e.g., "Reached waypoint_2").

### Setup
1. Ensure the `web3` library is installed:
   ```bash
   pip install web3
   ```
   Use `sudo pip3 install web3` if installing system-wide.

2. Add dependencies to your `package.xml`:
   ```xml
   <depend>rclpy</depend>
   <depend>nav2_msgs</depend>
   <depend>geometry_msgs</depend>
   <exec_depend>python3-web3</exec_depend>
   ```

3. Update `CMakeLists.txt` to install the node:
   ```cmake
   install(PROGRAMS
     my_robot_nav/nav_goal_logger.py
     DESTINATION lib/${PROJECT_NAME}
   )
   ```

4. Place `nav_goal_logger.py` in your package’s Python module directory (e.g., `my_robot_nav/my_robot_nav/`).

### Building the Package
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_nav
source install/setup.bash
```

### Running the Node
1. Start Ganache (CLI or GUI) to ensure the blockchain is running.
2. Launch your ROS2 navigation stack and Gazebo simulation:
   ```bash
   ros2 launch <your_nav_package> <your_nav_launch_file>
   ```
3. Run the node:
   ```bash
   ros2 run my_robot_nav nav_goal_logger
   ```
   The node sends a test goal to `/navigate_to_pose` and logs a transaction when the goal is reached. To use with actual navigation goals, modify the node to subscribe to `/goal_pose` or integrate with your navigation pipeline.

### Transaction Logging
- When the robot reaches a goal, the node sends a transaction with:
  - **Timestamp**: Unix epoch time (e.g., `1739876543`).
  - **Position**: Robot’s coordinates (e.g., `x:1.00 y:2.00 z:0.00`).
  - **Message**: Status (e.g., `Reached waypoint_2`).
- The transaction hash and decoded data are logged to the console:
  ```
  [INFO] [nav_goal_logger]: Transaction hash: 0x123...
  [INFO] [nav_goal_logger]: Transaction 0x123... details:
  [INFO] [nav_goal_logger]:   Data: Timestamp: 1739876543, Position: x:1.00 y:2.00 z:0.00, Message: Reached waypoint_2
  ```

### Viewing Transactions
- **In the Node**: The node automatically prints transaction details after logging.
- **Ganache CLI**: Check the terminal for transaction logs (hex-encoded `input` field).
- **Ganache GUI**: Go to the "Transactions" tab, select a transaction, and decode the `data` field (hex) using:
  ```python
  from web3 import Web3
  print(Web3.to_text("0x..."))  # Replace with hex from GUI
  ```
- **Programmatically**: Add a method to `nav_goal_logger.py` to scan all transactions (see code comments for `view_all_transactions`).

## Main ROS2 Functionality
[Add your description of the main ROS2 navigation functionality here, e.g., how the robot navigates, uses Nav2, or interacts with Gazebo.]

## Troubleshooting
- **Ganache Connection**: Ensure Ganache is running and the port matches (`8545` for CLI, `7545` for GUI).
- **web3 Errors**: Verify `web3` is installed (`pip show web3`) and matches your Python version.
- **Navigation Issues**: Ensure `/navigate_to_pose` action server is active (`ros2 action list`).
- **Build Errors**: Check `colcon build` logs and verify `package.xml`/`CMakeLists.txt` dependencies.