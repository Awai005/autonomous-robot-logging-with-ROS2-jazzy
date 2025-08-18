# Robot Navigation with Blockchain Logging

## Overview
This package integrates a ROS2 Jazzy node with a local Ethereum blockchain (using Ganache) to log robot navigation data as transactions. Each time the robot reaches a navigation goal, it saves a transaction containing a timestamp, position, and status message to the blockchain.

## Prerequisites
- **Ubuntu 24.04** with **ROS2 Jazzy** installed.
- **Python 3.12** (default with Ubuntu 24.04).
- **Node.js and npm** for Ganache installation.
- **Python `web3` library**: Install with `pip install web3`.
- A working ROS2 navigation stack (e.g., Nav2) with Gazebo Harmonic.

## Main ROS2 Functionality
[Add your description of the main ROS2 navigation functionality here, e.g., how the robot navigates, uses Nav2, or interacts with Gazebo.]


## Blockchain Setup with Ganache
The node uses Ganache to simulate a local Ethereum blockchain for logging transactions.


### Running Ganache
- **CLI**: Start Ganache to run a local blockchain on `http://127.0.0.1:8545`:
  ```bash
  ganache
  ```
  Note the first account address (e.g., `0x...`) displayed in the terminal.


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

## Troubleshooting
- **Ganache Connection**: Ensure Ganache is running and the port matches (`8545` for CLI, `7545` for GUI).
- **web3 Errors**: Verify `web3` is installed (`pip show web3`) and matches your Python version.
- **Navigation Issues**: Ensure `/navigate_to_pose` action server is active (`ros2 action list`).
- **Build Errors**: Check `colcon build` logs and verify `package.xml`/`CMakeLists.txt` dependencies.
