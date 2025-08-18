import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tersoo/autonomous-robot-logging-with-ROS2-jazzy/myros2_ws/install/twist_bridge'
