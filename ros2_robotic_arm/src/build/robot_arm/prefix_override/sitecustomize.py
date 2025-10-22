import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/CIN/mas11/Documents/robotic_arm/ros2_robotic_arm/src/install/robot_arm'
