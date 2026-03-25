import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pablo/Desktop/ros2_dam/ros2_ws/install/baile_sencillo'
