import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pablo/Desktop/ros2_dam/simulation/lbr-stack/install/lbr_demos_py'
