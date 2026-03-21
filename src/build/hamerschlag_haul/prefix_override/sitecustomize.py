import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hhaul/ros2_ws/src/install/hamerschlag_haul'
