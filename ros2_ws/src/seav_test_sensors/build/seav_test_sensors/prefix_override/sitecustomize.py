import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/seav/ros2_ws/src/seav_test_sensors/install/seav_test_sensors'
