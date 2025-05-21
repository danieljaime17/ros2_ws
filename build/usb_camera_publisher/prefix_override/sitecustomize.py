import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lenovo/ros2_ws/install/usb_camera_publisher'
