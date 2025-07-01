import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pim0ubuntu/myrobot_ws/src/install/myrobot'
