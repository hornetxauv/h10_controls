import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/h10_pooltest_ws/src/h10_controls/install/controls_movement'
