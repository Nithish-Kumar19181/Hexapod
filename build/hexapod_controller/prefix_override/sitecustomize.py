import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nithish/slam_hexapod_5th_jan/install/hexapod_controller'
