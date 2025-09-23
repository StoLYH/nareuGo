import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/donggun/S13P21A501/Embedded/install/delivery_interface'
