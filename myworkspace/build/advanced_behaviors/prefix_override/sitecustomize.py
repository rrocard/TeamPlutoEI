import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mystudent/Documents/EiDrone/myworkspace/install/advanced_behaviors'