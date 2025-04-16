import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bah/LineFollower_Sim2Real/install/linepreprocessor'
