import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspaces/isaac_ros-dev/src/LineFollower_Sim2Real/src/linepreprocessor/install/linepreprocessor'
