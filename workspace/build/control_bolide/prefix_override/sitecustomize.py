import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/voiture/course_2025_slam_pkgs/workspace/install/control_bolide'
