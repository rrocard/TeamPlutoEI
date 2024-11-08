import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/users/student/2022chojnackay/Bureau/EI/teamplutoei/install/lower_level'
