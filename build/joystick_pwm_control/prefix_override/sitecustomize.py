import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/devika/joystick_package_extreme_3d/install/joystick_pwm_control'
