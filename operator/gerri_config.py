ROBOT_ID                    = "avatar_piper_01"
OPERATOR_ID                 = "avatar_operator"

ROBOT_TYPE                  = "manipulator"

MASTER_ARM_USB_LEFT         = '/dev/ttyUSB0'
MASTER_ARM_USB_RIGHT        = '/dev/ttyUSB1'
# MASTER_ARM_USB_LEFT         = 'COM3'
# MASTER_ARM_USB_RIGHT        = 'COM4'



PUPPET_ARM_NAME_LEFT        = "puppet_left"
PUPPET_ARM_NAME_RIGHT       = "puppet_right"

JOINT_LIMIT_LEFT            = [[-90, 30], [None, None], [None, None], [None, None], [None, None], [None, None], [None, None]]
JOINT_LIMIT_RIGHT           = [[-30, 90], [None, None], [None, None], [None, None], [None, None], [None, None], [None, None]]


KEYBOARD_MOUSE_CONTROL_UI   = True