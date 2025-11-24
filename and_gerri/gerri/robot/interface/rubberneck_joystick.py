import threading

class RubberNeckJoyStick:
    def __init__(self):
        self.duplicate_queue = []

    def on_message(self,value):
        axis = value.get('axes')
        buttons = value.get('buttons')

        LT = True if (axis[4] or buttons[6]) else False
        RT = True if (axis[5] or buttons[7]) else False

        A = True if buttons[0] else False
        B = True if buttons[1] else False
        X = True if buttons[2] else False
        Y = True if buttons[3] else False
        LB = True if buttons[4] else False
        RB = True if buttons[5] else False
        BACK = True if buttons[8] else False
        START = True if buttons[9] else False
        C1 = True if buttons[10] else False
        C2 = True if buttons[11] else False
        FOWARD = True if buttons[12] else False
        BACKWARD = True if buttons[13] else False
        LEFT = True if buttons[14] else False
        RIGHT = True if buttons[15] else False

        AXIS_0_LEFT = True if axis[0] == -1 else False
        AXIS_0_RIGHT = True if axis[0] == 1 else False
        AXIS_0_UP = True if axis[1] == -1 else False
        AXIS_0_DOWN = True if axis[1] == 1 else False
        AXIS_1_LEFT = True if axis[2] == -1 else False
        AXIS_1_RIGHT = True if axis[2] == 1 else False
        AXIS_1_UP = True if axis[3] == -1 else False
        AXIS_1_DOWN = True if axis[3] == 1 else False

        if LT or RT:
            if RT:
                if A:
                    self.sub_controller.set_auto_mode()
                if B:
                    self.sub_controller.set_teleop_mode()

            if LT:
                if BACK:
                    self.sub_controller.task_pause()
                if START:
                    self.sub_controller.task_resume()

        else:
            if AXIS_0_UP:
                self.sub_controller.move_forward()
            if AXIS_0_DOWN:
                self.sub_controller.move_backward()
            if AXIS_0_LEFT:
                self.sub_controller.shift_left()
            if AXIS_0_RIGHT:
                self.sub_controller.shift_right()

            if AXIS_1_LEFT:
                self.sub_controller.turn_left()
            if AXIS_1_RIGHT:
                self.sub_controller.turn_right()

            if FOWARD:
                self.sub_controller.increase_linear_speed()
            if BACKWARD:
                self.sub_controller.decrease_linear_speed()
            if LEFT:
                self.sub_controller.increase_angular_speed()
            if RIGHT:
                self.sub_controller.decrease_angular_speed()
            if RB:
                self.sub_controller.dock()
            if Y:
                self.sub_controller.undock()
            if A:
                self.sub_controller.stand()
            if X:
                self.sub_controller.sit()

            
