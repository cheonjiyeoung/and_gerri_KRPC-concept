import sys
import pygame
import time
import serial
import threading
joystick_btn_input = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
joystick_axis_input = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
joystick_hat_input = [(0, 0)]

# LT = joystick_axis_input[2] # default = -1 pushed = 1
# RT = joystick_axis_input[5] # default = -1 pushed = 1
# RX_Y = joystick_axis_input[4] # up = -1 down = 1
# RX_X = joystick_axis_input[3] # left = -1 right = 1
# LX_Y = joystick_axis_input[1] # up = -1 down = 1
# LX_X = joystick_axis_input[0] # left = -1 right = 1

class JoyStickReceiver:
    def __init__(self,robot_controller=None):
        self. robot_controller = robot_controller
        self.joystick = None
        self.th_joystick = None
        try:
            self.joystick_init()
        except Exception as e:
            print(f"error in initializing joystick : {e}")

    def joystick_init(self):
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        print(self.joystick.get_name())
        self.joystick.init()
        self.start()

    def start(self):
        self.th_joystick = threading.Thread(target=self.joystick_loop, daemon=True)
        self.th_joystick.start()

    def joystick_loop(self):
        while True:
            pygame.event.pump()
            speed = 0.4
            vx = 0.0
            vth = 0.0
            joystick_btn_input = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
            joystick_axis_input = [float(f"{self.joystick.get_axis(i):.1f}") for i in range(self.joystick.get_numaxes())]
            joystick_hat_input = [self.joystick.get_hat(i) for i in range(self.joystick.get_numhats())]

            LX_X, LX_Y, LT, RX_X, RX_Y, RT = joystick_axis_input
            A, B, X, Y, LB, RB, BACK, START, LOGO, LX_B, RX_B = joystick_btn_input
            ARROW_X, ARROW_Y = joystick_hat_input[0]

            if LX_Y != 0 or RX_X != 0:
                self.robot_controller.move(vx=-1 * LX_Y, vth=-1 * RX_X)
            if START:
                self.robot_controller.dock_activate()
                
            time.sleep(0.1)

if __name__ == "__main__":
    import os, sys
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
    # from gerri.robot.examples.gyd_mobile.gyd_mobile_controller import GydMobileController
    # robot_controller = GydMobileController()
    js = JoyStickReceiver()
    while True:
        time.sleep(1)