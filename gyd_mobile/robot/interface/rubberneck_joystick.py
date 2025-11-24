import threading
import time
import re
from pubsub import pub


class RubberNeckJoyStick:
    # ---------------------------------------------------------
    # User-modifiable configuration
    # ---------------------------------------------------------
    CONFIG = {
        "duplicate_time": 0.3,        # Prevent repeated execution for this duration
        "trigger_latch_time": 0.5,    # LT/RT latch duration
    }

    # Button index mapping
    BUTTONS = {
        "A": 0, "B": 1, "X": 2, "Y": 3,
        "LB": 4, "RB": 5,
        "BACK": 8, "START": 9,
        "UP" : 12, "DOWN" : 13,
        "LEFT" : 14, "RIGHT" : 15
    }

    # Axis index mapping
    AXES = {
        "LX": 0, "LY": 1,
        "RX": 2, "RY": 3,
        "LT": 4, "RT": 5,
    }

    # LT + Button mode actions
    ACTIONS_LT = {
        "BACK":  "task_pause",
        "START": "task_resume",
    }

    # RT + Button mode actions
    ACTIONS_RT = {
        "A": "set_auto_mode",
        "B": "set_teleop_mode",
    }

    # Normal mode actions (when LT/RT are not active)
    ACTIONS_MAIN = {
        # Button actions
        "A": "stand",
        "X": "sit",
        "Y": "undock",
        "RB": "dock",
        "UP": "increase_linear_speed",
        "DOWN": "decrease_linear_speed",
        "LEFT": "decrease_angular_speed",
        "RIGHT": "increase_angular_speed",

        # Axis actions (format: "<AXIS><SIGN><VALUE>")
        "LY-1": "move_forward",
        "LY+1": "move_backward",
        "LX-1": "shift_left",
        "LX+1": "shift_right",
        "RX-1": "turn_left",
        "RX+1": "turn_right",
    }


    # ---------------------------------------------------------
    # Internal system (Users do not need to modify this section)
    # ---------------------------------------------------------
    def __init__(self, sub_controller):
        self.duplicate_queue = []
        self.trigger_latch = {"LT": False, "RT": False}
        self.sub_controller = sub_controller

        pub.subscribe(self.on_message, "receive_message")

    # ---------------------------------------------------------
    # Duplicate prevention
    # ---------------------------------------------------------
    def run_once(self, key, func):
        if key in self.duplicate_queue:
            return
        func()
        t = self.CONFIG["duplicate_time"]
        threading.Thread(target=self._unlock, args=(key, t), daemon=True).start()

    def _unlock(self, key, t):
        self.duplicate_queue.append(key)
        time.sleep(t)
        try:
            self.duplicate_queue.remove(key)
        except:
            pass

    # ---------------------------------------------------------
    # LT/RT latch system
    # ---------------------------------------------------------
    def latch_trigger(self, key):
        if self.trigger_latch[key]:
            return
        self.trigger_latch[key] = True

        def reset():
            time.sleep(self.CONFIG["trigger_latch_time"])
            self.trigger_latch[key] = False

        threading.Thread(target=reset, daemon=True).start()

    # ---------------------------------------------------------
    # Axis condition parser (Regex-based)
    # ---------------------------------------------------------
    AXIS_PATTERN = re.compile(r"^([A-Z]+)([+-])([0-9]+)$")

    def parse_axis_condition(self, cond):
        match = self.AXIS_PATTERN.match(cond)
        if not match:
            raise ValueError(f"Invalid axis condition format: {cond}")

        axis_name, sign, value_str = match.groups()
        value = int(value_str)
        expected = value if sign == "+" else -value
        return axis_name, expected

    # ---------------------------------------------------------
    # Main joystick input handler
    # ---------------------------------------------------------
    def on_message(self, message):
        topic = message.get("topic")

        if topic != "/joy":
            return

        value = message.get("value")
        axis = value["axes"]
        buttons = value["buttons"]

        # Read button and axis states
        btn = {k: bool(buttons[i]) for k, i in self.BUTTONS.items()}
        ax  = {k: axis[i] for k, i in self.AXES.items()}

        # Raw LT/RT state detection
        raw_LT = ax["LT"] != 0 or btn.get("LB", False)
        raw_RT = ax["RT"] != 0 or btn.get("RB", False)

        # Start latch when triggers are pressed
        if raw_LT:
            self.latch_trigger("LT")
        if raw_RT:
            self.latch_trigger("RT")

        LT = self.trigger_latch["LT"]
        RT = self.trigger_latch["RT"]

        # ----------------- LT mode -----------------
        if LT and not RT:
            for b, method in self.ACTIONS_LT.items():
                if btn.get(b, False):
                    self.run_once(f"LT_{b}", getattr(self.sub_controller, method))
            return

        # ----------------- RT mode -----------------
        if RT and not LT:
            for b, method in self.ACTIONS_RT.items():
                if btn.get(b, False):
                    self.run_once(f"RT_{b}", getattr(self.sub_controller, method))
            return

        # ----------------- Normal mode -----------------
        # Button-based actions
        for b, method in self.ACTIONS_MAIN.items():
            if "-" not in b and "+" not in b:
                if btn.get(b, False):
                    self.run_once(f"BTN_{b}", getattr(self.sub_controller, method))

        # Axis-based actions
        for cond, method in self.ACTIONS_MAIN.items():
            if "-" in cond or "+" in cond:
                axis_name, expected = self.parse_axis_condition(cond)

                if ax.get(axis_name) == expected:
                    getattr(self.sub_controller, method)()
