import requests
import time

class ReemanMobileController:
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.base_url = f"http://{robot_ip}"
        self.status = 'disconnected'

    def _post_request(self, endpoint, data=None):
        url = f"{self.base_url}/{endpoint}"
        try:
            response = requests.post(url, json=data, timeout=5)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Error with POST request to {endpoint}: {e}")
            return None

    def _get_request(self, endpoint):
        url = f"{self.base_url}/{endpoint}"
        try:
            response = requests.get(url, timeout=5)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Error with GET request to {endpoint}: {e}")
            return None

    def connect(self):
        response = self._get_request("gyd_mobile/current_version")
        if response:
            self.status = 'connected'
            print(f"Connected to robot, version: {response.get('version', 'unknown')}")
        else:
            print("Failed to connect to the robot.")

    def disconnect(self):
        self.status = 'disconnected'
        print("Disconnected from robot.")

    def get_pose(self):
        response = self._get_request("gyd_mobile/pose")
        if response:
            print(f"Current pose: x={response['x']}, y={response['y']}, theta={response['theta']}")
            return response
        return None

    def move(self, distance, direction, speed):
        data = {
            "distance": distance,  # cm
            "direction": direction,  # 0 for backward, 1 for forward
            "speed": speed  # m/s
        }
        response = self._post_request("cmd/move", data)
        if response:
            print(f"Move command successful: {response}")
        else:
            print("Move command failed.")

    def joy(self, linear_speed, angular_speed):
        data = {
            "vx": linear_speed,  # m/s
            "vth": angular_speed  # rad/s
        }
        response = self._post_request("cmd/speed", data)
        if response:
            print(f"Set speed successful: {response}")
        else:
            print("Set speed failed.")

    def turn(self, angle, direction, speed):
        data = {
            "direction": direction,  # 1 for left, 0 for right
            "angle": angle,  # degrees
            "speed": speed  # rad/s
        }
        response = self._post_request("cmd/turn", data)
        if response:
            print(f"Turn command successful: {response}")
        else:
            print("Turn command failed.")

    def get_battery_status(self):
        response = self._get_request("gyd_mobile/base_encode")
        if response:
            battery = response.get("battery", "unknown")
            charging_status = response.get("chargeFlag", "unknown")
            print(f"Battery: {battery}%, Charging Status: {charging_status}")
            return response
        return None

    def stop(self):
        self.set_speed(0, 0)
        print("Robot stopped.")

# Example usage
if __name__ == "__main__":
    robot_ip = "192.168.0.10"  # Replace with your robot's IP address
    controller = ReemanMobileController(robot_ip)

    controller.connect()
    time.sleep(1)

    controller.get_pose()
    controller.move(100, 1, 0.5)  # Move forward 100 cm at 0.5 m/s
    time.sleep(2)

    controller.turn(90, 1, 0.5)  # Turn left 90 degrees at 0.5 rad/s
    time.sleep(2)

    controller.get_battery_status()
    controller.stop()

    controller.disconnect()
