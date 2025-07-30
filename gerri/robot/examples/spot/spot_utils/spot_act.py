from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.docking import DockingClient, blocking_dock_robot, blocking_undock, get_dock_id
from bosdyn.client import robot_command
from bosdyn.client.power import PowerClient
from bosdyn.client.license import LicenseClient
import bosdyn.api.power_pb2 as PowerServiceProto
import bosdyn.api.robot_state_pb2 as robot_state_proto
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.geometry import EulerZXY
import time
import os,sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
# from spot_YW_config import VELOCITY_CMD_DURATION,DEFAULT_BASE_SPEED,DEFAULT_BASE_ANGULAR
# from spot_utils.grpc_utils import try_grpc, try_grpc_async
from gerri.robot.examples.spot.spot_utils.grpc_utils import try_grpc, try_grpc_async

VELOCITY_CMD_DURATION = 0.5
# 동작관련 클래스
class SpotAct:
    def __init__(self,robot_client):
        self.body_height = 0.0
        self.yaw = 0
        self.roll = 0
        self.pitch = 0
        self.mobility_params = spot_command_pb2.MobilityParams(
            locomotion_hint=spot_command_pb2.HINT_AUTO)
        self.robot_client = robot_client
        self.command_client = self.robot_client.robot.ensure_client(RobotCommandClient.default_service_name)
        self.power_client = self.robot_client.robot.ensure_client(PowerClient.default_service_name)
        
        try:
            self.estop_client = self.robot_client.robot.ensure_client(EstopClient.default_service_name)
            self.estop_endpoint = EstopEndpoint(self.estop_client, 'GNClient', estop_timeout=20, estop_cut_power_timeout=None)
        except:
            self.estop_client = None
            self.estop_endpoint = None

        self.command_input_rate = 0.1

        self._estop_keepalive = None

    def stance(self,value):
        if value == 3:
            self.stand()
        elif value == 2:
            self.sit()

    def _start_robot_command(self, desc, command_proto, end_time_secs=None):
        def _start_command():
            self.command_client.robot_command(command=command_proto,
                                                     end_time_secs=end_time_secs)
        try_grpc(desc, _start_command)

    def move(self,vx,vy,vth, desc="move"):
        self.mobility_params = RobotCommandBuilder.mobility_params(
            body_height=self.body_height,
            locomotion_hint=self.mobility_params.locomotion_hint,
            stair_hint=self.mobility_params.stair_hint
        )

        cmd = RobotCommandBuilder.synchro_velocity_command(
        v_x=vx, v_y=vy, v_rot=vth
        )

        self._start_robot_command(desc, cmd, end_time_secs=time.time() + VELOCITY_CMD_DURATION)
        
    def walk(self):
        pass

    def sit(self):
        self._start_robot_command('sit', RobotCommandBuilder.synchro_sit_command())

    def stand(self):
        print("??????????????????")
        self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())

    def action_stop(self):
        self._start_robot_command('stop', RobotCommandBuilder.stop_command())

    def toggle_power(self):
        power_state = self.power_state()
        if power_state is None:
            print('Could not toggle power because power state is unknown')
            return

        if power_state == robot_state_proto.PowerState.STATE_OFF:
            try_grpc_async('powering-on', self.request_power_on_ensure)
        else:
            try_grpc('powering-off', self.safe_power_off)

    def request_power_on(self):
        request = PowerServiceProto.PowerCommandRequest.REQUEST_ON
        return self.power_client.power_command_async(request)

    def request_power_on_ensure(self):
        request = PowerServiceProto.PowerCommandRequest.REQUEST_ON
        self.power_client.power_command_async(request)
        for i in range(10):
            motor_power = self.power_state()
            if motor_power == robot_state_proto.PowerState.STATE_ON:
                print("motor_power on")
                return True
            else:
                print(f"wait for motor on ... {i+1}sec , {motor_power == robot_state_proto.PowerState.STATE_ON}")
                time.sleep(1)
        return False

    def safe_power_off(self):
        self._start_robot_command('safe_power_off', RobotCommandBuilder.safe_power_off_command())

    def power_state(self):
        state = self.robot_client.robot_state
        if not state:
            return None
        return state.power_state.motor_power_state
    
    def toggle_estop(self):
        """toggle estop on/off. Initial state is ON"""
        if self.estop_client is not None and self.estop_endpoint is not None:
            if not self._estop_keepalive:
                self._estop_keepalive = EstopKeepAlive(self.estop_endpoint)
                return True
            else:
                try_grpc('stopping estop', self._estop_keepalive.stop)
                self._estop_keepalive.shutdown()
                self._estop_keepalive = None    
                return False

    def dock(self,undock=False):
        license_client = self.robot_client.robot.ensure_client(LicenseClient.default_service_name)
        if not license_client.get_feature_enabled([DockingClient.default_service_name
                                                ])[DockingClient.default_service_name]:
            self.robot_client.robot.logger.error('This robot is not licensed for docking.')
            return False
        if undock:
            power_state = self.power_state()
            if power_state == robot_state_proto.PowerState.STATE_OFF:
                self.request_power_on_ensure()
            dock_id = get_dock_id(self.robot_client.robot)
            if dock_id is None:
                print('Robot does not seem to be docked; trying anyway')
            else:
                print(f'Docked at {dock_id}')
            blocking_undock(self.robot_client.robot)
            print('Undocking Success')
        else:
            dock_id = self.robot_client.spot_camera.get_fiducial_objects(dock=True)
            if dock_id is not None:
                # Stand before trying to dock.
                robot_command.blocking_stand(self.command_client)
                blocking_dock_robot(self.robot_client.robot, dock_id)
                print('Docking Success')
            else:
                print("cant find qr-code")
    
    def set_stance(self,stance_params,desc='set_stance'):
        if self.robot_client.status.mode == "WALK":
            yaw = stance_params.get("yaw", 0)
            roll = stance_params.get("roll", 0)
            pitch = stance_params.get("pitch", 0) 
            reset = stance_params.get("reset", 0)
            if reset:
                orientation = EulerZXY(0, 0, 0)
                self.yaw = 0
                self.roll = 0
                self.pitch = 0
            else:
                orientation = EulerZXY(yaw+self.yaw,roll+self.roll, pitch+self.pitch)
                self.yaw = yaw+self.yaw
                self.roll = roll+self.roll
                self.pitch = pitch+self.pitch
            try:
                cmd = RobotCommandBuilder.synchro_stand_command(body_height=0.0,
                                                                footprint_R_body=orientation)
                self._start_robot_command(desc,cmd, end_time_secs=time.time() + VELOCITY_CMD_DURATION)

            except Exception as e:
                print(f"{e}")