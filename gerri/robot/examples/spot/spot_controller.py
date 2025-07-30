from bosdyn.client import create_standard_sdk
from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_state import RobotStateClient
from threading import Thread
import bosdyn.mission.client
from bosdyn.client.exceptions import ResponseError
# import graph_nav_util
import time
import os,sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.examples.spot.spot_utils.spot_act import SpotAct
from gerri.robot.examples.spot.spot_utils.spot_camera import SpotCamera
from gerri.robot.examples.spot.spot_utils.spot_autowalk import SpotAutowalk

# from spot_utils.spot_act import SpotAct
# from spot_YW_config import SPOT_ADDR,SPOT_ID,SPOT_PWD,USING_ARM,GRAPH_PATH
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.api import world_object_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, nav_pb2

import logging, pprint
LOGGER = logging.getLogger()

class AsyncRobotState(AsyncPeriodicQuery):

    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__('robot_state', robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()
    
    
# 사용권, 클라이언트 객체
class SpotController:
    def __init__(self,spot_info):
        self.sdk = create_standard_sdk('main', [bosdyn.mission.client.MissionClient])

        if spot_info["using_ptz"]:
            from bosdyn.client import spot_cam
            spot_cam.register_all_service_clients(self.sdk)
        self.robot = self.sdk.create_robot(spot_info["spot_ip"])
        self.robot.authenticate(spot_info["spot_id"],spot_info["spot_pwd"])
        self.robot.time_sync.wait_for_sync()
        # print(self.robot.get_cached_robot_state())

        self._graph_nav_client = self.robot.ensure_client(GraphNavClient.default_service_name)
        # Store the most recent knowledge of the state of the robot based on rpc calls.
        self._current_graph = None
        self._current_edges = dict()  #maps to_waypoint to list(from_waypoint)
        self._current_waypoint_snapshots = dict()  # maps id to waypoint snapshot
        self._current_edge_snapshots = dict()  # maps id to edge snapshot
        self._current_annotation_name_to_wp_id = dict()

        self.lease_client =             self.robot.ensure_client(LeaseClient.default_service_name)
        self.robot_state_client =       self.robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_state_task = AsyncRobotState(self.robot_state_client)
        self._running = True
        
        self.spot_act = SpotAct(self)
        self.spot_camera = SpotCamera(self.robot,spot_info=spot_info)
        self.spot_autowalk = SpotAutowalk(self.robot,self.lease_client)

        self.robot_id = None
        self.lease_keepalive = None

        # self.upload_graph_and_snapshots()

    def connect(self):
        self.initialize()

    def initialize(self):
        self.toggle_lease()
        self.start()
        res = self.spot_act.toggle_estop()
        power = self.spot_act.request_power_on_ensure()
        if not power:
            print("can not on motor power")
            sys.exit()

    def get_latest_state(self):
        return self._robot_state_task.proto  # or .response, depending on your need

    def stop(self):
        self._running = False
        self._state_thread.join()

    def toggle_lease(self):
        try:
            self.lease_keepalive.shutdown()
            self.lease_keepalive = None            
        except:
            pass
        """toggle lease acquisition. Initial state is acquired"""
        if self.lease_client is not None:
            try:
                if self.lease_keepalive is None:
                    self.lease_keepalive = LeaseKeepAlive(self.lease_client, must_acquire=True,
                                                        return_at_exit=True)
                else:
                    self.lease_keepalive.shutdown()
                    self.lease_keepalive = None
            except Exception as e:
                print(f"error : {e}")

    def start(self):
        """Begin communication with the robot."""
        # Construct our lease keep-alive object, which begins RetainLease calls in a thread.
        self.lease_keepalive = LeaseKeepAlive(self.lease_client, must_acquire=True,
                                               return_at_exit=True)

        self.robot_id = self.robot.get_id()
        if self.spot_act.estop_client is not None:
            self.spot_act.estop_endpoint.force_simple_setup(
            )  # Set this endpoint as the robot's sole estop.

    @property
    def robot_state(self):
        """Get latest robot state proto."""
        return self._robot_state_task.proto

    """
    estop -> power -> stand
        self._command_dictionary = {
            ord('b'): self._battery_change_pose,
        }"""
    def upload_graph_and_snapshots(self, *args):
        print('Loading the graph from disk into local storage...')
        with open(GRAPH_PATH + '/graph', 'rb') as graph_file:
            # Load the graph from disk.
            data = graph_file.read()
            self._current_graph = map_pb2.Graph()
            self._current_graph.ParseFromString(data)
            print(
                f'Loaded graph has {len(self._current_graph.waypoints)} waypoints and {len(self._current_graph.edges)} edges'
            )
        for waypoint in self._current_graph.waypoints:
            # Load the waypoint snapshots from disk.
            with open(f'{GRAPH_PATH}/waypoint_snapshots/{waypoint.snapshot_id}',
                      'rb') as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                self._current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot
        for edge in self._current_graph.edges:
            if len(edge.snapshot_id) == 0:
                continue
            # Load the edge snapshots from disk.
            with open(f'{GRAPH_PATH}/edge_snapshots/{edge.snapshot_id}',
                      'rb') as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                self._current_edge_snapshots[edge_snapshot.id] = edge_snapshot
        # Upload the graph to the robot.
        print('Uploading the graph and snapshots to the robot...')
        true_if_empty = not len(self._current_graph.anchoring.anchors)
        response = self._graph_nav_client.upload_graph(graph=self._current_graph,
                                                       generate_new_anchoring=true_if_empty)
        # Upload the snapshots to the robot.
        for snapshot_id in response.unknown_waypoint_snapshot_ids:
            waypoint_snapshot = self._current_waypoint_snapshots[snapshot_id]
            self._graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot)
            print(f'Uploaded {waypoint_snapshot.id}')

        for snapshot_id in response.unknown_edge_snapshot_ids:
            edge_snapshot = self._current_edge_snapshots[snapshot_id]
            self._graph_nav_client.upload_edge_snapshot(edge_snapshot)
            print(f'Uploaded {edge_snapshot.id}')

        # The upload is complete! Check that the robot is localized to the graph,
        # and if it is not, prompt the user to localize the robot before attempting
        # any navigation commands.
        localization_state = self._graph_nav_client.get_localization_state()
        if not localization_state.localization.waypoint_id:
            # The robot is not localized to the newly uploaded graph.
            print('\n')
            print(
                'Upload complete! The robot is currently not localized to the map; please localize'
                ' the robot need initial with fiducial or initial with waypoint before attempting a navigation command.')


    def get_localization_state(self, *args):
        """from initial pose(at booting)."""
        state = self._graph_nav_client.get_localization_state(request_gps_state=False)
        # print(f'Got localization: \n{state.localization}')
        loc = state.localization
        try:
            self.pose["x"] = loc.seed_tform_body.position.x
            self.pose["y"] = loc.seed_tform_body.position.y
            self.pose["z"] = loc.seed_tform_body.position.z
        except Exception as e:
            print(e)
        # odom_tform_body = get_odom_tform_body(state.robot_kinematics.transforms_snapshot)
        # print(f'Got robot state in kinematic odometry frame: \n{odom_tform_body}')

    def set_initial_localization_fiducial(self, *args):
        """Trigger localization when near a fiducial."""
        robot_state = self.robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot).to_proto()
        # Create an empty instance for initial localization since we are asking it to localize
        # based on the nearest fiducial.
        localization = nav_pb2.Localization()
        self._graph_nav_client.set_localization(initial_guess_localization=localization,
                                                ko_tform_body=current_odom_tform_body)

    def _navigate_to_anchor(self, *args):
        """Navigate to a pose in seed frame, using anchors."""
        # The following options are accepted for arguments: [x, y], [x, y, yaw], [x, y, z, yaw],
        # [x, y, z, qw, qx, qy, qz].
        # When a value for z is not specified, we use the current z height.
        # When only yaw is specified, the quaternion is constructed from the yaw.
        # When yaw is not specified, an identity quaternion is used.

        if len(args) < 1 or len(args[0]) not in [2, 3, 4, 7]:
            print('Invalid arguments supplied.')
            return

        seed_T_goal = SE3Pose(float(args[0][0]), float(args[0][1]), 0.0, Quat())

        if len(args[0]) in [4, 7]:
            seed_T_goal.z = float(args[0][2])
        else:
            localization_state = self._graph_nav_client.get_localization_state()
            if not localization_state.localization.waypoint_id:
                print('Robot not localized')
                return
            seed_T_goal.z = localization_state.localization.seed_tform_body.position.z

        if len(args[0]) == 3:
            seed_T_goal.rot = Quat.from_yaw(float(args[0][2]))
        elif len(args[0]) == 4:
            seed_T_goal.rot = Quat.from_yaw(float(args[0][3]))
        elif len(args[0]) == 7:
            seed_T_goal.rot = Quat(w=float(args[0][3]), x=float(args[0][4]), y=float(args[0][5]),
                                z=float(args[0][6]))

        if not self.toggle_power(should_power_on=True):
            print('Failed to power on the robot, and cannot complete navigate to request.')
            return
        nav_to_cmd_id = None
        # Navigate to the destination.
        is_finished = False
        while not is_finished:
            # Issue the navigation command about twice a second such that it is easy to terminate the
            # navigation command (with estop or killing the program).
            try:
                nav_to_cmd_id = self._graph_nav_client.navigate_to_anchor(
                    seed_T_goal.to_proto(), 1.0, command_id=nav_to_cmd_id)
            except ResponseError as e:
                print(f'Error while navigating {e}')
                break
            time.sleep(.5)  # Sleep for half a second to allow for command execution.
            # Poll the robot for feedback to determine if the navigation command is complete. Then sit
            # the robot down once it is finished.
            is_finished = self._check_success(nav_to_cmd_id)

        # Power off the robot if appropriate.
        if self._powered_on and not self._started_powered_on:
            # Sit the robot down + power off after the navigation command is complete.
            self.toggle_power(should_power_on=False)

if __name__ == "__main__":
    from pynput import keyboard
    test = SpotController()
    # test.spot_act.move(move_params={"vx":0.5})
    def on_press(key):
        try:
            print(f'Key {key.char} pressed.')
            if key.char == "i":
                test.spot_act.stand()
            elif key.char == "k":
                test.spot_act.sit()
            elif key.char == "w":
                test.spot_act.move(move_params={"vx":0.5})
            elif key.char == "s":
                test.spot_act.move(move_params={"vx":-0.5})
            elif key.char == "a":
                test.spot_act.move(move_params={"vy":0.5})
            elif key.char == "d":
                test.spot_act.move(move_params={"vy":-0.5})
            elif key.char == "q":
                test.spot_act.move(move_params={"vth":0.5})
            elif key.char == "e":
                test.spot_act.move(move_params={"vth":-0.5})
            elif key.char == "o":
                print(test.robot_state)
            elif key.char == "p":
                test.spot_act.docking()
            elif key.char == "l":
                test.spot_act.docking(undock=True)
            elif key.char == "t":
                test.spot_act.set_stance(stance_params={"pitch":0.1})
            elif key.char == "g":
                test.spot_act.set_stance(stance_params={"pitch":-0.1})
            elif key.char == "f":
                test.spot_act.set_stance(stance_params={"roll":0.1})
            elif key.char == "h":
                test.spot_act.set_stance(stance_params={"roll":-0.1})
            elif key.char == "r":
                test.spot_act.set_stance(stance_params={"yaw":0.1})
            elif key.char == "y":
                test.spot_act.set_stance(stance_params={"yaw":-0.1})
            elif key.char == "m":
                test.spot_act.set_stance(stance_params={"reset":True})
            elif key.char == "z":
                test.get_localization_state()
            elif key.char == "x":
                test.set_initial_localization_fiducial()

        except Exception as e:
            print(e)
        # except AttributeError:
        #     print(f'Special key {key} pressed.')
        # except:
        #     pass

    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    
    try:
        while True:
            time.sleep(1)
            # print(test.robot_state)

    except KeyboardInterrupt:
        try:
            listener.stop()
            if hasattr(test, "lease_keepalive") and test.lease_keepalive:
                test.lease_keepalive.shutdown()
                print("lease shutdown good.")
        except Exception as e:
            print(f"Failed to shutdown lease: {e}")