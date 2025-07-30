# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""
Mission Replay Script.  Command-line utility to replay stored missions, including Autowalk missions.
"""

import os,sys
import time
import google.protobuf.wrappers_pb2
import bosdyn.client
import bosdyn.mission.client
import threading
from bosdyn.api.autowalk import walks_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, nav_pb2
from bosdyn.api.mission import mission_pb2, nodes_pb2
from bosdyn.client.autowalk import AutowalkResponseError

class SpotAutowalk:
    def __init__(self,robot,lease_client):
        self.robot = robot
        self.lease_client = lease_client

        self.mission_client = self.robot.ensure_client(bosdyn.mission.client.MissionClient.default_service_name)
        # self.mission_state_client = self.robot_client.ensure_client(bosdyn.mission.client.MissionClient.default_service_name)

    def get_mission_state(self):
        try:
            state = self.mission_client.get_state()
            return state
        except Exception as e:
            print(f"미션 상태를 가져오는 중 오류 발생: {e}")
            return None

    def pause_mission(self):
        if self.mission_client is not None:
            self.mission_client.pause_mission()
        else:
            print("mission_clent is none")

    def run_mission(self, robot, lease_client, fail_on_question, mission_timeout,
                    disable_directed_exploration, path_following_mode):
        """Run mission once"""
        robot.logger.info('Running mission')
        mission_state = self.mission_client.get_state()

        while mission_state.status in (mission_pb2.State.STATUS_NONE, mission_pb2.State.STATUS_RUNNING):
            # We optionally fail if any questions are triggered. This often indicates a problem in
            # Autowalk missions.
            if mission_state.questions and fail_on_question:
                robot.logger.info(
                    f'Mission failed by triggering operator question: {mission_state.questions}')
                print("Dead111111", mission_state.questions)
                return False

            body_lease = lease_client.lease_wallet.advance()
            local_pause_time = time.time() + mission_timeout

            play_settings = mission_pb2.PlaySettings(
                disable_directed_exploration=disable_directed_exploration,
                path_following_mode=path_following_mode)

            self.mission_client.play_mission(local_pause_time, [body_lease], play_settings)
            time.sleep(1)

            mission_state = self.mission_client.get_state()

        robot.logger.info(f'Mission status = {mission_state.Status.Name(mission_state.status)}')

        return mission_state.status in (mission_pb2.State.STATUS_SUCCESS,
                                        mission_pb2.State.STATUS_PAUSED)

    def restart_mission(self,mission_timeout=300):
        threading.Thread(target=lambda:self._restart_mission(mission_timeout=mission_timeout),daemon=True).start()

    def _restart_mission(self,mission_timeout=3):
        body_lease = self.lease_client.lease_wallet.advance()
        play_settings = mission_pb2.PlaySettings(
            disable_directed_exploration=True,
            path_following_mode=map_pb2.Edge.Annotations.PATH_MODE_UNKNOWN)
        local_pause_time = time.time() + mission_timeout
        self.mission_client.play_mission(local_pause_time, [body_lease], play_settings)
        time.sleep(1)
        # """Restart current mission"""
        # if self.mission_client is not None:

        #     # self.robot.logger.info('Restarting mission')
            
        #     body_lease = self.robot_client.lease_client.lease_wallet.advance()
        #     local_pause_time = time.time() + mission_timeout

        #     status = self.mission_client.restart_mission(local_pause_time, [body_lease])
        #     time.sleep(1)

        #     return status == mission_pb2.State.STATUS_SUCCESS
        # else:
        #     return False

    def handle_mission(self,value):
        threading.Thread(target=lambda:self._handle_mission(value),daemon=True).start()


    def _handle_mission(self, value):
        print(f"mission_run. value = {value}")
        self.mission_running_state = True
        if value:
            default_filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mission_files") # spot_utils/mission_files
            print(default_filepath)
            # walk_directory = default_filepath + "/test_250425_walk.walk"
            # mission_file = walk_directory + "/missions/test_250425_walk.walk"
            walk_directory = default_filepath + f"/{value}.walk"
            mission_file = walk_directory + f"/missions/{value}.walk"
            print(mission_file)
            print(walk_directory)

            graph_nav_client = self.init_mission_clients(
                self.robot, mission_file, walk_directory, do_map_load=True, disable_alternate_route_finding=False,
                upload_timeout=300)
            
            # Localize robot
            localization_error = False
            do_map_load = True
            fail_on_question = True
            do_localization = True
            static_mode = False
            duration = 0.0
            disable_directed_exploration = False
            path_following_mode = map_pb2.Edge.Annotations.PATH_MODE_UNKNOWN
            mission_timeout = 3.0

            if do_localization:
                graph = graph_nav_client.download_graph(timeout=60)
                self.robot.logger.info('Localizing robot...')
                localization = nav_pb2.Localization()

                # Attempt to localize using any visible fiducial
                graph_nav_client.set_localization(
                    initial_guess_localization=localization, ko_tform_body=None, max_distance=None,
                    max_yaw=None,
                    fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NEAREST)
            # Run mission
            if not static_mode and not localization_error:
                ### it comes here
                self.run_mission(self.robot, self.lease_client, fail_on_question,
                            mission_timeout, disable_directed_exploration,
                            path_following_mode)
            # else:
            #     self.repeat_mission(self.robot, mission_client, lease_client, args.duration, fail_on_question,
            #                     args.mission_timeout, args.disable_directed_exploration,
            #                     path_following_mode)
        self.mission_running_state = False

    def init_mission_clients(self, robot, mission_file, walk_directory, do_map_load, disable_alternate_route_finding,
                    upload_timeout):
        """Initialize clients"""

        graph_nav_client = None
        # Create autowalk and mission client
        robot.logger.info('Creating mission client...')

        

        robot.logger.info('Creating autowalk client...')
        autowalk_client = robot.ensure_client(
            bosdyn.client.autowalk.AutowalkClient.default_service_name)


        if do_map_load:

            if not os.path.isdir(walk_directory):

                robot.logger.fatal(f'Unable to find walk directory: {walk_directory}.')
                sys.exit(1)

            # Create graph-nav client
            robot.logger.info('Creating graph-nav client...')
            graph_nav_client = robot.ensure_client(
                bosdyn.client.graph_nav.GraphNavClient.default_service_name)


            # Clear map state and localization
            robot.logger.info('Clearing graph-nav state...')
            graph_nav_client.clear_graph()


            # Upload map to robot
            self.upload_graph_and_snapshots(robot, graph_nav_client, walk_directory,
                                    disable_alternate_route_finding, upload_timeout)


            # Here we assume the input file is an autowalk file so we parse it as a walk proto
            # and upload through the autowalk service.
            # If that fails we try parsing as a node and uploading through the mission service.
            try:
                self.upload_autowalk(robot, autowalk_client, mission_file, upload_timeout)

            except google.protobuf.message.DecodeError:

                robot.logger.warning(
                    f'Failed to parse autowalk proto from {mission_file}. Attempting to parse as node proto.'
                )
                self.upload_mission(robot, self.mission_client, mission_file, upload_timeout)


        else:
            # Upload mission to robot
            self.upload_mission(robot, self.mission_client, mission_file, upload_timeout)

        # Create command client
        robot.logger.info('Creating command client...')

        # Create robot state client
        robot.logger.info('Creating robot state client...')
  

        return graph_nav_client

    def upload_graph_and_snapshots(self, robot, client, path, disable_alternate_route_finding,
                                upload_timeout):
        """Upload the graph and snapshots to the robot"""

        # Load the graph from disk.
        graph_filename = os.path.join(path, 'graph')
        robot.logger.info(f'Loading graph from {graph_filename}')

        with open(graph_filename, 'rb') as graph_file:
            data = graph_file.read()
            current_graph = map_pb2.Graph()
            current_graph.ParseFromString(data)
            robot.logger.info(
                f'Loaded graph has {len(current_graph.waypoints)} waypoints and {len(current_graph.edges)} edges'
            )

        if disable_alternate_route_finding:
            for edge in current_graph.edges:
                edge.annotations.disable_alternate_route_finding = True

        # Load the waypoint snapshots from disk.
        current_waypoint_snapshots = dict()
        for waypoint in current_graph.waypoints:
            if len(waypoint.snapshot_id) == 0:
                continue
            snapshot_filename = os.path.join(path, 'waypoint_snapshots', waypoint.snapshot_id)
            robot.logger.info(f'Loading waypoint snapshot from {snapshot_filename}')

            with open(snapshot_filename, 'rb') as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot

        # Load the edge snapshots from disk.
        current_edge_snapshots = dict()
        for edge in current_graph.edges:
            if len(edge.snapshot_id) == 0:
                continue
            snapshot_filename = os.path.join(path, 'edge_snapshots', edge.snapshot_id)
            robot.logger.info(f'Loading edge snapshot from {snapshot_filename}')

            with open(snapshot_filename, 'rb') as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                current_edge_snapshots[edge_snapshot.id] = edge_snapshot

        # Upload the graph to the robot.
        robot.logger.info('Uploading the graph and snapshots to the robot...')
        true_if_empty = not len(current_graph.anchoring.anchors)
        response = client.upload_graph(graph=current_graph, generate_new_anchoring=true_if_empty,
                                    timeout=upload_timeout)
        robot.logger.info('Uploaded graph.')

        # Upload the snapshots to the robot.
        for snapshot_id in response.unknown_waypoint_snapshot_ids:
            waypoint_snapshot = current_waypoint_snapshots[snapshot_id]
            client.upload_waypoint_snapshot(waypoint_snapshot=waypoint_snapshot, timeout=upload_timeout)
            robot.logger.info(f'Uploaded {waypoint_snapshot.id}')

        for snapshot_id in response.unknown_edge_snapshot_ids:
            edge_snapshot = current_edge_snapshots[snapshot_id]
            client.upload_edge_snapshot(edge_snapshot=edge_snapshot, timeout=upload_timeout)
            robot.logger.info(f'Uploaded {edge_snapshot.id}')


    def upload_autowalk(self, robot, autowalk_client, filename, upload_timeout):
        """Upload the autowalk mission to the robot"""

        # Load the autowalk from disk
        robot.logger.info(f'Loading autowalk from {filename}')

        autowalk_proto = walks_pb2.Walk()
        with open(filename, 'rb') as walk_file:
            data = walk_file.read()
            try:
                autowalk_proto.ParseFromString(data)
            except google.protobuf.message.DecodeError as exc:
                raise exc

        # Upload the mission to the robot
        robot.logger.info('Uploading the autowalk to the robot...')
        autowalk_client.load_autowalk(autowalk_proto, timeout=upload_timeout)
        robot.logger.info('Uploaded autowalk to robot.')


    def upload_mission(self, robot, client, filename, upload_timeout):
        """Upload the mission to the robot"""

        # Load the mission from disk
        robot.logger.info(f'Loading mission from {filename}')

        mission_proto = nodes_pb2.Node()
        with open(filename, 'rb') as mission_file:
            data = mission_file.read()
            mission_proto.ParseFromString(data)

        # Upload the mission to the robot
        robot.logger.info('Uploading the mission to the robot...')
        client.load_mission(mission_proto, timeout=upload_timeout)
        robot.logger.info('Uploaded mission to robot.')
