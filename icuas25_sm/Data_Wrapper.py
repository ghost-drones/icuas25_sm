#!/usr/bin/env python3
import os
import ast
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Pose, Point
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from nav_msgs.msg import Path
from Functions_Wrapper import *
from icuas25_msgs.msg import Waypoints

# Serviços para controle
from crazyflie_interfaces.srv import GoTo, Land, Takeoff
from icuas25_msgs.srv import PathService

import threading

class DataWrapper(Node):
    _instance = None  # Singleton

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(DataWrapper, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        # Evita reinicialização (singleton)
        if hasattr(self, '_initialized') and self._initialized:
            return
        super().__init__('data_wrapper_sm')
        self._initialized = True

        # Lock para sincronização
        self.data_lock = threading.Lock()

        # Configuração dos drones
        num_robots = int(os.getenv("NUM_ROBOTS", "5"))
        self.drone_ids = list(range(1, num_robots + 1))

        # Dicionários para armazenar os dados dos tópicos
        self.poses = {}
        self.batteries = {}
        self.trajectories = {}
        self.trajectories_id = {}
        self.trajectories_path = {}
        self.trajectories_all = ""

        self.go_to_clients = {}
        self.land_clients = {}
        self.takeoff_clients = {}
        self.next_cluster_waypoints = {}

        self.clusterIteration = 0

        self.client_path_planner = self.create_client(PathService, '/ghost/path_planner')
        while not self.client_path_planner.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /ghost/path_planner not available, waiting again...')
            
        # Configuração do QoS para os subscribers
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.create_subscription(
            String,
            '/ghost/trajectory_encoded',
            lambda msg, id=drone_id: self.trajectory_all_callback(msg, id),
            qos_profile
        )

        self.create_subscription(
            Waypoints,
            '/ghost/waypoints',
            self.waypoints_callback,
            10
        )

        # Subscriptions para cada drone
        for drone_id in self.drone_ids:
            pose_topic = f'/cf_{drone_id}/pose'
            battery_topic = f'/cf_{drone_id}/battery_status'
            trajectory_topic = f'/ghost/cf_{drone_id}/trajectory_encoded'
            trajectory_ids_topic = f'/ghost/cf_{drone_id}/trajectory_ids'
            trajectory_path_topic = f'/ghost/cf_{drone_id}/trajectory_path'

            go_to_service = f'/cf_{drone_id}/go_to'
            land_service = f'/cf_{drone_id}/land'
            takeoff_service = f'/cf_{drone_id}/takeoff'

            self.go_to_clients[drone_id] = self.create_client(GoTo, go_to_service)
            self.land_clients[drone_id] = self.create_client(Land, land_service)
            self.takeoff_clients[drone_id] = self.create_client(Takeoff, takeoff_service)

            self.create_subscription(
                PoseStamped,
                pose_topic,
                lambda msg, id=drone_id: self.pose_callback(msg, id),
                qos_profile
            )
            self.create_subscription(
                BatteryState,
                battery_topic,
                lambda msg, id=drone_id: self.battery_callback(msg, id),
                qos_profile
            )
            self.create_subscription(
                String,
                trajectory_topic,
                lambda msg, id=drone_id: self.trajectory_callback(msg, id),
                qos_profile
            )
            self.create_subscription(
                String,
                trajectory_ids_topic,
                lambda msg, id=drone_id: self.trajectory_ids_callback(msg, id),
                qos_profile
            )
            self.create_subscription(
                Path,
                trajectory_path_topic,
                lambda msg, id=drone_id: self.trajectory_path_callback(msg, id),
                qos_profile
            )
            self.get_logger().info(f'Subscribed to topics for drone {drone_id}')

    # Callbacks dos subscribers
    def pose_callback(self, msg, drone_id):
        with self.data_lock:
            self.poses[drone_id] = msg

    def battery_callback(self, msg, drone_id):
        with self.data_lock:
            self.batteries[drone_id] = msg

    def trajectory_callback(self, msg, drone_id):
        with self.data_lock:
            self.trajectories[drone_id] = msg.data

    def trajectory_all_callback(self, msg):
        with self.data_lock:
            self.trajectories_all = msg.data

    def trajectory_ids_callback(self, msg, drone_id):
        with self.data_lock:
            self.trajectories_id[drone_id] = ast.literal_eval(msg.data)

    def trajectory_path_callback(self, msg, drone_id):
        with self.data_lock:
            self.trajectories_path[drone_id] = msg.poses

    def waypoints_callback(self, msg):
        with self.data_lock:
            self.waypoints = msg
        
    # Métodos de acesso aos dados
    def get_drone_ids(self):
        return self.drone_ids

    def get_pose(self, drone_id):
        with self.data_lock:
            return self.poses.get(drone_id, None)

    def get_battery(self, drone_id):
        with self.data_lock:
            return self.batteries.get(drone_id, None)

    def get_trajectory(self, drone_id):
        with self.data_lock:
            return self.trajectories.get(drone_id, None)

    def get_trajectory_all(self):
        with self.data_lock:
            return self.trajectories_all
        
    def get_trajectory_ids(self, drone_id):
        with self.data_lock:
            return self.trajectories_id.get(drone_id, None)

    def get_trajectory_path(self, drone_id):
        with self.data_lock:
            return self.trajectories_path.get(drone_id, None)

    def get_clusterIteration(self):
        with self.data_lock:
            return self.clusterIteration
        
    def get_waypoints(self):
        with self.data_lock:
            return self.waypoints
    
    def update_clusterIteration(self):
        with self.data_lock:
            self.clusterIteration += 1

    def send_takeoff(self, drone_id, height, duration_sec, group_mask=0):
        client = self.takeoff_clients[drone_id]
        request = Takeoff.Request()
        request.group_mask = group_mask
        request.height = height
        request.duration.sec = int(duration_sec)
        request.duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        client.call_async(request)

    def send_go_to(self, drone_id, pose: PoseStamped, duration_sec, group_mask=0, relative=False):
        client = self.go_to_clients[drone_id]
        yaw = get_yaw_from_pose(pose)
        request = GoTo.Request()
        request.group_mask = group_mask
        request.relative = relative
        request.goal = pose.pose.position
        request.yaw = yaw
        request.duration.sec = int(duration_sec)
        request.duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        client.call_async(request)

    def send_land(self, drone_id, height, duration_sec, group_mask=0):
        client = self.land_clients[drone_id]
        request = Land.Request()
        request.group_mask = group_mask
        request.height = height
        request.duration.sec = int(duration_sec)
        request.duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        client.call_async(request)

    def get_pose_by_id(self, waypoint_id: int) -> Pose:

        if waypoint_id == 0:
            pose = Pose()
            pose.position.x = 0.0
            pose.position.y = 0.0
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            return pose

        for waypoint in self.waypoints:
            if waypoint.id == waypoint_id:
                return waypoint.pose

        raise ValueError(f"Waypoint com id {waypoint_id} não encontrado.")

    def get_poses_by_ids_list(self, waypoint_ids: list) -> list:

        poses = []
        for wid in waypoint_ids:
            pose = self.get_pose_by_id(wid)
            poses.append(pose)
        return poses

    def request_traverse_origin(self, origin: Point, destination: Point, support: Point):
        request = PathService.Request()
        request.origin = Point(x=origin.x, y=origin.y, z=origin.z)
        request.destination = Point(x=destination.x, y=destination.y, z=destination.z)
        request.support = Point(x=support.x, y=support.y, z=support.z)

        future = self.client_path_planner.call_async(request)
        future.add_done_callback(self.handle_path_response)

    def handle_path_response(self, future):
        try:
            self.path_response = future.result()
        except Exception as e:
            self.get_logger().error(f"Ghost/path_planner service call failed: {e}")