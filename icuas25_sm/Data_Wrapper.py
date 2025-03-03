#!/usr/bin/env python3
import os
import ast
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from nav_msgs.msg import Path
from Functions_Wrapper import *

# Serviços para controle
from crazyflie_interfaces.srv import GoTo, Land, Takeoff

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

        self.go_to_clients = {}
        self.land_clients = {}
        self.takeoff_clients = {}

        # Configuração do QoS para os subscribers
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
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

    def trajectory_ids_callback(self, msg, drone_id):
        with self.data_lock:
            self.trajectories_id[drone_id] = ast.literal_eval(msg.data)

    def trajectory_path_callback(self, msg, drone_id):
        with self.data_lock:
            self.trajectories_path[drone_id] = msg.poses

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

    def get_trajectory_ids(self, drone_id):
        with self.data_lock:
            return self.trajectories_id.get(drone_id, None)

    def get_trajectory_path(self, drone_id):
        with self.data_lock:
            return self.trajectories_path.get(drone_id, None)

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
