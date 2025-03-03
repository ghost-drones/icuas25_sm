#!/usr/bin/env python3
import os
import ast
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from nav_msgs.msg import Path

class DataWrapper(Node):
    _instance = None  # Instância única

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(DataWrapper, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        # Evita reinicialização
        if hasattr(self, '_initialized') and self._initialized:
            return
        super().__init__('data_wrapper_sm')
        self._initialized = True

        # Configuração dos drones
        num_robots = int(os.getenv("NUM_ROBOTS", "5"))
        self.drone_ids = list(range(1, num_robots + 1))

        # Dicionários para armazenar os dados
        self.poses = {}
        self.batteries = {}
        self.trajectories = {}
        self.trajectories_id = {}
        self.trajectories_path = {}

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        for drone_id in self.drone_ids:
            pose_topic = f'/cf_{drone_id}/pose'
            battery_topic = f'/cf_{drone_id}/battery_status'
            trajectory_topic = f'/ghost/cf_{drone_id}/trajectory_encoded'
            trajectory_ids_topic = f'/ghost/cf_{drone_id}/trajectory_ids'
            trajectory_path_topic = f'/ghost/cf_{drone_id}/trajectory_path'

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

    def pose_callback(self, msg, drone_id):
        self.poses[drone_id] = msg

    def battery_callback(self, msg, drone_id):
        self.batteries[drone_id] = msg

    def trajectory_callback(self, msg, drone_id):
        self.trajectories[drone_id] = msg.data

    def trajectory_ids_callback(self, msg, drone_id):
        self.trajectories_id[drone_id] = ast.literal_eval(msg.data)

    def trajectory_path_callback(self, msg, drone_id):
        self.trajectories_path[drone_id] = msg.poses

    def get_drone_ids(self):
        return self.drone_ids

    def get_pose(self, drone_id):
        return self.poses.get(drone_id, None)

    def get_battery(self, drone_id):
        return self.batteries.get(drone_id, None)

    def get_trajectory(self, drone_id):
        return self.trajectories.get(drone_id, None)

    def get_trajectory_ids(self, drone_id):
        return self.trajectories_id.get(drone_id, None)

    def get_trajectory_path(self, drone_id):
        return self.trajectories_path.get(drone_id, None)
