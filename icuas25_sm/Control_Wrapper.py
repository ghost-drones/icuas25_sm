#!/usr/bin/env python3
import os
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.srv import GoTo, Land, Takeoff
from tf_transformations import euler_from_quaternion

class ControlWrapper(Node):
    _instance = None  # Instância única

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(ControlWrapper, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        # Evita reinicialização
        if hasattr(self, '_initialized') and self._initialized:
            return
        self._initialized = True

        super().__init__('control_wrapper_node')

        num_robots = int(os.getenv("NUM_ROBOTS", "5"))
        drone_ids = list(range(1, num_robots + 1))
        self.drone_ids = drone_ids

        self.go_to_clients = {}
        self.land_clients = {}
        self.takeoff_clients = {}

        for drone_id in self.drone_ids:
            go_to_service = f'/cf_{drone_id}/go_to'
            land_service = f'/cf_{drone_id}/land'
            takeoff_service = f'/cf_{drone_id}/takeoff'

            self.go_to_clients[drone_id] = self.create_client(GoTo, go_to_service)
            self.land_clients[drone_id] = self.create_client(Land, land_service)
            self.takeoff_clients[drone_id] = self.create_client(Takeoff, takeoff_service)

    def is_pose_reached(self, current_pose: PoseStamped, target_pose: PoseStamped, tolerance=1.0) -> bool:
        dx = current_pose.pose.position.x - target_pose.pose.position.x
        dy = current_pose.pose.position.y - target_pose.pose.position.y
        dz = current_pose.pose.position.z - target_pose.pose.position.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        return distance < tolerance

    def get_yaw_from_pose(self, pose: PoseStamped) -> float:
        quat = [pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(quat)
        return yaw

    def calc_duration(self, pos_1: PoseStamped, pos_2: PoseStamped, avr_vel=1.2) -> float:
        p1 = np.array([pos_1.pose.position.x, pos_1.pose.position.y, pos_1.pose.position.z])
        p2 = np.array([pos_2.pose.position.x, pos_2.pose.position.y, pos_2.pose.position.z])
        return np.linalg.norm(p2 - p1) / avr_vel

    def get_flat_index(self, drone_id, step, substep):
        steps = self.trajectories_id[drone_id]
        flat_index = 0
        for i in range(step):
            elem = steps[i]
            if isinstance(elem, list):
                flat_index += len(elem)
            else:
                flat_index += 1
        current_elem = steps[step]
        if isinstance(current_elem, list):
            flat_index += substep
        else:
            flat_index += 0
        return flat_index
    
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

        yaw = self.get_yaw_from_pose(pose)
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