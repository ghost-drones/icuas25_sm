#!/usr/bin/env python3
import rclpy
from yasmin import State
from .Data_Wrapper import *
from .Control_Wrapper import *
from .Common_Ros_Node import CommonRosNode
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from sensor_msgs.msg import BatteryState

class AwaitingTrajectory(State):
    def __init__(self, node):
        super().__init__(name="AwaitingTrajectory", outcomes=["Not_Received", "Received_All"])

        self.data = Data_Wrapper()
        self.control = Control_Wrapper()
        self.node = node

    def execute(self, data):
        
        received_value = 0

        for drone_id in self.drone_ids:
            if not (drone_id not in self.poses or 
                drone_id not in self.batteries or 
                drone_id not in self.trajectories or
                drone_id not in self.trajectories_id or 
                drone_id not in self.trajectories_path):
                received_value += 1
        
        if received_value == len(self.data.get_drone_ids):
            return 'Received_All'
        else:
            return 'Not_Received'

class batteries():
    def __init__(self):
        self.data = Data_Wrapper()
        self.batteries = self.data.get_full_battery()
        self.poses = self.data.get_full_poses()
        self.drone_ids = self.data.get_drone_ids()
        self.avg_vel = 0.7
        self.decrease_battery_value_per_second = 1
        self.target = 20.0
    
    def calc_duration(self, pos_1:PoseStamped, pos_2:PoseStamped) -> float:
        if isinstance(pos_1,PoseStamped) and isinstance(pos_2,PoseStamped):
            position_01 = pos_1.pose.position
            position_02 = pos_2.pose.position
            pos_1 = np.array([position_01.x, position_01.y, position_01.z], dtype='float32')
            pos_2 = np.array([position_02.x, position_02.y, position_02.z], dtype='float32')

            duration = np.linalg.norm(pos_2 - pos_1) / self.avg_vel

            return float(duration)
      
    def it_achieve(self,destination:Point) -> float:
        
        for drone_id in self.drone_ids:
            batteries = self.batteries[drone_id]
            if isinstance(batteries,BatteryState):
                percentage = batteries.percentage
            
            pose = self.poses[drone_id]
            if isinstance(pose,PoseStamped):
                position = pose.pose.position
                duration = self.calc_duration(position,destination)
                battery_usage_forecast = duration*self.avg_vel*self.decrease_battery_value_per_second
                if (percentage - battery_usage_forecast) < self.target:
                    return False
                else:
                    return True
                
                
        
        
            
        
        
        
                
        
class Takeoff(State):
    def __init__(self, node:CommonRosNode):
        super().__init__(name="Takeoff", outcomes=["Not_Reached", "Reached"])

        self.data = Data_Wrapper()
        self.control = Control_Wrapper()
        self.node = node
        self.layer_gap = node.get_parameter_value("state_machine/layer_gap")
        self.tolerance = node.get_parameter_value("state_machine/tolerance")
        self.first_execution = True
        self.target = {}

    def execute(self, data):
        
        received_value = 0

        for drone_id in self.data.drone_ids:
            current_pose = self.data.get_pose(drone_id)

            if self.first_execution: # Envia os setpoints

                target_pose = deepcopy(current_pose)
                target_pose.pose.position.z = self.layer_gap * drone_id

                duration = self.control.calc_duration(current_pose, target_pose)

                self.control.send_takeoff(drone_id, target_pose.pose.position.z, duration_sec=duration)
                self.target[drone_id] = target_pose
            else:
                # Checa todos os setpoints para ver se já chegaram ao destino
                if self.control.is_pose_reached(current_pose, self.target[drone_id], self.tolerance):
                    received_value += 1
        
        if received_value == len(self.data.get_drone_ids):
            return 'Reached'
        else:
            return 'Not_Reached'
        
class WaypointNav(State):
    def __init__(self, node):
        super().__init__(name="WaypointNav", outcomes=["Not_Received", "Received_All"])

        self.data = Data_Wrapper()
        self.control = Control_Wrapper()
        self.node = node

    def execute(self, data):
        
        received_value = 0

        for drone_id in self.drone_ids:
            if not (drone_id not in self.poses or 
                drone_id not in self.batteries or 
                drone_id not in self.trajectories or
                drone_id not in self.trajectories_id or 
                drone_id not in self.trajectories_path):
                received_value += 1
        
        if received_value == len(self.data.get_drone_ids):
            return 'Received_All'
        else:
            return 'Not_Received'