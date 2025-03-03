#!/usr/bin/env python3
import rclpy
from Data_Wrapper import *
from geometry_msgs.msg import PoseStamped, Point
from copy import deepcopy
from smach import State
from icuas25_msgs.srv import PathService
from Functions_Wrapper import *

class AwaitingTrajectory(State):
    def __init__(self):
        super().__init__(outcomes=["Not_Received", "Received_All"])

        self.data = DataWrapper()

    def execute(self, userdata):
        received_value = 0

        for drone_id in self.data.get_drone_ids():
            pose = self.data.get_pose(drone_id)
            battery = self.data.get_battery(drone_id)
            trajectory = self.data.get_trajectory(drone_id)
            trajectory_ids = self.data.get_trajectory_ids(drone_id)
            trajectory_path = self.data.get_trajectory_path(drone_id)

            if (pose is not None and battery is not None and 
                trajectory is not None and trajectory_ids is not None and 
                trajectory_path is not None):
                received_value += 1

        if received_value == len(self.data.get_drone_ids()):
            return 'Received_All'
        else:
            return 'Not_Received'
        
class Takeoff(State):
    def __init__(self):
        super().__init__(outcomes=["Not_Reached", "Reached"])

        self.data = DataWrapper()
        self.layer_gap = 0.8 #node.get_parameter_value("layer_gap")
        self.tolerance = 1.0 #node.get_parameter_value("tolerance")
        self.first_execution = [True] * len(self.data.get_drone_ids())
        self.target = {}

    def execute(self, data):
        
        received_value = 0

        for drone_id in self.data.get_drone_ids():
            current_pose = self.data.get_pose(drone_id)

            if self.first_execution[drone_id-1]: # Envia os setpoints (-1 porque os drone_ids sempre começam com 1)

                target_pose = deepcopy(current_pose)
                target_pose.pose.position.z = self.layer_gap * drone_id

                duration = calc_duration(current_pose, target_pose)

                self.data.send_takeoff(drone_id, target_pose.pose.position.z, duration_sec=duration)
                self.target[drone_id] = target_pose

                self.first_execution[drone_id-1] = False

                return 'Not_Reached'
            else:
                # Checa todos os setpoints para ver se já chegaram ao destino
                if is_pose_reached(current_pose, self.target[drone_id], self.tolerance):
                    received_value += 1
        
        if received_value == len(self.data.get_drone_ids()):
            return 'Reached'
        else:
            return 'Not_Reached'
        
class DecideMovement(State):
    def __init__(self):
        super().__init__(outcomes=["Done"])

        self.data = DataWrapper()

    def execute(self, data):

        return 'Done'
    
class Charging(State):
    def __init__(self):
        super().__init__(outcomes=["Below_Threshold", "Above_Threshold"])

        self.data = DataWrapper()

    def execute(self, data):

        return 'Below_Threshold'
        #return 'Above_Threshold'
    
class ClusterNavSup(State):
    def __init__(self):
        super().__init__(outcomes=["Sent_Wp", "Navigating_To_Wp", "Reached_Intermediary_Step", "Reached_Base_and_Needs_Battery", "Next_Step_Exploration"])

        self.data = DataWrapper()

    def execute(self, data):

        return 'Sent_Wp'
        #return 'Navigating_To_Wp'
        #return 'Reached_Intermediary_Step'
        #return 'Reached_Base_and_Needs_Battery'
        #return 'Next_Step_Exploration'
    
class ClusterNavExp(State):
    def __init__(self):
        super().__init__(outcomes=["Sent_Wp", "Navigating_To_Wp", "Reached_Intermediary_Step", "Reached_End_Of_Cluster"])

        self.data = DataWrapper()

    def execute(self, data):

        return 'Sent_Wp'
        #return 'Navigating_To_Wp'
        #return 'Reached_Intermediary_Step'
        #return 'Reached_End_Of_Cluster'