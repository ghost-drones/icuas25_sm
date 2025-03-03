#!/usr/bin/env python3
import rclpy
from Data_Wrapper import *
from Common_Ros_Node import CommonRosNode
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
        
class WaypointNav(State):
    def __init__(self, node):
        super().__init__(outcomes=["Executing", "Low_Battery", "Finished"])

        self.drone_ids = self.data.get_drone_ids
        self.data = DataWrapper()
        self.control = ControlWrapper()
        self.layer_gap = node.get_parameter_value("layer_gap")
        self.circle_radius = node.get_parameter_value("circle_radius")
        self.node = node

        self.current_step = 0
        self.current_substep = {drone_id: 0 for drone_id in self.drone_ids}
        self.command_sent = {drone_id: False for drone_id in self.drone_ids}

        origin = Point(x=-1.0, y=-1.0, z=-1.0)

        # Variável para armazenar a resposta do ghost/path_planner
        self.path_response = origin
        self.request_traverse_origin(origin, origin, origin)

        self.save_first_value = {}
        for drone_id in self.drone_ids:
            self.save_first_value[drone_id] = None

        self.client_path_planner = self.create_client(PathService, '/ghost/path_planner')
        while not self.client_path_planner.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /ghost/path_planner not available, waiting again...')

    def execute(self, data):
        
        finished_current_step = {}

        for drone_id in self.drone_ids:
            trajetory_ids = self.data.get_trajectory_ids(drone_id)
            current_command = trajetory_ids[self.current_step]
            future_command = trajetory_ids[self.current_step+1]

            support = True if self.is_support(drone_id) == True else False
            target_pose = self.get_target_pose(drone_id, support)
            
            if support: # Ida à pontos de suporte
                target_pose = self.add_offset_2_pose(drone_id, target_pose, self.circle_radius, self.layer_gap)

                if current_command == 0: # Retorno à base
                    if self.save_first_value[drone_id] is None:
                        self.save_first_value[drone_id] = self.poses[drone_id].pose.position
                    
                    destination = self.get_target_pose(drone_id, future_command).pose.position
                    support = self.get_target_pose(drone_id, current_command).pose.position

                    self.request_traverse_origin(
                        origin=self.save_first_value[drone_id], 
                        destination=destination, 
                        support=support
                    )

                    while self.path_response.path == Point(x=-1.0, y=-1.0, z=-1.0):
                        self.request_traverse_origin(
                            origin=self.save_first_value[drone_id], 
                            destination=destination, 
                            support=support
                        )                        

                    target_pose = PoseStamped()
                    target_pose.pose.position = self.path_response.path

                    self.path_response = Point(x=-1.0, y=-1.0, z=-1.0)

            else: # Exploração
                substep = self.current_substep[drone_id]
                unique_ids = self.extract_unique_ids(trajetory_ids)
                
                if current_command[substep] in unique_ids: # Se for um ponto de suporte em lista de exploração
                    if self.current_substep[drone_id] < len(current_command) - 1: 
                        # Se ainda tiverem outros pontos na lista, ignora o ponto
                        self.current_substep[drone_id] += 1
                        finished_current_step[drone_id] = False
                    else:
                        # Se não tiverem outros pontos, finaliza o step
                        finished_current_step[drone_id] = True

            if not self.command_sent[drone_id]:
                duration = self.control.calc_duration(self.data.get_pose[drone_id], target_pose)
                self.control.send_go_to(drone_id, target_pose, duration_sec=duration)
                self.command_sent[drone_id] = True

            current_pose = self.data.get_pose(drone_id)

            if self.control.is_pose_reached(current_pose, target_pose):
                if not support:
                    if self.current_substep[drone_id] < len(current_command) - 1:
                        self.current_substep[drone_id] += 1
                        self.command_sent[drone_id] = False
                        finished_current_step[drone_id] = False
                    else:
                        finished_current_step[drone_id] = True
                else:
                    finished_current_step[drone_id] = True
            else:
                finished_current_step[drone_id] = False

        if all(finished_current_step.get(d, False) for d in self.drone_ids):
            self.current_step += 1
            for drone_id in self.drone_ids:
                self.command_sent[drone_id] = False
                self.current_substep[drone_id] = 0
        
        return "Executing"

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

    def get_target_pose(self, drone_id, support):
            
        if support: # Ida a um Suporte
            substep = 0
            flat_index = self.control.get_flat_index(drone_id, self.current_step, substep)
            target_pose = self.data.get_trajectory_path[drone_id][flat_index]

        else: # Exploração
            substep = self.current_substep[drone_id]
            flat_index = self.control.get_flat_index(drone_id, self.current_step, substep)
            target_pose = self.data.get_trajectory_path[drone_id][flat_index]
        
        return target_pose

    def is_support(self, drone_id):
        steps = self.data.get_trajectory_ids[drone_id]
        current_command = steps[self.current_step]
            
        if not isinstance(current_command, list):
            return True
        else:
            return False

    def add_offset_2_pose(self, drone_id, target_pose, circle_radius=0.5, layer_gap=0.8):

        z_offset = drone_id * layer_gap

        num_drones = len(self.drone_ids())

        angle = 2 * math.pi * (drone_id - 1) / (num_drones - 1)
        x_offset = circle_radius * math.cos(angle)
        y_offset = circle_radius * math.sin(angle)

        target_pose.pose.position.x += x_offset
        target_pose.pose.position.y += y_offset
        target_pose.pose.position.z += z_offset
        
        return target_pose

    def extract_unique_ids(self, data):
        unique_ids = []
        for values in data.values():
            for item in values:
                if not isinstance(item, list):
                    if item not in unique_ids:
                        unique_ids.append(item)
        return unique_ids

"""  
class Return2Home(State):
    def __init__(self, node):
        super().__init__(name="Return2Home", outcomes=["Executing", "Reached_Base"])

        self.data = Data_Wrapper()
        self.control = Control_Wrapper()
        self.node = node

    def execute(self, data):
        
""" 