#!/usr/bin/env python3
import rclpy
from Data_Wrapper import *
from geometry_msgs.msg import PoseStamped, Point,Pose
from copy import deepcopy
from smach import State
from Functions_Wrapper import *

class AwaitingTrajectory(State):
    def __init__(self):
        super().__init__(outcomes=["Not_Received", "Received_All"])

        self.data = DataWrapper()
        self.drone_ids = self.data.get_drone_ids()

    def execute(self, userdata):
        received_value = 0

        for drone_id in self.drone_ids:
            pose = self.data.get_pose(drone_id)
            battery = self.data.get_battery(drone_id)
            trajectory = self.data.get_trajectory(drone_id)
            trajectory_ids = self.data.get_trajectory_ids(drone_id)
            trajectory_path = self.data.get_trajectory_path(drone_id)

            if (pose is not None and battery is not None and 
                trajectory is not None and trajectory_ids is not None and 
                trajectory_path is not None):
                received_value += 1

        if received_value == len(self.drone_ids):
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
        self.drone_ids = self.data.get_drone_ids()
        self.target = {}

    def execute(self, data):
        
        received_value = 0

        for drone_id in self.drone_ids:
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
        
        if received_value == len(self.drone_ids):
            return 'Reached'
        else:
            return 'Not_Reached'
        
class DecideMovement(State):
    def __init__(self):
        super().__init__(outcomes=["Done"])

        self.data = DataWrapper()
        self.battery_threshold = 30

        self.drone_ids = self.data.get_drone_ids()

    def execute(self, data):

        needs_recharging = False
        
        # Encontrar próximo Cluster
        cluster_iteration = self.data.get_clusterIteration()
        cluster_list = get_clusters_list(self.data.get_trajectory_all())
        #next_cluster = cluster_list[cluster_iteration] -> Cluster ID
        
        for drone_id in self.drone_ids:

            # Encontrar passos até próximo Cluster
            trajectory_ids = self.data.get_trajectory_ids(drone_id)
            segment_till_cluster, segment_with_return = get_trajectory_segment(cluster_iteration, trajectory_ids, cluster_list)
        
            # Calcular tempo/bateria necessária
            trajectory_poses = self.data.get_poses_by_ids_list(segment_with_return)
            battery_consumption_on_cluster = calculate_battery_consumption(trajectory_poses)
            
            # Drone conseguem fazer leitura do Cluster com bateria atual?
            current_battery = self.data.get_battery(drone_id).percentage
            battery_after_cluster = current_battery - battery_consumption_on_cluster

            # Sim (Cluster = próxima prioridade)
            self.data.next_cluster_waypoints[drone_id] = segment_till_cluster

            if battery_after_cluster < self.battery_threshold:
                needs_recharging = True
        
        if needs_recharging:
            # Não (Cluster = 0)
            segment_till_base = truncate_after_zero(segment_till_cluster)
            for drone_id in self.data.get_drone_ids():
                self.data.next_cluster_waypoints[drone_id] = segment_till_base

        return 'Done'
    
class ClusterNavSup(State):
    def __init__(self):
        super().__init__(outcomes=["Sent_Wp", "Navigating_To_Wp", "Reached_Intermediary_Step", "Reached_Base_and_Needs_Battery", "Next_Step_Exploration"])

        self.data = DataWrapper()

        self.cluster_waypoints = {}
        self.target_pose = {}
        self.support_step = 0
        self.drone_ids = self.data.get_drone_ids()
        self.command_sent = False

        # Armazena passos até a finalização
        for drone_id in self.drone_ids:
            self.cluster_waypoints[drone_id] = self.data.next_cluster_waypoints[drone_id]

        # Armazena suportes atuais e suportes destino
        cluster_iteration = self.data.get_clusterIteration()
        self.last_drone = self.drone_ids[-1]
        self.support_current_cluster, support_next_cluster = get_support_segments(self.last_drone, cluster_iteration)

        # Armazena suporte mútuo de maior grau em self.mutual_support_index
        self.mutual_support_index = 0
        for i in range(min(len(self.support_current_cluster), len(support_next_cluster))):
            if self.support_current_cluster[i] == support_next_cluster[i] and self.support_current_cluster[i] > self.mutual_support_index:
                self.mutual_support_index = self.support_current_cluster[i]

    def execute(self, data):

        # Se o próximo passo é uma lista, muda para estado exploração
        for drone_id in self.drone_ids:
            if isinstance(self.cluster_waypoints[drone_id][self.support_step], list):
                return 'Next_Step_Exploration'

        if not self.command_sent: 
            for drone_id in self.drone_ids:

                current_pose = self.data.get_pose(drone_id)

                if self.support_step == self.mutual_support_index: # Se o passo atual for aquele em que há maior suporte mútuo, buscar caminho mais rápido
                    current_destination = self.cluster_waypoints[drone_id][self.support_step]
                    current_support = self.support_current_cluster[self.support_step]

                    self.target_pose[drone_id] = self.data.request_traverse_origin(
                        origin=current_pose, 
                        destination=self.data.get_pose_by_id(current_destination).position, 
                        support=self.data.get_pose_by_id(current_support).position
                    ).path
                else:
                    self.target_pose[drone_id] = self.cluster_waypoints[drone_id][self.support_step]
                
                duration = calc_duration(current_pose, self.target_pose[drone_id])
                self.data.send_go_to(drone_id, self.target_pose[drone_id], duration_sec=duration)

            self.command_sent = True

            return 'Sent_Wp'
        else:

            finished_current_step = {}

            for drone_id in self.drone_ids:
                current_pose = self.data.get_pose(drone_id)

                if not is_pose_reached(current_pose, self.target_pose[drone_id]):
                    finished_current_step[drone_id] = True
                    
            if all(finished_current_step.get(d, False) for d in self.drone_ids):
                if self.support_step == len(self.cluster_waypoints[self.last_drone]) and self.cluster_waypoints[self.last_drone][self.support_step] == 0:
                    return 'Reached_Base_and_Needs_Battery'
                else:
                    self.support_step += 1
                    self.command_sent = False
                    return 'Reached_Intermediary_Step'
            else:
                return 'Navigating_To_Wp'
            
class ClusterNavExp(State):
    def __init__(self):
        super().__init__(outcomes=["Navigating", "Reached_End_Of_Cluster"])

        self.data = DataWrapper()

        self.cluster_waypoints = {}
        self.target_pose = {}
        self.support_step = {}
        self.command_sent = {}
        self.drone_ids = self.data.get_drone_ids()

        # Armazena passos até a finalização
        for drone_id in self.drone_ids:
            self.cluster_waypoints[drone_id] = self.data.next_cluster_waypoints[drone_id][-1]

        self.unique_ids = extract_unique_ids(self.drone_ids[-1])

    def execute(self, data):
        
        drones_reached = 0

        for drone_id in self.drone_ids:
            current_pose = self.data.get_pose(drone_id)

            if not self.command_sent[drone_id]: # Target já foi publicado?
                
                # Calcula Target
                self.target_pose[drone_id] = self.cluster_waypoints[drone_id][self.support_step]
                
                # Target é também algum suporte?
                if self.target_pose[drone_id] not in self.unique_ids:
                    duration = calc_duration(current_pose, self.target_pose[drone_id])
                    self.data.send_go_to(drone_id, self.target_pose[drone_id], duration_sec=duration)  
                    self.command_sent[drone_id] = True
                else:
                    # Final da sua lista? (Com id de suporte)
                    if self.support_step[drone_id] == len(self.cluster_waypoints[drone_id]-1):
                        drones_reached += 1
                    else:
                        self.support_step[drone_id] += 1
                        self.command_sent[drone_id] = False
            else:
                
                # Chegou ao Waypoint?
                if is_pose_reached(current_pose, self.target_pose[drone_id]):

                    # Final da sua lista?
                    if self.support_step[drone_id] == len(self.cluster_waypoints[drone_id]-1):
                        drones_reached += 1
                    else:
                        self.support_step[drone_id] += 1
                        self.command_sent[drone_id] = False
                 
        if drones_reached == len(self.cluster_waypoints[-1]):
            self.data.increase_clusterIteration()

            return "Reached_End_Of_Cluster"
        else:
            return "Navigating"
                    

class Charging(State):
    def __init__(self):
        super().__init__(outcomes=["Below_Threshold", "Above_Threshold"])

        self.data = DataWrapper()
        self.bat_lim = 90.0
        self.drone_ids = self.data.get_drone_ids()
        self.decrease_battery_value_per_second = 1
        
        battery_usage_forecast = {}
        for drone_id in self.drone_ids:
            pose_stamped = self.data.get_pose(drone_id)
            if isinstance(pose_stamped,PoseStamped):
                pose1 = pose_stamped.pose.position
                pose2 = Pose(x=pose1.x,y=pose1.y,z=0.0)
                height = pose1.z
                _duration = calc_duration(pose1,pose2)
            self.data.send_land(drone_id,height,_duration)

            trajectory_ids = self.data.get_trajectory_ids(drone_id)
            current_iteration = self.data.get_clusterIteration()
            current = get_current_trajectory_id(trajectory_ids,current_iteration)
            
            duration = 0.0
            path = []
            for element in trajectory_ids[:current]:
                if isinstance(element,int):
                    path.append(element)
                elif isinstance(element,list):
                    for el in element:
                        path.append(el)
            for pos1, pos2 in zip(path, path[1:]):
                if isinstance(pos1,int) and isinstance(pos2,int):
                    pose1 = self.data.get_pose_by_id(pos1)
                    pose2 = self.data.get_pose_by_id(pos1)
                    duration += calc_duration(pose1,pose2)
            
            battery_usage_forecast[drone_id] = duration*self.decrease_battery_value_per_second

        drone_id_max_attery_forecast = max(
            self.drone_ids, key=lambda drone_id: battery_usage_forecast[drone_id]
        )
        max_battery_usage_forecast = battery_usage_forecast[drone_id_max_attery_forecast]
        
        # garantindo que tenha no minimo 20% ao voltar
        max_battery_usage_forecast += 20.0  
        
        if max_battery_usage_forecast < 90.0 and max_battery_usage_forecast > 30.0:
            self.bat_lim = max_battery_usage_forecast
        else:
            self.bat_lim = 30.0

    def execute(self, data):
        for drone_id in self.drone_ids:
            battery = self.data.get_battery(drone_id).percentage
            if battery > self.bat_lim:  
                self.drone_ids.remove(drone_id)
            if len(self.drone_ids) == 0:
                return 'Above_Threshold'
        return 'Below_Threshold'