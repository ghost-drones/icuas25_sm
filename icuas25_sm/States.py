#!/usr/bin/env python3
import rclpy
from Data_Wrapper import *
from Control_Wrapper import *
from geometry_msgs.msg import PoseStamped, Point,Pose
from copy import deepcopy
from smach import State
from Functions_Wrapper import *

class AwaitingTrajectory(State):
    def __init__(self):
        super().__init__(outcomes=["Not_Received", "Received_All"])

        self.data = DataWrapper()
        self.control = ControlWrapper()
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
            self.control.publish_debug("AwaitingTrajectory() concluído.")
            return 'Received_All'
        else:
            return 'Not_Received'
        
class Takeoff(State):
    def __init__(self):
        super().__init__(outcomes=["Not_Reached", "Reached"])

        self.data = DataWrapper()
        self.control = ControlWrapper()
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
            self.control.publish_debug("Takeoff() concluído.")
            self.first_execution = [True] * len(self.data.get_drone_ids())
            return 'Reached'
        else:
            return 'Not_Reached'
        
class DecideMovement(State):
    def __init__(self):
        super().__init__(outcomes=["Finished"])

        self.data = DataWrapper()
        self.control = ControlWrapper()

        self.drone_ids = self.data.get_drone_ids()

    def execute(self, data):

        needs_recharging = False
        
        # Encontrar próximo Cluster
        cluster_iteration = self.data.get_clusterIteration()
        cluster_indexes = get_clusters_indexes_on_traj_ids(self.data.get_trajectory_all())
        self.control.publish_debug("DecideMovement(): " + str(self.data.get_trajectory_all()))
        next_cluster = cluster_indexes[cluster_iteration]
        last_drone_traj_ids = self.data.get_trajectory_ids(self.drone_ids[-1])
        #self.control.publish_debug("DecideMovement(): " + str(cluster_iteration) + str(cluster_indexes) + str(next_cluster)+ str(last_drone_traj_ids))

        for drone_id in self.drone_ids:

            # Encontrar passos até próximo Cluster
            trajectory_ids = self.data.get_trajectory_ids(drone_id)

            segment_till_cluster, ___ = get_trajectory_segment(cluster_iteration, trajectory_ids, cluster_indexes)
            segment_till_cluster_next, ___ = get_trajectory_segment(cluster_iteration+1, trajectory_ids, cluster_indexes)

            # Sim (Cluster = próxima prioridade)
            self.data.next_cluster_waypoints[drone_id] = segment_till_cluster
        
        if self.data.need_charging:
            # Não (Cluster = 0)
            segment_till_base = truncate_after_zero(segment_till_cluster_next)
            for drone_id in self.data.get_drone_ids():
                self.data.next_cluster_waypoints[drone_id] = segment_till_base
            
            self.control.publish_debug("DecideMovement(): Voltar para base e carregar. Waypoints necessários: " + str(self.data.next_cluster_waypoints[drone_id]))
        else:
            self.control.publish_debug(f"DecideMovement(): Ida para Cluster. Waypoints necessários: " + str(self.data.next_cluster_waypoints[drone_id][:-1]))
        
        return 'Finished'

class ClusterNavSup(State):
    def __init__(self):
        super().__init__(outcomes=["Sent_Wp", "Navigating_To_Wp", "Reached_Intermediary_Step", 
                                     "Reached_Base_and_Needs_Battery", "Next_Step_Exploration"])
        self.data = DataWrapper()
        self.control = ControlWrapper()
        self.drone_ids = self.data.get_drone_ids()
        # Inicialmente, vamos carregar os waypoints do cluster para uso interno.
        self.cluster_waypoints = {}
        self.target_pose = {drone_id: None for drone_id in self.drone_ids}
        self.support_step = 0
        self.command_sent = False
        self.first_execution = True
        self.layer_gap = 0.5
        self.hor_offset = 2.0

    def execute(self, data):
        if self.first_execution:

            # Atualiza os waypoints do cluster para uso interno.
            self.cluster_waypoints = deepcopy(self.data.next_cluster_waypoints)
            
            # Obtém os segmentos de suporte para o cluster atual.
            cluster_iteration = self.data.get_clusterIteration()
            self.last_drone = self.drone_ids[-1]
            last_drone_traj_ids = self.data.get_trajectory_ids(self.last_drone)
            self.support_current_cluster, support_next_cluster = get_support_segments(last_drone_traj_ids, cluster_iteration)
            
            # Determina o índice de maior suporte mútuo.
            self.mutual_support_index = 0

            mutual_id = calculate_mutual_id(self.support_current_cluster, support_next_cluster)
            
            self.mutual_support_index = calculate_mutual_index(mutual_id, self.data.next_cluster_waypoints[self.last_drone])

            self.control.publish_debug(f"ClusterNavSup(): Maior suporte mútuo {mutual_id}. Index: {self.mutual_support_index}")
            self.first_execution = False

        # Se o próximo waypoint para o último drone for uma lista, muda para o estado de exploração.
        if isinstance(self.data.next_cluster_waypoints[self.last_drone][self.support_step], list):
            self.support_step = 0
            self.first_execution = True
            return 'Next_Step_Exploration'

        back2origin = True if not isinstance(self.data.next_cluster_waypoints[self.last_drone][-1], list) else False
        
        # Se o comando ainda não foi enviado, calcula e envia os setpoints para cada drone.
        if not self.command_sent:
            for drone_id in self.drone_ids:
                current_pose = self.data.get_pose(drone_id)

                # Se suporte mútuo, envia o ponto traversado que possui LOS com a origem. (E se não estiver voltando pra base)
                if self.support_step == self.mutual_support_index and back2origin == False:
                    current_destination = self.data.next_cluster_waypoints[drone_id][self.support_step+1]
                    current_support = self.data.next_cluster_waypoints[drone_id][self.support_step]

                    self.target_pose[drone_id] = self.control.request_traverse_origin_sync(drone_id=drone_id,
                        origin=current_pose.pose.position,
                        destination=self.data.get_pose_by_id(current_destination).position,
                        support=self.data.get_pose_by_id(current_support).position
                    ).path
                else:
                    
                    if back2origin == True and self.data.next_cluster_waypoints[drone_id][self.support_step] == 0:
                        self.target_pose[drone_id] = add_offset_2_pose(len(self.drone_ids), drone_id, self.data.get_pose_by_id(self.data.next_cluster_waypoints[drone_id][self.support_step]), 0.8, 0.5, 0, back2origin)
                    else:
                        self.target_pose[drone_id] = add_offset_2_pose(len(self.drone_ids), drone_id, self.data.get_pose_by_id(self.data.next_cluster_waypoints[drone_id][self.support_step]), self.hor_offset,self.layer_gap, self.data.get_order_by_id(self.data.next_cluster_waypoints[drone_id][self.support_step]), back2origin)
                
                duration = calc_duration(current_pose, self.target_pose[drone_id])
                self.data.send_go_to(drone_id, self.target_pose[drone_id], duration_sec=duration)
            
            if self.support_step == self.mutual_support_index and isinstance(self.data.next_cluster_waypoints[self.last_drone][-1], list):
                self.control.publish_debug(f"ClusterNavSup(): Traversing {self.data.next_cluster_waypoints[drone_id][self.support_step]}")

            self.command_sent = True

            return 'Sent_Wp'
        else:
            # Verifica se todos os drones já atingiram o waypoint atual.
            finished_current_step = {}
            for drone_id in self.drone_ids:
                current_pose = self.data.get_pose(drone_id)

                if is_pose_reached(current_pose, self.target_pose[drone_id]):
                    finished_current_step[drone_id] = True
                else:
                    finished_current_step[drone_id] = False
                        
            if all(finished_current_step.get(d, False) for d in self.drone_ids):
                # Verifica se o waypoint atual é o da base (quando o valor é 0).
                if (self.support_step == len(self.cluster_waypoints[self.last_drone]) - 1 and 
                    self.cluster_waypoints[self.last_drone][self.support_step] == 0):
                    self.support_step = 0
                    self.first_execution = True
                    return 'Reached_Base_and_Needs_Battery'
                else:
                    if self.support_step == self.mutual_support_index:
                        self.control.publish_debug(f"ClusterNavSup(): Finalizado Traversing {self.data.next_cluster_waypoints[drone_id][self.support_step]}.")
                    else:
                        self.control.publish_debug(f"ClusterNavSup(): Finalizado Waypoint Id {self.data.next_cluster_waypoints[drone_id][self.support_step]}.")

                    self.support_step += 1
                    self.command_sent = False
                    return 'Reached_Intermediary_Step'
            else:
                return 'Navigating_To_Wp'

class ClusterNavExp(State):
    def __init__(self):
        super().__init__(outcomes=["Navigating", "Reached_End_Of_Cluster", 'Low_Battery'])
        self.data = DataWrapper()
        self.control = ControlWrapper()
        self.drone_ids = self.data.get_drone_ids()
        self.first_execution = True
        self.battery_threshold = 80

        # Inicializa os dicionários para cada drone (SM)
        self.target_pose = {drone_id: None for drone_id in self.drone_ids}
        self.support_sub_step = {drone_id: 0 for drone_id in self.drone_ids}
        self.command_sent = {drone_id: False for drone_id in self.drone_ids}

        self.counter_print = 0

    def needs_recharge(self, drone_id, cluster_iteration, cluster_indexes, trajectory_ids):
        segment_till_cluster, ___ = get_trajectory_segment(cluster_iteration, trajectory_ids, cluster_indexes)
        segment_till_base = truncate_after_zero(segment_till_cluster)
        
        trajectory_poses = self.data.get_poses_by_ids_list(flatten(segment_till_base))
        battery_consumption_to_base = calculate_battery_consumption(trajectory_poses)

        current_battery = self.data.get_battery(drone_id).percentage
        real_battery = current_battery - battery_consumption_to_base

        if real_battery < self.battery_threshold:
            return True
        else:
            return False

    def execute(self, data):
        if self.first_execution:
            # Se a intenção for manter a lista completa, não sobrescreva os waypoints.
            # Se necessário, defina um índice de finalização para cada drone.
            last_drone = self.drone_ids[-1]
            unique_ids = extract_unique_ids(self.data.get_trajectory_ids(last_drone))
            self.first_execution = False

            cluster_iteration = self.data.get_clusterIteration()
            cluster_indexes = get_clusters_indexes_on_traj_ids(self.data.get_trajectory_all())
            next_cluster = cluster_indexes[cluster_iteration]

            self.control.publish_debug(f"ClusterNavExp(): Começando exploração.")
            
            for drone_id in self.drone_ids:
                if isinstance(self.data.next_cluster_waypoints[drone_id][-1], list):
                    for i, value in enumerate(self.data.next_cluster_waypoints[drone_id][-1]):
                        if value in unique_ids:
                            self.data.next_cluster_waypoints[drone_id][-1].pop(i)

        cluster_iteration = self.data.get_clusterIteration()
        cluster_indexes = get_clusters_indexes_on_traj_ids(self.data.get_trajectory_all())

        # Verificação da Bateria
        needs_charge = False
        for drone_id in self.drone_ids:
            trajectory_ids = self.data.get_trajectory_ids(drone_id)
            if self.needs_recharge(drone_id, cluster_iteration, cluster_indexes, trajectory_ids) == True:
                self.control.publish_debug(f"ClusterNavExp(): "+str(cluster_iteration)+ " aaaa "+str(cluster_indexes)+ " aaaa "+str(trajectory_ids))
                needs_charge = True

        if needs_charge:
            self.data.update_expIteration_before_charge(self.support_sub_step)
            self.first_execution = True
            self.target_pose = {drone_id: None for drone_id in self.drone_ids}
            self.command_sent = {drone_id: False for drone_id in self.drone_ids}
            self.control.publish_debug(f"ClusterNavExp(): Retornando da Exploração por falta de bateria. ExpIteration: {self.support_sub_step}")
            self.data.need_charging = True
            return 'Low_Battery'
        
        drones_reached = 0

        for drone_id in self.drone_ids:
            current_pose = self.data.get_pose(drone_id)
            
            if not self.command_sent[drone_id]:
                # Usa o índice específico para cada drone
                if not isinstance(self.data.next_cluster_waypoints[drone_id][-1], list) or self.data.next_cluster_waypoints[drone_id][-1] == []: # Se não for lista, ignora
                    drones_reached += 1 # Já está no destino
                else:
                    target_pose_id = self.data.next_cluster_waypoints[drone_id][-1][self.support_sub_step[drone_id]] # Id

                    self.target_pose[drone_id] = self.data.get_pose_by_id(target_pose_id)

                    duration = calc_duration(current_pose, self.target_pose[drone_id])
                    self.data.send_go_to(drone_id, self.target_pose[drone_id], duration_sec=duration)
                    self.command_sent[drone_id] = True
            else:
                if is_pose_reached(current_pose, self.target_pose[drone_id]):
                    if self.support_sub_step[drone_id] == len(self.data.next_cluster_waypoints[drone_id][-1]) - 1:
                        drones_reached += 1
                    else:
                        self.support_sub_step[drone_id] += 1
                        self.command_sent[drone_id] = False

        if drones_reached == len(self.drone_ids):

            self.first_execution = True
            self.target_pose = {drone_id: None for drone_id in self.drone_ids}
            self.support_sub_step = {drone_id: 0 for drone_id in self.drone_ids}
            self.command_sent = {drone_id: False for drone_id in self.drone_ids}

            self.control.publish_debug(f"ClusterNavExp(): Finalizado.")
            self.data.increase_clusterIteration()
            
            return "Reached_End_Of_Cluster"
        else:
            return "Navigating"

class Charging(State):
    def __init__(self):
        super().__init__(outcomes=["Below_Threshold", "Above_Threshold"])

        self.data = DataWrapper()
        self.control = ControlWrapper()
        self.bat_threshold = 89.9
        self.drone_ids = self.data.get_drone_ids()
        self.last_drone = self.drone_ids[-1]
        
        # A cada 1min 20s, perde-se 10% da bateria. (A cada segundo, perde-se 1/8% de bateria)
        self.decrease_battery_value_per_second = 1/8
        
        self.battery_usage_forecast = {}

        self.first_execution = True

    def execute(self, data):
        
        if self.first_execution:
            for drone_id in self.drone_ids:
                self.data.send_land(drone_id,0.0, 3.0)
                current_iteration = self.data.get_clusterIteration()
                cluster_indexes = get_clusters_indexes_on_traj_ids(self.data.get_trajectory_all())
                iteration_on_traj_id = cluster_indexes[current_iteration]

                traj_until_finish = self.data.get_trajectory_ids(self.last_drone)[iteration_on_traj_id:]
                trajectory_poses = self.data.get_poses_by_ids_list(flatten(traj_until_finish))
                battery_consumption_until_finish = calculate_battery_consumption(trajectory_poses)

                self.bat_threshold = battery_consumption_until_finish + 10.0

                if self.bat_threshold > 89.9:
                    self.bat_threshold = 89.9
                
            
            self.control.publish_debug(f"Charging(): Threshold {self.bat_threshold:.1f}. Consumo até o final: {battery_consumption_until_finish:.1f}")        
            self.first_execution = False

        drones_charged = 0

        for drone_id in self.drone_ids:
            battery = self.data.get_battery(drone_id).percentage

            if battery >= self.bat_threshold:
                drones_charged += 1
        
        if drones_charged == len(self.drone_ids):
            self.first_execution = True
            self.control.publish_debug(f"Charging(): Finalizado.")
            self.data.need_charging = False
            return 'Above_Threshold'
        else:
            return 'Below_Threshold'