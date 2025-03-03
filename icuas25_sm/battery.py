from .Data_Wrapper import Data_Wrapper
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import BatteryState
import numpy as np
from std_msgs.msg import String
import ast

class batteries():
    def __init__(self):
        self.data = Data_Wrapper()
        self.batteries = self.data.get_full_battery()
        self.drone_ids = self.data.get_drone_ids()
        self.trajectory = self.data.get_full_trajectory()
        self.trajectory_ids = self.data.get_full_trajectory_ids()
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
      
    def it_achieve(self,cluster_origin:int) -> float:
        
        for drone_id in self.drone_ids:
            batteries = self.batteries[drone_id]
            if isinstance(batteries,BatteryState):
                percentage = batteries.percentage
                
            trajectory_id = self.trajectory_ids[drone_id]
            if isinstance(trajectory_id,String):
                trajectory_id_list = ast.literal_eval(trajectory_id.data)
            
                
                for i, item in enumerate(trajectory_id_list[cluster_origin:], start=cluster_origin):
                    if isinstance(item, list):
                        cluster_destination = i
                        break
                    
            cluster_origin_id = trajectory_id[cluster_origin]
            cluster_destination_id = trajectory_id[cluster_destination]
            if isinstance(cluster_origin_id, list) and isinstance(cluster_destination_id, list):
                origin = cluster_origin_id[-1]
                duration_list = []
                path = list(origin) + cluster_destination_id
                for pos1, pos2 in zip(path, path[1:]):
                    if isinstance(pos1,int) and isinstance(pos2,int):
                        duration = self.calc_duration(FUNC(pos1),FUNC(pos2))
                        duration_list.append(duration)
                
                battery_usage_forecast = 0.0
                for duration in duration_list:
                    battery_usage_forecast += duration*self.avg_vel*self.decrease_battery_value_per_second
                    if (percentage - battery_usage_forecast) < self.target:
                        return False
        return True