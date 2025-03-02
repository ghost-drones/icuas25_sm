#!/usr/bin/env python3
from yasmin import State
from Data_Wrapper import DroneData
from Control_Wrapper import Control_Wrapper

class AwaitingTrajectory(State, DroneData):
    def __init__(self, node):
        # Inicializa o estado com os outcomes desejados
        super().__init__(name="AwaitingTrajectory", outcomes=["Not_Received", "Received_All"])
        DroneData.__init__(self)
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
        
        if received_value == len(self.drone_ids):
            return 'Received_All'
        else:
            return 'Not_Received'