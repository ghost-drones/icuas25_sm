#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from yasmin import StateMachine
from States_AwaitingTrajectory import *

node = Node('ssm_awaiting_trajectory')
awaitingTrajectory = StateMachine(initial_state='Waiting_For_Trajectories', outcomes=['Done'])

with awaitingTrajectory:

    awaitingTrajectory.add_state('Waiting_For_Trajectories', AwaitingTrajectory(node),
                 transitions={'Not_Received': 'Waiting_For_Trajectories',
                              'Received_All': 'Done'})
    
if __name__ == '__main__':
    awaitingTrajectory.execute()