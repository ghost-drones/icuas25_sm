#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading

from smach import StateMachine, State
from Data_Wrapper import DataWrapper
from Control_Wrapper import ControlWrapper
from States import *

# Função que executa a máquina de estados em uma thread separada
def state_machine_thread():
    sm_node = Node('main_sm')

    sm = StateMachine(outcomes=['End'])
    sm.node = sm_node
    """
        StateMachine.add('Waiting_For_Trajectories', AwaitingTrajectory(),
                    transitions={'Not_Received': 'Waiting_For_Trajectories',
                                'Received_All': 'Takeoff'})
    
        StateMachine.add('Takeoff', Takeoff(self),
                    transitions={'Not_Reached': 'Takeoff',
                                'Reached': 'WaypointNav'})

        StateMachine.add('WaypointNav', WaypointNav(self),
                    transitions={'Executing': 'WaypointNav',
                                 'Low_Battery': 'Return2Home',
                                 'Finished': 'Return2Home'})


        self.sm.add_state('Return2Home', Return2Home(self),
                    transitions={'Executing': 'Return2Home',
                                 'Reached_Base': 'HomeActions'})

        self.sm.add_state('HomeActions', HomeActions(self),
                    transitions={'Recharging': 'HomeActions',
                                 'Charged': 'WaypointNav',
                                 'Finished_Traj': 'End'})
    """

    with sm:
        StateMachine.add('Waiting_For_Trajectories', AwaitingTrajectory(), 
                           transitions={'Not_Received': 'Waiting_For_Trajectories',
                                        'Received_All': 'Takeoff'})
        
        StateMachine.add('Takeoff', Takeoff(),
                    transitions={'Not_Reached': 'Takeoff',
                                'Reached': 'End'})
        
    outcome = sm.execute()
    sm_node.get_logger().info(f"Máquina de estados finalizada com outcome: {outcome}")
    sm_node.destroy_node()

def main():
    rclpy.init()

    # Instancia o DataWrapper (singleton)
    data_wrapper = DataWrapper()
    control_wrapper = ControlWrapper()

    executor = MultiThreadedExecutor()
    executor.add_node(data_wrapper)
    executor.add_node(control_wrapper)

    sm_thread = threading.Thread(target=state_machine_thread)
    sm_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        data_wrapper.get_logger().info("Encerrando execução...")
        control_wrapper.get_logger().info("Encerrando execução...")
    finally:
        data_wrapper.destroy_node()
        control_wrapper.destroy_node()
        rclpy.shutdown()
        sm_thread.join()

if __name__ == '__main__':
    main()
