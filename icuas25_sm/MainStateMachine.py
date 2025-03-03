#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading

from smach import StateMachine, State
from Data_Wrapper import DataWrapper
from States import *

# Função que roda a máquina de estados em uma thread separada
def state_machine_thread(sm_node):

    sm = StateMachine(outcomes=['End'])
    sm.node = sm_node

    with sm:
        StateMachine.add('Waiting_For_Trajectories', AwaitingTrajectory(), 
                           transitions={'Not_Received': 'Waiting_For_Trajectories',
                                        'Received_All': 'Takeoff'})
        StateMachine.add('Takeoff', Takeoff(),
                           transitions={'Not_Reached': 'Takeoff',
                                        'Reached': 'End'})
        StateMachine.add('WaypointNav', WaypointNav(),
                           transitions={'Not_Reached': 'Takeoff',
                                        'Reached': 'End'})
        
    outcome = sm.execute()
    sm_node.get_logger().info(f"Máquina de estados finalizada com outcome: {outcome}")
    sm_node.destroy_node()


def main():
    rclpy.init()

    # Instancia o DataWrapper (singleton) e o nó da máquina de estados
    data_wrapper = DataWrapper()
    sm_node = Node('main_sm')

    # Executor multi-thread para rodar ambos os nós em paralelo
    executor = MultiThreadedExecutor()
    executor.add_node(data_wrapper)
    executor.add_node(sm_node)

    # Inicia a thread da máquina de estados, passando o nó dela
    sm_thread = threading.Thread(target=state_machine_thread, args=(sm_node,))
    sm_thread.start()

    try:
        executor.spin()  # roda os nós adicionados (processa callbacks, etc.)
    except KeyboardInterrupt:
        data_wrapper.get_logger().info("Encerrando execução...")
    finally:
        data_wrapper.destroy_node()
        sm_node.destroy_node()
        rclpy.shutdown()
        sm_thread.join()

if __name__ == '__main__':
    main()
