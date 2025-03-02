#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from yasmin import StateMachine
from icuas25_sm.States_AwaitingTrajectory import AwaitingTrajectory

class MainStateMachineNode(Node):
    def __init__(self):
        super().__init__('main_statemachine')
        self.get_logger().info("Iniciando a Máquina de Estados Principal...")

        self.sm = StateMachine(initial_state='AwaitingTrajectory', outcomes=['End'])

        self.sm.add_state('Waiting_For_Trajectories', AwaitingTrajectory(self),
                    transitions={'Not_Received': 'Waiting_For_Trajectories',
                                'Received_All': 'Done'})

    def run_state_machine(self):
        result = self.sm.execute()
        self.get_logger().info(f"Máquina de estados finalizada com resultado: {result}")

def main(args=None):
    rclpy.init(args=args)
    node = MainStateMachineNode()
    try:
        node.run_state_machine()
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando a máquina de estados.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
