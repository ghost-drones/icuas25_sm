#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # SM params
    state_params = os.path.join(
        get_package_share_directory('icuas25_sm'),
        'params',
        'SM_params.yaml'
    )

    # MainStateMachine node
    sm_state_machine_node = Node(
        package='icuas25_sm',
        executable='MainStateMachine.py',
        name='SM_state_machine',
        output='screen',
        parameters=[state_params],
        arguments=['--ros-args', '--log-level', 'error']
    )

    return LaunchDescription([
        sm_state_machine_node
    ])

if __name__ == '__main__':
    generate_launch_description()