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

    # Data_Wrapper node
    sm_wrapper_node = Node(
        package='icuas25_sm',
        executable='Data_Wrapper.py',
        name='SM_wrapper',
        output='screen',
        parameters=[state_params]
    )

    # Control_Wrapper node
    sm_control_wrapper_node = Node(
        package='icuas25_sm',
        executable='Control_Wrapper.py',
        name='SM_control',
        output='screen',
        parameters=[state_params]
    )

    # MainStateMachine node
    sm_state_machine_node = Node(
        package='icuas25_sm',
        executable='MainStateMachine.py',
        name='SM_state_machine',
        output='screen',
        parameters=[state_params]
    )

    return LaunchDescription([
        sm_wrapper_node,
        sm_state_machine_node,
        sm_control_wrapper_node,
    ])

if __name__ == '__main__':
    generate_launch_description()