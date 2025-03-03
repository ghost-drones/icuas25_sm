from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
import math
import numpy as np

def is_pose_reached(current_pose: PoseStamped, target_pose: PoseStamped, tolerance=1.0) -> bool:
    dx = current_pose.pose.position.x - target_pose.pose.position.x
    dy = current_pose.pose.position.y - target_pose.pose.position.y
    dz = current_pose.pose.position.z - target_pose.pose.position.z
    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
    return distance < tolerance

def get_yaw_from_pose(pose: PoseStamped) -> float:
    quat = [pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w]
    _, _, yaw = euler_from_quaternion(quat)
    return yaw

def calc_duration(pos_1: PoseStamped, pos_2: PoseStamped, avr_vel=1.2) -> float:
    p1 = np.array([pos_1.pose.position.x, pos_1.pose.position.y, pos_1.pose.position.z])
    p2 = np.array([pos_2.pose.position.x, pos_2.pose.position.y, pos_2.pose.position.z])
    return np.linalg.norm(p2 - p1) / avr_vel

def get_flat_index(trajectories_id, step, substep):
    steps = trajectories_id
    flat_index = 0
    for i in range(step):
        elem = steps[i]
        if isinstance(elem, list):
            flat_index += len(elem)
        else:
            flat_index += 1
    current_elem = steps[step]
    if isinstance(current_elem, list):
        flat_index += substep
    else:
        flat_index += 0
    return flat_index