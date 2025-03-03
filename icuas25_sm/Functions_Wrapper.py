from geometry_msgs.msg import PoseStamped,Pose
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

def calc_duration(pos_1, pos_2,avr_vel) -> float:
        if isinstance(pos_1,PoseStamped) and isinstance(pos_2,PoseStamped):
            position_01 = pos_1.pose.position
            position_02 = pos_2.pose.position
        elif isinstance(pos_1,Pose) and isinstance(pos_2,Pose):
            position_01 = pos_1.position
            position_02 = pos_2.position
        else:
            return 0.0
        pos_1 = np.array([position_01.x, position_01.y, position_01.z], dtype='float32')
        pos_2 = np.array([position_02.x, position_02.y, position_02.z], dtype='float32')

        duration = np.linalg.norm(pos_2 - pos_1) / avr_vel

        return float(duration)

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

def get_clusters_list(lista):
    return [int(item[1:]) for item in lista if item.startswith("E")]

def get_trajectory_segment(cluster_index, trajectory_ids, cluster_list):
    
    if cluster_index < 0 or cluster_index >= len(cluster_list):
        raise ValueError("Índice do cluster inválido.")
    
    start = 0 if cluster_index == 0 else cluster_list[cluster_index - 1] + 1
    end = cluster_list[cluster_index] + 1
    forward_segment = trajectory_ids[start:end]
    
    if cluster_index == 0:
        return forward_segment, forward_segment
    
    mirror_source = forward_segment[:-1]
    mirror_reversed = list(reversed(mirror_source))
    
    mirror = []
    for val in mirror_reversed:
        mirror.append(val)
        if val == 0:
            break

    expanded_segment = forward_segment + mirror
    return forward_segment, expanded_segment

def calculate_battery_consumption(poses, avr_vel=1.2) -> float:
    """
    Calcula o percentual de bateria consumido por um trajeto definido por uma lista de pontos PoseStamped.
    
    A lógica é a seguinte:
      - Para cada segmento entre dois pontos consecutivos, utiliza a função calc_duration para determinar
        o tempo de deslocamento.
      - A cada 80 segundos (1min20s) há um consumo de 10% da bateria.
      
    Assim, o consumo percentual total é dado por:
    
        consumo (%) = (tempo_total / 80) * 10
        
    Args:
        poses (list): Lista de pontos do trajeto (PoseStamped).
        avr_vel (float): Velocidade média utilizada no cálculo (default=1.2).
    
    Returns:
        float: Porcentagem da bateria consumida pelo trajeto.
    """
    total_duration = 0.0
    # Calcula o tempo de trajeto entre pontos consecutivos
    for i in range(len(poses) - 1):
        total_duration += calc_duration(poses[i], poses[i+1], avr_vel)
    
    # Converte o tempo total para percentual consumido.
    # Cada 80 segundos correspondem a 10% de bateria.
    battery_consumed = (total_duration / 80) * 10
    return battery_consumed

def truncate_after_zero(lst: list) -> list:
    """
    Retorna uma nova lista contendo todos os elementos de 'lst' até a primeira ocorrência de 0 (inclusive).
    
    Exemplo:
        Entrada: [1, 2, 3, 4, 0, 2, 2, 1]
        Saída: [1, 2, 3, 4, 0]
    """
    truncated = []
    for num in lst:
        truncated.append(num)
        if num == 0:
            break
    return truncated