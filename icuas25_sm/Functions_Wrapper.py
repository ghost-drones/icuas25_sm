from geometry_msgs.msg import PoseStamped,Pose,Point
from tf_transformations import euler_from_quaternion
import math
import numpy as np
import time
from copy import deepcopy

def is_pose_reached(current_pose, target_pose, tolerance=1.0) -> bool:

    if isinstance(current_pose,PoseStamped):
        current_pose = current_pose.pose.position
    elif isinstance(current_pose,Pose):
        current_pose = current_pose.position
    else: # Point
        current_pose = current_pose
    
    if isinstance(target_pose,PoseStamped):
        target_pose = target_pose.pose.position
    elif isinstance(target_pose,Pose):
        target_pose = target_pose.position
    else: # Point
        target_pose = target_pose
    
    dx = current_pose.x - target_pose.x
    dy = current_pose.y - target_pose.y
    dz = current_pose.z - target_pose.z
    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
    return distance < tolerance

def get_yaw_from_pose(pose) -> float:
    if isinstance(pose,PoseStamped):
        pose = pose.pose.orientation
    elif isinstance(pose,Pose):
        pose = pose.orientation
    else:
        return 0.0
    
    quat = [pose.x,
            pose.y,
            pose.z,
            pose.w]
    _, _, yaw = euler_from_quaternion(quat)
    return yaw

def calc_duration(pos_1, pos_2, avr_vel=1.2, yaw_increment=1.0) -> float:
    if isinstance(pos_1, PoseStamped):
        position_01 = pos_1.pose.position
        orientation_01 = pos_1.pose.orientation
    elif isinstance(pos_1, Pose):
        position_01 = pos_1.position
        orientation_01 = pos_1.orientation
    else:  # Point
        position_01 = pos_1
        orientation_01 = None

    if isinstance(pos_2, PoseStamped):
        position_02 = pos_2.pose.position
        orientation_02 = pos_2.pose.orientation
    elif isinstance(pos_2, Pose):
        position_02 = pos_2.position
        orientation_02 = pos_2.orientation
    else:  # Point
        position_02 = pos_2
        orientation_02 = None
    
    position_01_np_array = np.array([position_01.x, position_01.y, position_01.z], dtype='float32')
    position_02_np_array = np.array([position_02.x, position_02.y, position_02.z], dtype='float32')
    
    # Calcular diferença de yaw, se as orientações forem fornecidas
    yaw_diff = 1.0
    if orientation_01 and orientation_02:
        yaw_1 = euler_from_quaternion([orientation_01.x, orientation_01.y, orientation_01.z, orientation_01.w])[2]
        yaw_2 = euler_from_quaternion([orientation_02.x, orientation_02.y, orientation_02.z, orientation_02.w])[2]
        yaw_diff = np.arctan2(np.sin(yaw_2 - yaw_1), np.cos(yaw_2 - yaw_1))
        yaw_diff = 1 + yaw_increment * abs(yaw_diff) / np.pi  # Mapeia yaw_diff para o intervalo [1, yaw_inc]

        if yaw_diff < 1.3:
            yaw_diff = 1.0
    
    duration = yaw_diff*np.linalg.norm(position_02_np_array - position_01_np_array) / avr_vel

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

def get_clusters_indexes_on_traj_ids(lista):
    clusters = []

    for i, value in enumerate(lista):
        value = value.strip()
        if value.startswith("E"):
            clusters.append(i)

    return clusters

def flatten(a):  
    res = []  
    for x in a:  
        if isinstance(x, list):  
            res.extend(flatten(x))  # Recursively flatten nested lists  
        else:  
            res.append(x)  # Append individual elements  
    return res  

def get_trajectory_segment(cluster_iteration, trajectory_ids, cluster_indexes_on_traj_ids):

    if cluster_iteration == 0:
        for i, value in enumerate(trajectory_ids):
            if value == 0:
                zero_index = i
                break
    
        return trajectory_ids[0:1], trajectory_ids[0:zero_index+1]

    curr_trajectory_id = cluster_indexes_on_traj_ids[cluster_iteration-1]
    forward_trajectory_id = cluster_indexes_on_traj_ids[cluster_iteration]

    forward_segment = trajectory_ids[curr_trajectory_id+1:forward_trajectory_id+1]

    for i, value in enumerate(forward_segment):
        if value == 0:
            zero_index = i
    
    back_to_zero = forward_segment[zero_index:]
    back_to_zero.pop(-1)

    expanded_segment = forward_segment + back_to_zero[::-1]

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

def sublist_from_first_zero(lst):
    """
    Dado uma lista de ints, retorna a sublista iniciando no primeiro 0 (inclusive).
    Se não houver 0, retorna lista vazia.
    """
    for idx, val in enumerate(lst):
        if val == 0:
            return lst[idx:]
    return []

def get_support_segments(data, cluster_index):

    # Identifica os índices onde os elementos são clusters (listas)
    cluster_positions = [i for i, el in enumerate(data) if isinstance(el, list)]
    
    if cluster_index < 0 or cluster_index >= len(cluster_positions):
        raise ValueError("Índice do cluster inválido.")
    
    # Extrai os segmentos de suporte entre os clusters
    support_segments = []
    for i in range(1, len(cluster_positions)):
        start = cluster_positions[i - 1] + 1
        end = cluster_positions[i]
        # Suporte entre clusters pode conter vários ints
        support_segments.append(data[start:end])
    
    # Para o primeiro cluster, não há suporte "anterior"
    if cluster_index == 0:
        support_current_cluster = [0]
    else:
        # O suporte anterior ao cluster i é o suporte entre cluster (i-1) e cluster i
        seg = support_segments[cluster_index - 1]
        support_current_cluster = sublist_from_first_zero(seg)
    
    # Se houver um cluster seguinte, o suporte após o cluster i é o suporte entre cluster i e cluster i+1
    if cluster_index < len(cluster_positions) - 1:
        seg = support_segments[cluster_index]
        support_next_cluster = sublist_from_first_zero(seg)
    else:
        support_next_cluster = []
    
    return support_current_cluster, support_next_cluster

def extract_unique_ids(data):
    unique_ids = []
    for element in data:
        if isinstance(element, list):
            pass
        else:
            if element not in unique_ids:
                unique_ids.append(element)
    return unique_ids

def add_offset_2_pose(num_drones, drone_id, target_pose, hor_offset, layer_gap, wp_order):
    # Calcula o ângulo e os offsets
    angle = 2 * math.pi * (drone_id - 1) / (num_drones - (wp_order+1))
    x_offset = hor_offset * math.cos(angle)
    y_offset = hor_offset * math.sin(angle)
    z_offset = drone_id * layer_gap

    # Cria uma cópia independente do objeto de posição
    if isinstance(target_pose, PoseStamped):
        new_target = deepcopy(target_pose.pose.position)
    elif isinstance(target_pose, Pose):
        new_target = deepcopy(target_pose.position)
    elif isinstance(target_pose, Point):
        new_target = deepcopy(target_pose)
    else:
        raise TypeError("Tipo de target_pose não suportado: {}".format(type(target_pose)))

    # Aplica os offsets na cópia
    
    # Se o drone que vai pro pt de suporte é aquele que será o suporte, offset = 0
    if not drone_id == wp_order+1:
        new_target.x += x_offset
        new_target.y += y_offset
        new_target.z += z_offset

    return new_target