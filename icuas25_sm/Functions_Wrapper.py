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

def calc_duration(pos_1, pos_2,avr_vel=1.2) -> float:
    
    if isinstance(pos_1,PoseStamped):
        position_01 = pos_1.pose.position
    elif isinstance(pos_1,Pose):
        position_01 = pos_1.position
    else:
        return 0.0
    if isinstance(pos_2,PoseStamped):
        position_02 = pos_2.pose.position
    elif isinstance(pos_2,Pose):
        position_02 = pos_2.position
    else:
        return 0.0
    position_01_np_array = np.array([position_01.x, position_01.y, position_01.z], dtype='float32')
    position_02_np_array = np.array([position_02.x, position_02.y, position_02.z], dtype='float32')

    duration = np.linalg.norm(position_02_np_array - position_01_np_array) / avr_vel

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
    for values in data.values():
        for item in values:
            if not isinstance(item, list):
                if item not in unique_ids:
                    unique_ids.append(item)
    return unique_ids
 
def get_current_trajectory_id(trajectory_ids:list,current_iteration:int) -> int:
    current = 0
    for count, element in enumerate(trajectory_ids):
        if isinstance(element,list):
            if current == current_iteration:
                break
            else:
                current+=1
    return count