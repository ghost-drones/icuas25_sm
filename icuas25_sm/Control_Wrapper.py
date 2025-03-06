#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import String, Bool  # Importa a mensagem padrão para string
from Functions_Wrapper import *
import os

# Serviços para controle
from icuas25_msgs.srv import PathService
from crazyflie_interfaces.msg import TrajectoryPolynomialPiece

# Serviços para controle
from crazyflie_interfaces.srv import GoTo, Land, Takeoff, UploadTrajectory, StartTrajectory
import threading
from typing import Union

class ControlWrapper(Node):
    _instance = None  # Singleton

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(ControlWrapper, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        # Evita reinicialização (singleton)
        if hasattr(self, '_initialized') and self._initialized:
            return
        super().__init__('data_wrapper_sm')
        self._initialized = True

        num_robots = int(os.getenv("NUM_ROBOTS", "5"))
        self.drone_ids = list(range(1, num_robots + 1))
        # Lock para sincronização
        self.data_lock = threading.Lock()

        self.clusterIteration = 0
        self.upload_trajectory_clients = {}
        self.start_trajectory_clients = {}

        self.client_path_planner = self.create_client(PathService, '/ghost/path_planner')
        #while not self.client_path_planner.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('Service /ghost/path_planner not available, waiting again...')

        for drone_id in self.drone_ids:
            upload_trajectory_service = f'/cf_{drone_id}/upload_trajectory'
            start_trajectory_service = f'/cf_{drone_id}/start_trajectory'
            self.upload_trajectory_clients[drone_id] = self.create_client(UploadTrajectory, upload_trajectory_service)
            self.start_trajectory_clients[drone_id] = self.create_client(StartTrajectory, start_trajectory_service)

        self.debug_publisher = self.create_publisher(String, '/ghost/debug', 10)
        self.end_publisher = self.create_publisher(Bool, '/mission_done', 10)

    def publish_end(self):
        msg = Bool()

        msg.data = True
        self.end_publisher.publish(msg)

    def publish_debug(self, message: str, ignore=False) -> None:
        msg = String()

        if not ignore:
            green = "\033[1;32m"
            yellow = "\033[1;33m"
            reset = "\033[0m"
            idx = message.find("()")
            if idx != -1:
                part1 = message[:idx]
                part2 = message[idx:idx+2]  # A substring "()" (2 caracteres)
                part3 = message[idx+2:]
                colored_message = f"{green}{part1}{reset}{yellow}{part2}{reset}{part3}"
            else:
                colored_message = message

            msg.data = colored_message
        else:
            msg.data = str(message)

        self.debug_publisher.publish(msg)

    def request_traverse_origin_sync(self, drone_id, origin: Point, destination: Point, support: Point):
        # Cria a requisição preenchendo os pontos
        request = PathService.Request()
        request.origin = Point(x=origin.x, y=origin.y, z=origin.z)
        request.destination = Point(x=destination.x, y=destination.y, z=destination.z)
        request.support = Point(x=support.x, y=support.y, z=support.z)

        # Faz a chamada assíncrona
        future = self.client_path_planner.call_async(request)
        
        # Bloqueia até que o future seja completado
        rclpy.spin_until_future_complete(self, future)

        try:
            # Obtém a resposta do serviço
            response = future.result()
            return response
        except Exception as e:
            self.get_logger().error(f"Ghost/path_planner service call failed: {e}")
            return None

    def send_upload_trajectory(self, drone_id: int, id: int, start_pos: Union[Pose,PoseStamped,Point], end_pos: Union[Pose,PoseStamped,Point], duration: float):
        client = self.upload_trajectory_clients[drone_id]

        if isinstance(start_pos,Point):
            yaw = 0.0
        else:
            yaw = get_yaw_from_pose(end_pos)

        if isinstance(start_pos,Pose):
            array_start = np.array([start_pos.position.x,start_pos.position.y,start_pos.position.z])
        elif isinstance(start_pos,PoseStamped):
            array_start = np.array([start_pos.pose.position.x,start_pos.pose.position.y,start_pos.pose.position.z])
        else:
            array_start = np.array([start_pos.x,start_pos.y,start_pos.z])

        if isinstance(end_pos,Pose):
            array_end = np.array([end_pos.position.x,end_pos.position.y,end_pos.position.z])
        elif isinstance(end_pos,PoseStamped):
            array_end = np.array([end_pos.pose.position.x,end_pos.pose.position.y,end_pos.pose.position.z])
        else:
            array_end = np.array([end_pos.x,end_pos.y,end_pos.z])

        traj = create_linear_trajectory(
            start_pos=array_start,
            end_pos=array_end,
            yaw=yaw,
            duration=duration
        )
        
        request = UploadTrajectory.Request()
        request.trajectory_id = id # id
        request.piece_offset = 0         # Offset inicial (usualmente zero)
        
        pieces = []
        for poly in traj.polynomials:
            piece = TrajectoryPolynomialPiece()
            # Converte os coeficientes para listas de floats
            piece.poly_x = [poly.px.p[0],poly.px.p[1],0.0,0.0,0.0,0.0,0.0,0.0]
            piece.poly_y = [poly.py.p[0],poly.py.p[1],0.0,0.0,0.0,0.0,0.0,0.0]
            piece.poly_z = [poly.pz.p[0],poly.pz.p[1],0.0,0.0,0.0,0.0,0.0,0.0]
            piece.poly_yaw = [poly.pyaw.p[0],poly.pyaw.p[1],0.0,0.0,0.0,0.0,0.0,0.0]
            # Preenche a duração da peça
            piece.duration.sec = int(traj.duration)
            piece.duration.nanosec = int((traj.duration - int(traj.duration)) * 1e9)
            pieces.append(piece)
        
        request.pieces = pieces
        #client.call_async(request)
        future = client.call_async(request)
        
        # Bloqueia até que o future seja completado
        rclpy.spin_until_future_complete(self, future)

        self.send_start_trajectory(drone_id)

    def send_start_trajectory(self, drone_id, group_mask=0, relative=False):
        client = self.start_trajectory_clients[drone_id]
        request = StartTrajectory.Request()
        request.group_mask = group_mask
        # Aqui usamos um ID fixo (por exemplo, 1). Se você gerencia múltiplas trajetórias, passe o id desejado.
        request.trajectory_id = 1
        request.timescale = 1.0      # Pode ajustar o timescale se necessário
        request.reversed = False     # Trajetória não invertida
        request.relative = relative  # Define se a trajetória é relativa ao estado atual
        #client.call_async(request)

        future = client.call_async(request)
        
        # Bloqueia até que o future seja completado
        rclpy.spin_until_future_complete(self, future)