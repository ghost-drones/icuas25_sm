#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import String  # Importa a mensagem padrão para string

# Serviços para controle
from icuas25_msgs.srv import PathService

import threading

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

        # Lock para sincronização
        self.data_lock = threading.Lock()

        self.clusterIteration = 0

        self.client_path_planner = self.create_client(PathService, '/ghost/path_planner')
        #while not self.client_path_planner.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('Service /ghost/path_planner not available, waiting again...')
        
        self.debug_publisher = self.create_publisher(String, '/ghost/debug', 10)

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
