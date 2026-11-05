#!/usr/bin/python3
"""Nodo que ejecuta las tres trayectorias de la práctica: Joint, Lineal y Circular."""

import copy
from collections.abc import Callable
from math import cos, pi, sin

import rclpy
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from ur3e_kinematics.control_robot import ControlRobot

TOPIC_TRAYECTORIA_ACTUAL = "/trayectoria_actual"
TAMANO_COLA = 10
SIN_TRAYECTORIA = ""  # Se publica entre trayectorias para indicar que no hay que grabar

# Posturas articulares (rad) en orden base -> muñeca
POSTURA_INICIAL = [0.0, -pi / 2, -pi / 2, -pi / 2, pi / 2, 0.0]
POSTURA_JOINT = [pi / 4, -2 * pi / 3, -pi / 3, -pi / 2, pi / 2, 0.0]

# Trayectoria lineal: cuadrado vertical (plano YZ) recorrido por el efector final
LADO_CUADRADO_M = 0.1
DESPLAZAMIENTOS_CUADRADO = [  # (y, z) de cada lado: baja, avanza, sube y vuelve
    (0.0, -LADO_CUADRADO_M),
    (LADO_CUADRADO_M, 0.0),
    (0.0, LADO_CUADRADO_M),
    (-LADO_CUADRADO_M, 0.0),
]

# Trayectoria circular: circunferencia horizontal (plano XY)
RADIO_CIRCULO_M = 0.05
PUNTOS_CIRCULO = 36  # Tramos rectos con los que se aproxima la circunferencia


class EjecutorTrayectorias(ControlRobot):
    """Ejecuta las trayectorias de la práctica e informa de cuál está en curso."""

    def __init__(self) -> None:
        """Inicializa el control del robot y el publicador de la trayectoria actual."""
        super().__init__()
        self._publicador_trayectoria = self.create_publisher(
            String, TOPIC_TRAYECTORIA_ACTUAL, TAMANO_COLA
        )

    def ejecutar_todas(self) -> None:
        """Lleva el robot a la postura inicial y ejecuta las tres trayectorias."""
        # Movimiento de preparación: no se anuncia, así que no se graba
        self.mover_articulaciones(POSTURA_INICIAL)

        self._ejecutar("joint", self._trayectoria_joint)
        self._ejecutar("lineal", self._trayectoria_lineal)
        self._ejecutar("circular", self._trayectoria_circular)

    def _ejecutar(self, nombre: str, trayectoria: Callable[[], bool]) -> None:
        """Anuncia una trayectoria, la ejecuta y avisa cuando termina."""
        self.get_logger().info(f"Iniciando trayectoria '{nombre}'")
        self._anunciar(nombre)
        exito = trayectoria()
        self._anunciar(SIN_TRAYECTORIA)

        if exito:
            self.get_logger().info(f"Trayectoria '{nombre}' completada")
        else:
            self.get_logger().warn(f"La trayectoria '{nombre}' no se ha podido completar")

    def _anunciar(self, nombre: str) -> None:
        """Publica el nombre de la trayectoria en curso."""
        mensaje = String()
        mensaje.data = nombre
        self._publicador_trayectoria.publish(mensaje)

    def _trayectoria_joint(self) -> bool:
        """Movimiento en espacio articular: ida a POSTURA_JOINT y vuelta."""
        return (
            self.mover_articulaciones(POSTURA_JOINT)
            and self.mover_articulaciones(POSTURA_INICIAL)
        )

    def _trayectoria_lineal(self) -> bool:
        """Cuadrado vertical recorrido en líneas rectas por el efector final."""
        pose = self.pose_actual()
        vertices: list[Pose] = []
        for desplazamiento_y, desplazamiento_z in DESPLAZAMIENTOS_CUADRADO:
            pose.position.y += desplazamiento_y
            pose.position.z += desplazamiento_z
            vertices.append(copy.deepcopy(pose))
        return self.mover_trayectoria(vertices)

    def _trayectoria_circular(self) -> bool:
        """Circunferencia horizontal recorrida por el efector final.

        El centro se coloca a un radio de distancia de la posición actual,
        para que la circunferencia empiece justo donde está el robot.
        """
        pose_inicial = self.pose_actual()
        centro_x = pose_inicial.position.x - RADIO_CIRCULO_M
        centro_y = pose_inicial.position.y

        puntos: list[Pose] = []
        # Se empieza en 1 porque el punto de ángulo 0 es la posición actual
        for indice in range(1, PUNTOS_CIRCULO + 1):
            angulo = 2 * pi * indice / PUNTOS_CIRCULO
            pose = copy.deepcopy(pose_inicial)
            pose.position.x = centro_x + RADIO_CIRCULO_M * cos(angulo)
            pose.position.y = centro_y + RADIO_CIRCULO_M * sin(angulo)
            puntos.append(pose)
        return self.mover_trayectoria(puntos)


def main() -> None:
    """Punto de entrada del nodo."""
    rclpy.init()
    nodo = EjecutorTrayectorias()
    nodo.ejecutar_todas()
    nodo.ejecutor.shutdown()
    nodo.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()