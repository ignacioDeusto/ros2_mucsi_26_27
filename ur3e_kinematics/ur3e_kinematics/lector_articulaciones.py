#!/usr/bin/python3
"""Nodo que lee el estado de las articulaciones del brazo UR3e."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

TOPIC_ESTADO_ARTICULACIONES = "/joint_states"
TAMANO_COLA = 10
PERIODO_IMPRESION_S = 1.0  # Segundos entre impresiones, para no saturar la terminal

# Articulaciones del brazo en orden base -> muñeca.
# En /joint_states llegan en otro orden y mezcladas con las de la pinza,
# por eso siempre se buscan por nombre y nunca por posición en la lista.
ARTICULACIONES_BRAZO = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class LectorArticulaciones(Node):
    """Suscriptor a /joint_states que extrae los datos del brazo UR3e."""

    def __init__(self) -> None:
        """Crea el nodo y la suscripción al topic de estado de las articulaciones."""
        super().__init__("lector_articulaciones")
        self.create_subscription(
            JointState, TOPIC_ESTADO_ARTICULACIONES, self._callback_estado, TAMANO_COLA
        )

    def _callback_estado(self, mensaje: JointState) -> None:
        """Procesa cada mensaje recibido, descartando los que no son del brazo."""
        if not self._es_mensaje_del_brazo(mensaje):
            return

        posiciones = self._extraer_posiciones(mensaje)
        self.get_logger().info(
            f"Posiciones (rad): {[round(p, 3) for p in posiciones]}",
            throttle_duration_sec=PERIODO_IMPRESION_S,
        )

    @staticmethod
    def _es_mensaje_del_brazo(mensaje: JointState) -> bool:
        """Indica si el mensaje contiene todas las articulaciones del brazo.

        En /joint_states también publica el driver de la pinza RG2, cuyos
        mensajes solo traen las articulaciones de los dedos.
        """
        return all(nombre in mensaje.name for nombre in ARTICULACIONES_BRAZO)

    @staticmethod
    def _extraer_posiciones(mensaje: JointState) -> list[float]:
        """Devuelve las posiciones ordenadas según ARTICULACIONES_BRAZO."""
        posicion_por_nombre = dict(zip(mensaje.name, mensaje.position))
        return [posicion_por_nombre[nombre] for nombre in ARTICULACIONES_BRAZO]


def main() -> None:
    """Punto de entrada del nodo."""
    rclpy.init()
    nodo = LectorArticulaciones()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()