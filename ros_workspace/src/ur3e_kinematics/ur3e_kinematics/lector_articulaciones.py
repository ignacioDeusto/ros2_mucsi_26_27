#!/usr/bin/python3
"""Nodo que lee el estado de las articulaciones del brazo UR3e y calcula
su velocidad y aceleración."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

TOPIC_ESTADO_ARTICULACIONES = "/joint_states"
TAMANO_COLA = 10
PERIODO_IMPRESION_S = 1.0  # Segundos entre impresiones, para no saturar la terminal
NANOSEGUNDOS_A_SEGUNDOS = 1e-9

# El robot publica cada ~2 ms. Derivar con intervalos tan pequeños amplifica
# el ruido, así que solo se procesa una muestra cada 10 ms (100 Hz).
PERIODO_MUESTREO_S = 0.01

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
    """Suscriptor a /joint_states que calcula la cinemática del brazo UR3e."""

    def __init__(self) -> None:
        """Crea el nodo y la suscripción al topic de estado de las articulaciones."""
        super().__init__("lector_articulaciones")
        self.create_subscription(
            JointState, TOPIC_ESTADO_ARTICULACIONES, self._callback_estado, TAMANO_COLA
        )

        # Última muestra procesada: hace falta para derivar numéricamente
        self._tiempo_anterior: float | None = None
        self._posiciones_anteriores: list[float] = []
        self._velocidades_anteriores: list[float] = []

    def _callback_estado(self, mensaje: JointState) -> None:
        """Procesa cada mensaje del brazo y calcula velocidad y aceleración."""
        if not self._es_mensaje_del_brazo(mensaje):
            return

        tiempo = self._tiempo_del_mensaje(mensaje)
        posiciones = self._extraer_posiciones(mensaje)

        # Primera muestra: aún no hay con qué comparar. Se asume que el robot
        # parte del reposo, así que la velocidad inicial es cero.
        if self._tiempo_anterior is None:
            self._guardar_muestra(tiempo, posiciones, [0.0] * len(posiciones))
            return

        intervalo = tiempo - self._tiempo_anterior
        if intervalo < PERIODO_MUESTREO_S:
            return

        velocidades = self._derivar(posiciones, self._posiciones_anteriores, intervalo)
        aceleraciones = self._derivar(velocidades, self._velocidades_anteriores, intervalo)
        self._guardar_muestra(tiempo, posiciones, velocidades)

        self.get_logger().info(
            f"\n  Posición (rad):        {self._formatear(posiciones)}"
            f"\n  Velocidad (rad/s):     {self._formatear(velocidades)}"
            f"\n  Aceleración (rad/s^2): {self._formatear(aceleraciones)}",
            throttle_duration_sec=PERIODO_IMPRESION_S,
        )

    def _guardar_muestra(
        self, tiempo: float, posiciones: list[float], velocidades: list[float]
    ) -> None:
        """Recuerda la muestra actual para usarla en el siguiente cálculo."""
        self._tiempo_anterior = tiempo
        self._posiciones_anteriores = posiciones
        self._velocidades_anteriores = velocidades

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

    @staticmethod
    def _tiempo_del_mensaje(mensaje: JointState) -> float:
        """Convierte la marca de tiempo del mensaje a segundos."""
        sello = mensaje.header.stamp
        return sello.sec + sello.nanosec * NANOSEGUNDOS_A_SEGUNDOS

    @staticmethod
    def _derivar(
        valores_actuales: list[float], valores_anteriores: list[float], intervalo: float
    ) -> list[float]:
        """Derivada numérica (diferencia hacia atrás) articulación a articulación."""
        return [
            (actual - anterior) / intervalo
            for actual, anterior in zip(valores_actuales, valores_anteriores)
        ]

    @staticmethod
    def _formatear(valores: list[float]) -> str:
        """Redondea los valores para mostrarlos de forma legible."""
        return str([round(valor, 3) for valor in valores])


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