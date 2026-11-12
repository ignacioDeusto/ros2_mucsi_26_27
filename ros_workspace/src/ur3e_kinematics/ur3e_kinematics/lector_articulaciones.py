#!/usr/bin/python3
"""Nodo que lee el estado de las articulaciones del brazo UR3e, calcula su
velocidad y aceleración y guarda los datos de cada trayectoria en un CSV."""

import signal
import threading
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from ur3e_kinematics.comunicacion import (
    QOS_TRAYECTORIA,
    SIN_TRAYECTORIA,
    TOPIC_TRAYECTORIA_ACTUAL,
)
from ur3e_kinematics.exportador_csv import ExportadorCsv
from ur3e_kinematics.muestra_cinematica import MuestraCinematica
from ur3e_kinematics_interfaces.msg import EstadoCinematico

TOPIC_ESTADO_ARTICULACIONES = "/joint_states"
TOPIC_ESTADO_CINEMATICO = "/estado_cinematico"
TAMANO_COLA = 10
PERIODO_IMPRESION_S = 1.0  # Segundos entre impresiones, para no saturar la terminal
NANOSEGUNDOS_A_SEGUNDOS = 1e-9

# Derivar dos veces con muestras cada ~2 ms amplifica el ruido (picos de
# 37 rad/s^2 con 10 ms). Con 50 ms se filtra sin perder detalle.
PERIODO_MUESTREO_S = 0.05

# Por debajo de esta velocidad en todas las articulaciones se considera que el
# robot está parado. En reposo, la derivada numérica no pasa de ~0.001 rad/s.
UMBRAL_MOVIMIENTO_RAD_S = 0.005

# Carpeta compartida con el host: los CSV se pueden abrir desde fuera
DIRECTORIO_DATOS = Path.home() / "ros_workspace" / "datos_cinematicos"

# Orden base -> muñeca. En /joint_states llegan desordenadas y con las de la
# pinza, así que se buscan por nombre.
ARTICULACIONES_BRAZO = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class LectorArticulaciones(Node):
    """Suscriptor a /joint_states que calcula la cinemática del brazo UR3e
    y la guarda separada por trayectoria, solo mientras el robot se mueve."""

    def __init__(self) -> None:
        """Crea el nodo y las suscripciones al estado del robot y a la trayectoria actual."""
        super().__init__("lector_articulaciones")
        self.create_subscription(
            JointState, TOPIC_ESTADO_ARTICULACIONES, self._callback_estado, TAMANO_COLA
        )

        # Última muestra procesada: hace falta para derivar numéricamente
        self._tiempo_anterior: float | None = None
        self._posiciones_anteriores: list[float] = []
        # None mientras aún no se ha podido calcular ninguna velocidad
        self._velocidades_anteriores: list[float] | None = None

        # Saber qué trayectoria se ejecuta permite guardar sus datos por separado
        self.create_subscription(
            String, TOPIC_TRAYECTORIA_ACTUAL, self._callback_trayectoria, QOS_TRAYECTORIA
        )
        self._trayectoria_actual: str = SIN_TRAYECTORIA
        self._muestras_por_trayectoria: dict[str, list[MuestraCinematica]] = {}
        self._exportador = ExportadorCsv(DIRECTORIO_DATOS, ARTICULACIONES_BRAZO)

        # Permite avisar por el log solo cuando el robot arranca o se para
        self._robot_en_movimiento: bool = False

        self._publicador_estado = self.create_publisher(
            EstadoCinematico, TOPIC_ESTADO_CINEMATICO, TAMANO_COLA
        )

    def finalizar(self) -> None:
        """Cierra la trayectoria en curso, si la hay. Se llama al apagar el nodo.

        El nodo de trayectorias termina justo después de anunciar el final de la
        última trayectoria, y ese mensaje puede no llegar a tiempo.
        """
        if self._trayectoria_actual != SIN_TRAYECTORIA:
            self._finalizar_trayectoria()
            self._trayectoria_actual = SIN_TRAYECTORIA

    def _callback_trayectoria(self, mensaje: String) -> None:
        """Cambia la trayectoria en la que se guardan las muestras."""
        nueva_trayectoria = mensaje.data
        if nueva_trayectoria == self._trayectoria_actual:
            return

        if self._trayectoria_actual != SIN_TRAYECTORIA:
            self._finalizar_trayectoria()

        self._trayectoria_actual = nueva_trayectoria
        # Se reinicia la derivación para no mezclar muestras de dos trayectorias
        self._tiempo_anterior = None

        if nueva_trayectoria != SIN_TRAYECTORIA:
            self._muestras_por_trayectoria[nueva_trayectoria] = []
            self.get_logger().info(f"Grabando trayectoria '{nueva_trayectoria}'")

    def _finalizar_trayectoria(self) -> None:
        """Exporta a CSV las muestras de la trayectoria que acaba de terminar."""
        muestras = self._muestras_por_trayectoria[self._trayectoria_actual]
        ruta = self._exportador.guardar(self._trayectoria_actual, muestras)
        self.get_logger().info(
            f"Trayectoria '{self._trayectoria_actual}' finalizada: "
            f"{len(muestras)} muestras guardadas en {ruta}"
        )

    def _callback_estado(self, mensaje: JointState) -> None:
        """Procesa cada mensaje del brazo, calcula velocidad y aceleración y lo guarda."""
        if self._trayectoria_actual == SIN_TRAYECTORIA:
            return
        if not self._es_mensaje_del_brazo(mensaje):
            return

        tiempo = self._tiempo_del_mensaje(mensaje)
        posiciones = self._extraer_posiciones(mensaje)

        # Primera muestra: no se supone reposo, porque el aviso de trayectoria
        # puede llegar tarde con el robot ya en marcha.
        if self._tiempo_anterior is None:
            self._guardar_muestra(tiempo, posiciones, None)
            return

        intervalo = tiempo - self._tiempo_anterior
        if intervalo < PERIODO_MUESTREO_S:
            return

        velocidades = self._derivar(posiciones, self._posiciones_anteriores, intervalo)
        velocidades_anteriores = self._velocidades_anteriores
        # Se recuerda aunque el robot esté quieto: la siguiente derivada la necesita
        self._guardar_muestra(tiempo, posiciones, velocidades)

        # Segunda muestra: aún no hay velocidad anterior para la aceleración
        if velocidades_anteriores is None:
            return
        aceleraciones = self._derivar(velocidades, velocidades_anteriores, intervalo)

        # Ingesta inteligente: los periodos con el robot quieto se descartan
        if not self._actualizar_estado_movimiento(velocidades):
            return

        muestra = MuestraCinematica(tiempo, posiciones, velocidades, aceleraciones)
        self._muestras_por_trayectoria[self._trayectoria_actual].append(muestra)
        self._publicar_estado(mensaje.header.stamp, muestra)

        self.get_logger().info(
            f"\n  Posición (rad):        {self._formatear(posiciones)}"
            f"\n  Velocidad (rad/s):     {self._formatear(velocidades)}"
            f"\n  Aceleración (rad/s^2): {self._formatear(aceleraciones)}",
            throttle_duration_sec=PERIODO_IMPRESION_S,
        )

    def _guardar_muestra(
        self, tiempo: float, posiciones: list[float], velocidades: list[float] | None
    ) -> None:
        """Recuerda la muestra actual para usarla en el siguiente cálculo."""
        self._tiempo_anterior = tiempo
        self._posiciones_anteriores = posiciones
        self._velocidades_anteriores = velocidades

    def _actualizar_estado_movimiento(self, velocidades: list[float]) -> bool:
        """Indica si el robot se está moviendo y avisa cuando arranca o se para.

        Se considera que se mueve si alguna articulación supera el umbral.
        """
        en_movimiento = any(
            abs(velocidad) > UMBRAL_MOVIMIENTO_RAD_S for velocidad in velocidades
        )
        if en_movimiento != self._robot_en_movimiento:
            if en_movimiento:
                self.get_logger().info("Robot en movimiento: grabando")
            else:
                self.get_logger().info("Robot parado: sin grabar")
            self._robot_en_movimiento = en_movimiento
        return en_movimiento

    def _publicar_estado(self, sello: Time, muestra: MuestraCinematica) -> None:
        """Publica la muestra en el mensaje personalizado EstadoCinematico.

        Se reutiliza el sello de tiempo de /joint_states para que el mensaje
        quede vinculado al instante real en que se midió.
        """
        estado = EstadoCinematico()
        estado.header.stamp = sello
        estado.trayectoria = self._trayectoria_actual
        estado.nombres_articulaciones = ARTICULACIONES_BRAZO
        estado.posiciones = muestra.posiciones
        estado.velocidades = muestra.velocidades
        estado.aceleraciones = muestra.aceleraciones
        self._publicador_estado.publish(estado)

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
    # Ctrl+C solo pide parar y el bucle termina en la siguiente vuelta. Si ROS
    # cerrase el nodo de golpe, podría hacerlo a mitad de leer un mensaje de
    # /joint_states y saltaría un RuntimeError.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nodo = LectorArticulaciones()
    parada = threading.Event()
    signal.signal(signal.SIGINT, lambda _senal, _marco: parada.set())
    try:
        while not parada.is_set():
            rclpy.spin_once(nodo, timeout_sec=0.1)
    finally:
        # Al pulsar Ctrl+C se guarda la última trayectoria antes de destruir el nodo
        nodo.finalizar()
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
