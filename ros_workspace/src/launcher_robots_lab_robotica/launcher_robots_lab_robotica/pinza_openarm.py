"""Nodo puente de la pinza del OpenArm: sirve la acción GripperCommand en milímetros.

El controlador nativo `robot_<id>_gripper_controller` (position_controllers/GripperActionController
de ros2_controllers) recibe la posición de la articulación finger_joint1 en radianes, con
signo distinto según el brazo (izquierdo: abre en positivo; derecho: abre en negativo).
Este nodo acepta la anchura total entre dedos en milímetros (igual que la acción de la
RG2), la convierte al ángulo del dedo y reenvía la orden al controlador, presentando el
resultado y el feedback también en mm.

Geometría de la conversión (mallas de openarm_description, pinch_gripper): el pivote de
cada dedo está a 18 mm del plano de simetría y la cara de agarre a 95,9 mm del pivote.
Con el dedo a |θ| rad la separación entre las dos caras es
    2 · (18 mm · (1 − cos |θ|) + 95,9 mm · sen |θ|)
que en la pinza totalmente abierta (|θ| = 0,7854 rad) da unos 146,2 mm. La conversión es
lineal en ese recorrido; a lo largo de la cara de agarre (11,8 mm) el error es de ±8 mm.
"""

import threading
from time import monotonic, sleep

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from launcher_robots_lab_robotica.robots import (
    ANCHURA_MAXIMA_OPENARM_MM as ANCHURA_MAXIMA_MM,
    LADOS_OPENARM,
    RAD_ABIERTA_OPENARM as RAD_ABIERTA,
    SIGNO_ABIERTA_OPENARM as SIGNO_ABIERTA,
)

# Distancia del plano de simetría a la cara de agarre con la pinza totalmente abierta;
# ANCHURA_MAXIMA_MM = 2 · DESPLAZAMIENTO_M · 1000
DESPLAZAMIENTO_M = ANCHURA_MAXIMA_MM / 2000.0
GAP_ABIERTA_M = 2.0 * DESPLAZAMIENTO_M
FRECUENCIA_HZ = 60.0
TIMEOUT_S = 10.0
TOLERANCIA_MM = 2.0  # Misma tolerancia que la del nodo de la RG2


def anchura_a_rad(anchura_mm: float) -> float:
    """Convierte la anchura entre dedos (mm) en el ángulo de finger_joint1 (rad)."""
    return (anchura_mm / 1000.0) / GAP_ABIERTA_M * RAD_ABIERTA


def rad_a_anchura(rad: float) -> float:
    """Convierte el ángulo de finger_joint1 (rad) en la anchura entre dedos (mm)."""
    return rad / RAD_ABIERTA * GAP_ABIERTA_M * 1000.0


class PinzaOpenArm(Node):

    def __init__(self) -> None:
        super().__init__("pinza_openarm")
        self.declare_parameter("id_brazo", "")
        self.declare_parameter("lado", "izq")
        self.id_brazo = self.get_parameter("id_brazo").value
        self.lado = self.get_parameter("lado").value
        if not self.id_brazo:
            raise RuntimeError("Falta el parámetro id_brazo (p. ej. '301_izq')")
        if self.lado not in SIGNO_ABIERTA:
            raise RuntimeError(f"Parámetro lado inválido: '{self.lado}' (use 'izq' o 'der')")
        self.signo = SIGNO_ABIERTA[self.lado]
        self.accion_controlador = f"/robot_{self.id_brazo}_gripper_controller/gripper_cmd"

        self.__candado = threading.Lock()
        self.__posicion_rad = 0.0
        self.__stalled = False

        # El joint_state_broadcaster no publica las articulaciones miméticas (no hay
        # soporte en Jazzy), así que este nodo publica finger_joint2 (mimic de
        # finger_joint1 con multiplicador 1) en /joint_states para RViz y MoveIt
        self.articulacion_pinza = f"{self.id_brazo}_finger_joint1"
        self.articulacion_mimica = f"{self.id_brazo}_finger_joint2"
        self.__posicion_pinza = 0.0
        self.create_subscription(JointState, "/joint_states", self.__cb_articulaciones, 10)
        self.publicador_mimico = self.create_publisher(JointState, "/joint_states", 10)
        self.temporizador_mimico = self.create_timer(0.5, self.__publicar_mimico)

        grupo = ReentrantCallbackGroup()
        self.cliente_controlador = ActionClient(
            self, GripperCommand, self.accion_controlador, callback_group=grupo)
        self.servidor = ActionServer(
            self,
            GripperCommand,
            "gripper_action",
            execute_callback=self.__ejecutar,
            goal_callback=lambda objetivo: GoalResponse.ACCEPT,
            cancel_callback=lambda objetivo: CancelResponse.ACCEPT,
            callback_group=grupo,
        )
        self.get_logger().info(
            f"Pinza del brazo {self.id_brazo}: /robot_{self.id_brazo}/gripper_action (mm) → "
            f"{self.accion_controlador}")

    @staticmethod
    def __esperar(future):
        while not future.done():
            sleep(0.01)
        return future.result()

    def __cb_feedback(self, feedback_msg):
        # rclpy pasa el mensaje envuelto (con goal_id y feedback), no el feedback a secas.
        # Con el hardware simulado la pinza llega al objetivo al instante y puede que no
        # haya ningún feedback: el resultado del controlador es la referencia fiable
        feedback = feedback_msg.feedback
        with self.__candado:
            self.__posicion_rad = feedback.position
            self.__stalled = feedback.stalled

    def __cb_articulaciones(self, msg: JointState) -> None:
        for nombre, posicion in zip(msg.name, msg.position):
            if nombre == self.articulacion_pinza:
                self.__posicion_pinza = posicion

    def __publicar_mimico(self) -> None:
        # Se publica de forma continua (barato y garantiza que MoveIt, que arranca
        # después, vea la articulación mimética en /joint_states)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.articulacion_mimica]
        msg.position = [self.__posicion_pinza]
        self.publicador_mimico.publish(msg)

    def __ejecutar(self, goal_handle):
        objetivo = goal_handle.request.command
        anchura = min(max(objetivo.position, 0.0), ANCHURA_MAXIMA_MM)
        if objetivo.position != anchura:
            self.get_logger().warn(
                f"Anchura {objetivo.position} fuera de rango [0, {ANCHURA_MAXIMA_MM}]; "
                f"se recorta a {anchura}")

        objetivo_controlador = GripperCommand.Goal()
        objetivo_controlador.command.position = self.signo * min(
            anchura_a_rad(anchura), RAD_ABIERTA)
        objetivo_controlador.command.max_effort = objetivo.max_effort
        self.get_logger().info(
            f"Anchura objetivo: {anchura:.1f} mm "
            f"({objetivo_controlador.command.position:+.4f} rad)")

        if not self.cliente_controlador.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"Controlador {self.accion_controlador} no disponible")
            goal_handle.abort()
            return GripperCommand.Result()

        with self.__candado:
            self.__posicion_rad = 0.0
            self.__stalled = False
        gestion = self.__esperar(
            self.cliente_controlador.send_goal_async(objetivo_controlador, self.__cb_feedback))
        if not gestion.accepted:
            goal_handle.abort()
            return GripperCommand.Result()

        fut_resultado = gestion.get_result_async()
        resultado = GripperCommand.Result()
        inicio = monotonic()
        while not fut_resultado.done():
            if goal_handle.is_cancel_requested:
                gestion.cancel_goal()
                goal_handle.canceled()
                return GripperCommand.Result()
            with self.__candado:
                posicion = self.__posicion_rad
                stalled = self.__stalled
            anchura_actual = rad_a_anchura(abs(posicion))

            feedback = GripperCommand.Feedback()
            feedback.position = anchura_actual
            feedback.stalled = bool(stalled)
            feedback.effort = objetivo.max_effort
            feedback.reached_goal = abs(anchura - anchura_actual) < TOLERANCIA_MM
            goal_handle.publish_feedback(feedback)

            resultado.position = anchura_actual
            resultado.effort = objetivo.max_effort
            resultado.stalled = bool(stalled)

            if monotonic() - inicio > TIMEOUT_S:
                gestion.cancel_goal()
                break
            sleep(1.0 / FRECUENCIA_HZ)

        # Posición final según el resultado del controlador (rad, con signo por lado)
        if fut_resultado.done():
            r = fut_resultado.result().result
            resultado.position = rad_a_anchura(abs(r.position))
            resultado.stalled = bool(r.stalled)

        if abs(anchura - resultado.position) < TOLERANCIA_MM or resultado.stalled:
            resultado.reached_goal = True
            goal_handle.succeed()
            self.get_logger().info("Objetivo alcanzado")
        else:
            resultado.reached_goal = False
            goal_handle.abort()
            self.get_logger().warn("Objetivo NO alcanzado")
        return resultado


def main(args=None) -> None:
    rclpy.init(args=args)
    nodo = PinzaOpenArm()
    executor = MultiThreadedExecutor()
    executor.add_node(nodo)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
