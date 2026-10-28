#!/usr/bin/python3
"""Control de los dos brazos de un cuerpo OpenArm del laboratorio con MoveIt 2, simétricos.

Cada cuerpo (./launcher.sh sim 301, ./launcher.sh sim 302) lleva dos brazos de 7
articulaciones y una pinza cada uno. El URDF construye el izquierdo como imagen especular
del derecho (reflexión por el plano y = 0 en "world"), así que los mismos valores
articulares dejan los dos brazos en poses opuestas y cada método mueve los dos a la vez:
los objetivos articulares van al grupo combinado robots_<cuerpo> del SRDF (una sola
trayectoria para los dos) y, en poses y cartesianos, cada brazo se planifica por su lado
(el izquierdo con la pose reflejada) y las dos ejecuciones se lanzan a la vez.

El identificador del cuerpo (ID_ROBOT abajo) selecciona cuál se controla:
  - "301"  -> cuerpo OpenArm 301 (brazos 301_izq y 301_der)
  - "302"  -> cuerpo OpenArm 302 (brazos 302_izq y 302_der)
"""

import copy
import threading
import time
from math import pi
from typing import List
from typing import Tuple

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

SISTEMA_REFERENCIA = "world"  # Frame raíz del URDF del laboratorio

# Brazos del cuerpo, en el orden en que se controlan
LADOS = ("izq", "der")

# Articulaciones y efector final de cada brazo, sin prefijo
ARTICULACIONES = [f"joint{i}" for i in range(1, 8)]
EFECTOR_FINAL = "grasp_frame"  # Punto de agarre entre los dedos de la pinza
# Acción de la pinza (dos dedos rotativos): command.position es la anchura entre los
# dedos en mm; el nodo puente de cada pinza hace la conversión de su lado (el signo de
# apertura cambia con el lado)
ACCION_PINZA = "gripper_action"
ANCHURA_MAXIMA = 146.2         # Anchura máxima de la pinza (mm)
# Pose inicial de los dos brazos (radianes): con articulaciones a cero quedan totalmente
# extendidos hacia abajo (singular, no pueden bajar más), así que se usa la del SRDF
# "hands_up" (codo flexionado)
POSICION_INICIAL = [0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0]
DESPLAZAMIENTO = 0.05          # Desplazamiento entre poses del ejemplo (m), menor que en los UR3e
FACTOR_ESCALA = 0.1            # Velocidad y aceleración, fracción de las máximas


def pose_espejo(pose: Pose) -> Pose:
    """
    Refleja la pose por el plano de simetría del cuerpo (y = 0 en "world").
    La orientación (x, y, z, w) pasa a (-x, y, -z, w).

    Args:
        pose (Pose): pose en "world" a reflejar.

    Returns:
        Pose: nueva pose reflejada (la original no se modifica)
    """
    espejo = copy.deepcopy(pose)
    espejo.position.y = -pose.position.y
    espejo.orientation.x = -pose.orientation.x
    espejo.orientation.z = -pose.orientation.z
    return espejo


class ControlOpenArm(Node):
    """
    Control de los dos brazos de un cuerpo OpenArm con las acciones y servicios de
    move_group (MoveIt 2). Los dos brazos se mueven siempre simétricamente.
    """

    def __init__(self, id_cuerpo: str) -> None:
        """
        Args:
            id_cuerpo (str): cuerpo a controlar ("301" o "302").
        """
        super().__init__("control_openarm")
        if not id_cuerpo:
            raise ValueError(
                'Falta el identificador del cuerpo ("301" o "302"); ver la cabecera del fichero')
        self.id_cuerpo = id_cuerpo
        self.brazos = [f"{id_cuerpo}_{lado}" for lado in LADOS]
        self.group_name = f"robots_{id_cuerpo}"  # Grupo combinado del SRDF (los dos brazos)
        self.grupo_brazo = {b: f"robot_{b}" for b in self.brazos}
        self.efector_final = {b: f"{b}_{EFECTOR_FINAL}" for b in self.brazos}
        self.articulaciones_brazo = {b: [f"{b}_{j}" for j in ARTICULACIONES] for b in self.brazos}
        articulaciones_total = [a for b in self.brazos for a in self.articulaciones_brazo[b]]

        self.__estado_articulaciones = {}

        self.create_subscription(JointState, "/joint_states", self.__cb_joint_states, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.move_group_client = ActionClient(self, MoveGroup, "move_action")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self.cartesian_client = self.create_client(GetCartesianPath, "compute_cartesian_path")
        self.scene_client = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.clientes_pinza = {b: ActionClient(
            self, GripperCommand, f"robot_{b}/{ACCION_PINZA}") for b in self.brazos}
        # Controlador de trayectoria de cada brazo (ver lanzador_openarm.py)
        self.clientes_trayectoria = {b: ActionClient(
            self, FollowJointTrajectory,
            f"/robot_{b}_joint_trajectory_controller/follow_joint_trajectory")
            for b in self.brazos}

        # Callbacks (tf, /joint_states, ...) en un hilo aparte
        self.ejecutor = MultiThreadedExecutor()
        self.ejecutor.add_node(self)
        self.hilo = threading.Thread(target=self.ejecutor.spin, daemon=True)
        self.hilo.start()

        self.get_logger().info(f"Cuerpo {self.id_cuerpo} ({self.group_name}): "
                               f"esperando a MoveIt (move_group)...")
        self.move_group_client.wait_for_server()
        self.execute_client.wait_for_server()
        self.cartesian_client.wait_for_service()
        self.scene_client.wait_for_service()
        for cliente in self.clientes_trayectoria.values():
            cliente.wait_for_server()
        while not all(j in self.__estado_articulaciones for j in articulaciones_total):
            time.sleep(0.05)

        self.añadir_suelo()

    def __cb_joint_states(self, msg: JointState) -> None:
        """
        Actualiza el estado de las articulaciones.

        Args:
            msg (JointState): mensaje recibido de /joint_states.
        """
        for nombre, posicion in zip(msg.name, msg.position):
            self.__estado_articulaciones[nombre] = posicion

    @staticmethod
    def __esperar(future):
        """
        Bloquea hasta que responde una petición y devuelve su resultado.

        Args:
            future (Future): petición pendiente.

        Returns:
            Any: resultado de la petición.
        """
        while not future.done():
            time.sleep(0.01)
        return future.result()

    def articulaciones_actuales(self) -> list:
        """
        Posiciones actuales de las 14 articulaciones (rad): izq y luego der.

        Returns:
            list: posiciones articulares de los dos brazos.
        """
        return [self.__estado_articulaciones[a]
                for b in self.brazos for a in self.articulaciones_brazo[b]]

    def __enviar_movimientos(self, objetivos: List[Tuple[str, Constraints]],
                             wait: bool = True) -> bool:
        """
        Envía a move_group un movimiento por cada (grupo, restricciones) y, si wait=True,
        espera a que terminen todos. Con un solo objetivo lo planifica y ejecuta move_group.
        Con varios (un brazo cada uno) los planifica uno detrás de otro, porque move_group
        no admite ejecuciones simultáneas, y manda las trayectorias a la vez a los
        controladores de cada brazo.

        Args:
            objetivos (List[Tuple[str, Constraints]]): (grupo de MoveIt, restricciones del objetivo).
            wait (bool): si es False vuelve al aceptarse los objetivos, sin esperar a que acaben.

        Returns:
            bool: True si se aceptaron (y, con wait, terminaron con éxito) todos.
        """
        un_objetivo = len(objetivos) == 1
        trayectorias = []
        for grupo, restricciones in objetivos:
            goal = MoveGroup.Goal()
            goal.request.group_name = grupo
            goal.request.num_planning_attempts = 10
            goal.request.allowed_planning_time = 5.0
            goal.request.max_velocity_scaling_factor = FACTOR_ESCALA
            goal.request.max_acceleration_scaling_factor = FACTOR_ESCALA
            goal.request.goal_constraints.append(restricciones)
            goal.request.start_state.is_diff = True
            goal.planning_options.planning_scene_diff.is_diff = True
            goal.planning_options.planning_scene_diff.robot_state.is_diff = True
            goal.planning_options.plan_only = not un_objetivo

            manejador = self.__esperar(self.move_group_client.send_goal_async(goal))
            if not manejador.accepted:
                self.get_logger().error(f"move_group ha rechazado el objetivo de {grupo}")
                return False
            if un_objetivo and not wait:
                return True
            resultado = self.__esperar(manejador.get_result_async()).result
            if resultado.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(
                    f"Fallo al mover {grupo}: código de error {resultado.error_code.val}")
                return False
            trayectorias.append((grupo, resultado.planned_trajectory.joint_trajectory))
        if un_objetivo:
            return True

        envios = []
        for grupo, trayectoria in trayectorias:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trayectoria
            cliente = self.clientes_trayectoria[grupo.removeprefix("robot_")]
            envios.append(cliente.send_goal_async(goal))
        manejadores = [self.__esperar(envio) for envio in envios]
        if any(not m.accepted for m in manejadores):
            self.get_logger().error("Un controlador ha rechazado la trayectoria")
            return False
        if not wait:
            return True
        return all(self.__esperar(m.get_result_async()).result.error_code
                   == FollowJointTrajectory.Result.SUCCESSFUL for m in manejadores)

    def mover_articulaciones(self, joint_goal: List[float], wait: bool = True) -> bool:
        """
        Mueve los dos brazos con una sola trayectoria, al grupo combinado
        robots_<cuerpo>: los mismos 7 valores (rad) para los dos, con los que quedan
        en poses especulares.

        Args:
            joint_goal (List[float]): 7 posiciones articulares (rad), válidas para cada brazo.
            wait (bool): si es False no espera a que termine el movimiento.

        Returns:
            bool: True si el movimiento se aceptó (y, con wait, terminó con éxito).
        """
        if len(joint_goal) != len(ARTICULACIONES):
            self.get_logger().error(
                f"Se esperaban {len(ARTICULACIONES)} articulaciones por brazo "
                f"y se han recibido {len(joint_goal)}")
            return False
        restricciones = Constraints()
        for brazo in self.brazos:
            for nombre, valor in zip(self.articulaciones_brazo[brazo], joint_goal):
                restricciones.joint_constraints.append(JointConstraint(
                    joint_name=nombre, position=float(valor),
                    tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
        return self.__enviar_movimientos([(self.group_name, restricciones)], wait)

    def pose_actual(self) -> Pose:
        """
        Pose del gripper del brazo derecho en "world", la más reciente del tf.
        El izquierdo está en su refleja (ver pose_espejo).

        Returns:
            Pose: pose del efector final derecho.
        """
        return self.__pose_actual_de(self.brazos[1])

    def __pose_actual_de(self, brazo: str) -> Pose:
        transformacion = self.tf_buffer.lookup_transform(
            SISTEMA_REFERENCIA, self.efector_final[brazo], rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=5.0))
        pose = Pose()
        pose.position.x = transformacion.transform.translation.x
        pose.position.y = transformacion.transform.translation.y
        pose.position.z = transformacion.transform.translation.z
        pose.orientation = transformacion.transform.rotation
        return pose

    def pose_a_stamped(self, pose: Pose) -> PoseStamped:
        """
        Pose con cabecera en "world", como las espera MoveIt.

        Args:
            pose (Pose): pose en "world".

        Returns:
            PoseStamped: la misma pose con la cabecera puesta.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = SISTEMA_REFERENCIA
        pose_stamped.pose = pose
        return pose_stamped

    def mover_a_pose(self, pose_goal: Pose, wait: bool = True) -> bool:
        """
        Mueve los dos grippers simétricamente: el derecho a la pose dada y el izquierdo
        a su refleja. Cada brazo se planifica en su grupo y las dos ejecuciones se lanzan
        a la vez.

        Args:
            pose_goal (Pose): pose en "world" del gripper derecho.
            wait (bool): si es False no espera a que termine el movimiento.

        Returns:
            bool: True si los dos movimientos se aceptaron (y, con wait, terminaron con éxito).
        """
        brazo_der, brazo_izq = self.brazos[1], self.brazos[0]
        objetivos = [
            (self.grupo_brazo[brazo_der],
             self.__restricciones_pose(pose_goal, brazo_der)),
            (self.grupo_brazo[brazo_izq],
             self.__restricciones_pose(pose_espejo(pose_goal), brazo_izq)),
        ]
        return self.__enviar_movimientos(objetivos, wait)

    def __restricciones_pose(self, pose_goal: Pose, brazo: str) -> Constraints:
        """
        Restricciones de posición (a 1 mm) y orientación (a 0.01) para el gripper del brazo.

        Args:
            pose_goal (Pose): pose objetivo en "world".
            brazo (str): identificador del brazo (p. ej. "301_izq").

        Returns:
            Constraints: restricciones para el objetivo de MoveIt.
        """
        restriccion_posicion = PositionConstraint()
        restriccion_posicion.header.frame_id = SISTEMA_REFERENCIA
        restriccion_posicion.link_name = self.efector_final[brazo]
        region = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        restriccion_posicion.constraint_region.primitives.append(region)
        restriccion_posicion.constraint_region.primitive_poses.append(pose_goal)
        restriccion_posicion.weight = 1.0

        restriccion_orientacion = OrientationConstraint()
        restriccion_orientacion.header.frame_id = SISTEMA_REFERENCIA
        restriccion_orientacion.link_name = self.efector_final[brazo]
        restriccion_orientacion.orientation = pose_goal.orientation
        restriccion_orientacion.absolute_x_axis_tolerance = 0.01
        restriccion_orientacion.absolute_y_axis_tolerance = 0.01
        restriccion_orientacion.absolute_z_axis_tolerance = 0.01
        restriccion_orientacion.weight = 1.0

        restricciones = Constraints()
        restricciones.position_constraints.append(restriccion_posicion)
        restricciones.orientation_constraints.append(restriccion_orientacion)
        return restricciones

    def añadir_caja_a_escena_de_planificación(self, pose_caja: Pose, name: str,
                                              tamaño: tuple = (.1, .1, .1)) -> None:
        """
        Añade una caja colisionable a la escena de planificación.

        Args:
            pose_caja (Pose): pose de la caja en "world".
            name (str): identificador del objeto en la escena.
            tamaño (tuple): dimensiones (x, y, z) en m.
        """
        caja = CollisionObject()
        caja.id = name
        caja.header.frame_id = SISTEMA_REFERENCIA
        caja.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(tamaño)))
        caja.primitive_poses.append(pose_caja)
        caja.operation = CollisionObject.ADD

        peticion = ApplyPlanningScene.Request()
        peticion.scene = PlanningScene()
        peticion.scene.is_diff = True
        peticion.scene.robot_state = RobotState(is_diff=True)
        peticion.scene.world.collision_objects.append(caja)
        self.__esperar(self.scene_client.call_async(peticion))

    def mover_trayectoria(self, poses: List[Pose], poses_izq: List[Pose] = None,
                          wait: bool = True) -> bool:
        """
        Recorre en línea recta las poses con los dos grippers: el derecho por poses y el
        izquierdo por sus reflejas, o por poses_izq si se dan (control independiente de
        cada brazo). Cada trayectoria se calcula en el grupo de su brazo y las dos
        ejecuciones se lanzan a la vez. Falla si algún brazo no puede pasar por todas
        (fracción < 1).

        Args:
            poses (List[Pose]): poses en "world" que recorre el gripper derecho.
            poses_izq (List[Pose]): poses que recorre el izquierdo; por defecto, las reflejas de poses.
            wait (bool): si es False no espera a que terminen las ejecuciones.

        Returns:
            bool: True si se aceptaron (y, con wait, terminaron con éxito) las dos ejecuciones.
        """
        if poses_izq is None:
            poses_izq = [pose_espejo(p) for p in poses]
        brazo_der, brazo_izq = self.brazos[1], self.brazos[0]
        peticiones = []
        for brazo, objetivos in ((brazo_der, poses), (brazo_izq, poses_izq)):
            poses_aux = copy.deepcopy(objetivos)
            poses_aux.insert(0, self.__pose_actual_de(brazo))

            peticion = GetCartesianPath.Request()
            peticion.header.frame_id = SISTEMA_REFERENCIA
            peticion.start_state.is_diff = True
            peticion.group_name = self.grupo_brazo[brazo]
            peticion.link_name = self.efector_final[brazo]
            peticion.waypoints = poses_aux
            peticion.max_step = 0.01
            peticion.jump_threshold = 0.0
            peticion.avoid_collisions = True
            peticion.max_velocity_scaling_factor = FACTOR_ESCALA
            peticion.max_acceleration_scaling_factor = FACTOR_ESCALA
            peticiones.append(peticion)

        respuestas = [self.__esperar(self.cartesian_client.call_async(p)) for p in peticiones]
        if any(r.fraction != 1.0 for r in respuestas):
            self.get_logger().error(
                f"Trayectoria cartesiana incompleta (fracciones {[r.fraction for r in respuestas]})")
            return False

        envios = []
        for brazo, respuesta in zip((brazo_der, brazo_izq), respuestas):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = respuesta.solution.joint_trajectory
            envios.append(self.clientes_trayectoria[brazo].send_goal_async(goal))
        manejadores = [self.__esperar(envio) for envio in envios]
        if any(not m.accepted for m in manejadores):
            self.get_logger().error("Un controlador ha rechazado la trayectoria")
            return False
        if not wait:
            return True
        return all(self.__esperar(m.get_result_async()).result.error_code
                   == FollowJointTrajectory.Result.SUCCESSFUL for m in manejadores)

    def añadir_suelo(self) -> None:
        """
        Caja en el plano del suelo para que MoveIt no planifique por debajo.
        """
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        pose_suelo.orientation.w = 1.0
        self.añadir_caja_a_escena_de_planificación(pose_suelo, "suelo", (2, 2, .05))

    def mover_pinza(self, anchura_dedos: float, fuerza: float) -> bool:
        """
        Abre o cierra las dos pinzas a la vez: anchura total entre los dedos, en mm (el
        nodo puente de cada pinza convierte al ángulo de su lado).

        Args:
            anchura_dedos (float): anchura total entre los dedos (mm), entre 0 y ANCHURA_MAXIMA.
            fuerza (float): esfuerzo máximo de la pinza.

        Returns:
            bool: True si las dos pinzas alcanzaron la anchura pedida.
        """
        if anchura_dedos < 0.0 or anchura_dedos > ANCHURA_MAXIMA:
            self.get_logger().warn(
                f"Anchura {anchura_dedos} fuera de rango [0, {ANCHURA_MAXIMA}] mm")
        for cliente in self.clientes_pinza.values():
            cliente.wait_for_server()

        envios = []
        for cliente in self.clientes_pinza.values():
            goal = GripperCommand.Goal()
            goal.command.position = anchura_dedos
            goal.command.max_effort = fuerza
            envios.append((cliente, cliente.send_goal_async(goal)))
        manejadores = [self.__esperar(envio) for _, envio in envios]
        if any(not m.accepted for m in manejadores):
            return False
        resultados = [self.__esperar(m.get_result_async()).result for m in manejadores]
        return all(r.reached_goal for r in resultados)


if __name__ == '__main__':
    rclpy.init()

    # Identificador del cuerpo a controlar; ver las opciones en la cabecera del fichero
    ID_ROBOT = "301"

    control = ControlOpenArm(ID_ROBOT)

    # Articulaciones iniciales de los dos brazos (SRDF "hands_up")
    control.mover_articulaciones(POSICION_INICIAL)
    paso = DESPLAZAMIENTO

    # Movimiento a una pose relativa a la actual (los dos grippers, en espejo)
    pose_actual = control.pose_actual()
    pose_actual.position.z -= paso
    control.mover_a_pose(pose_actual)

    # A partir de aquí se controlan los dos brazos de forma independiente: la trayectoria
    # del izquierdo es inversa en el eje vertical (si baja uno, sube el otro) y simétrica
    # en el otro eje
    poses = []  # Lista de poses que va a recorrer el brazo derecho

    # Pose 1
    pose_actual = control.pose_actual()
    inicio_z = pose_actual.position.z  # Altura de partida, igual en los dos brazos
    pose_actual.position.z += paso
    poses.append(copy.deepcopy(pose_actual))

    # Pose 2
    pose_actual.position.y += paso
    poses.append(copy.deepcopy(pose_actual))

    # Pose 3
    pose_actual.position.x += paso
    poses.append(copy.deepcopy(pose_actual))

    # Pose 4
    pose_actual.position.x -= paso
    pose_actual.position.y -= paso
    pose_actual.position.z -= paso
    poses.append(copy.deepcopy(pose_actual))

    # Poses del brazo izquierdo: espejo de las del derecho con la altura invertida
    poses_izq = []
    for pose in poses:
        pose_izq = pose_espejo(pose)
        pose_izq.position.z = 2 * inicio_z - pose.position.z
        poses_izq.append(pose_izq)

    control.mover_trayectoria(poses, poses_izq)

    control.ejecutor.shutdown()
    control.hilo.join()
    control.destroy_node()
    rclpy.shutdown()
