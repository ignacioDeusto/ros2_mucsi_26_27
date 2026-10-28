#!/usr/bin/python3
"""Control del brazo UR3e del laboratorio con MoveIt 2.

El identificador del robot (ID_ROBOT abajo) selecciona la escena:
  - ""          -> UR3e de la simulación individual (./launcher.sh sim 203)
  - "203" ...   -> un UR3e de una escena doble (./launcher.sh sim 203-204, 205-206)

En las escenas dobles los nombres llevan el prefijo "<id>_": articulaciones
("203_elbow_joint"), efector final ("203_wrist_3_link") y acción de la pinza
("robot_203/rg2_action_server").
"""

import copy
import threading
import time
from math import pi
from typing import List

import rclpy
from control_msgs.action import GripperCommand
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

# Articulaciones y efector final del UR3e, sin prefijo
ARTICULACIONES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
EFECTOR_FINAL = "wrist_3_link"  # Brida del UR, donde se monta la RG2
# Acción de la pinza RG2: command.position es la anchura entre los dedos en mm; el
# nodo de la pinza hace la conversión interna que necesite
ACCION_PINZA = "rg2_action_server"
ANCHURA_MAXIMA = 100.0          # Anchura máxima de la pinza (mm)
# Pose inicial del UR3e (radianes)
POSICION_INICIAL = [0, -pi / 2, -pi / 2, -pi / 2, pi / 2, 0]
DESPLAZAMIENTO = 0.1            # Desplazamiento entre poses del ejemplo (m)
FACTOR_ESCALA = 0.1             # Velocidad y aceleración, fracción de las máximas


class ControlUR(Node):
    """Control del UR3e con las acciones y servicios de move_group (MoveIt 2)."""

    def __init__(self, id_robot: str = "") -> None:
        """:param id_robot: "" para el UR individual, "203"-"206" en las escenas dobles."""
        super().__init__("control_ur")
        self.id_robot = id_robot
        if id_robot:
            self.group_name = f"robot_{id_robot}"
            self.efector_final = f"{id_robot}_{EFECTOR_FINAL}"
            self.articulaciones = [f"{id_robot}_{j}" for j in ARTICULACIONES]
            accion_pinza = f"robot_{id_robot}/{ACCION_PINZA}"
        else:
            self.group_name = "robot"
            self.efector_final = EFECTOR_FINAL
            self.articulaciones = list(ARTICULACIONES)
            accion_pinza = ACCION_PINZA

        self.__estado_articulaciones = {}

        self.create_subscription(JointState, "/joint_states", self.__cb_joint_states, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.move_group_client = ActionClient(self, MoveGroup, "move_action")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self.cartesian_client = self.create_client(GetCartesianPath, "compute_cartesian_path")
        self.scene_client = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.gripper_action_client = ActionClient(self, GripperCommand, accion_pinza)

        # Callbacks (tf, /joint_states, ...) en un hilo aparte
        self.ejecutor = MultiThreadedExecutor()
        self.ejecutor.add_node(self)
        threading.Thread(target=self.ejecutor.spin, daemon=True).start()

        self.get_logger().info(f"Robot {self.group_name}: esperando a MoveIt (move_group)...")
        self.move_group_client.wait_for_server()
        self.execute_client.wait_for_server()
        self.cartesian_client.wait_for_service()
        self.scene_client.wait_for_service()
        while not all(j in self.__estado_articulaciones for j in self.articulaciones):
            time.sleep(0.05)

        self.añadir_suelo()

    def __cb_joint_states(self, msg: JointState) -> None:
        """Actualiza el estado de las articulaciones."""
        for nombre, posicion in zip(msg.name, msg.position):
            self.__estado_articulaciones[nombre] = posicion

    @staticmethod
    def __esperar(future):
        """Bloquea hasta que responde una petición y devuelve su resultado."""
        while not future.done():
            time.sleep(0.01)
        return future.result()

    def articulaciones_actuales(self) -> list:
        """Posiciones actuales de las articulaciones (rad), de /joint_states."""
        return [self.__estado_articulaciones[j] for j in self.articulaciones]

    def __ir_a(self, restricciones: Constraints, wait: bool) -> bool:
        """Envía la petición a move_group con los parámetros del laboratorio; con
        wait=False la lanza y devuelve True al aceptarla, sin esperar el resultado."""
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = FACTOR_ESCALA
        goal.request.max_acceleration_scaling_factor = FACTOR_ESCALA
        goal.request.goal_constraints.append(restricciones)
        goal.request.start_state.is_diff = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False

        goal_handle = self.__esperar(self.move_group_client.send_goal_async(goal))
        if not goal_handle.accepted:
            return False
        if not wait:
            return True
        resultado = self.__esperar(goal_handle.get_result_async()).result
        return resultado.error_code.val == MoveItErrorCodes.SUCCESS

    def mover_articulaciones(self, joint_goal: List[float], wait: bool = True) -> bool:
        """Mueve las articulaciones a las posiciones dadas (radianes)."""
        if len(joint_goal) != len(self.articulaciones):
            self.get_logger().error(
                f"Se esperaban {len(self.articulaciones)} articulaciones "
                f"y se han recibido {len(joint_goal)}")
            return False
        restricciones = Constraints()
        for nombre, valor in zip(self.articulaciones, joint_goal):
            restricciones.joint_constraints.append(JointConstraint(
                joint_name=nombre, position=float(valor),
                tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
        return self.__ir_a(restricciones, wait)

    def pose_actual(self) -> Pose:
        """Pose del efector final en "world", la más reciente del tf."""
        transformacion = self.tf_buffer.lookup_transform(
            SISTEMA_REFERENCIA, self.efector_final, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=5.0))
        pose = Pose()
        pose.position.x = transformacion.transform.translation.x
        pose.position.y = transformacion.transform.translation.y
        pose.position.z = transformacion.transform.translation.z
        pose.orientation = transformacion.transform.rotation
        return pose

    def pose_a_stamped(self, pose: Pose) -> PoseStamped:
        """Pose con cabecera en "world", como las espera MoveIt."""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = SISTEMA_REFERENCIA
        pose_stamped.pose = pose
        return pose_stamped

    def mover_a_pose(self, pose_goal: Pose, wait: bool = True) -> bool:
        """Mueve el efector final a la pose: posición a 1 mm y orientación a 0.01."""
        restriccion_posicion = PositionConstraint()
        restriccion_posicion.header.frame_id = SISTEMA_REFERENCIA
        restriccion_posicion.link_name = self.efector_final
        region = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        restriccion_posicion.constraint_region.primitives.append(region)
        restriccion_posicion.constraint_region.primitive_poses.append(pose_goal)
        restriccion_posicion.weight = 1.0

        restriccion_orientacion = OrientationConstraint()
        restriccion_orientacion.header.frame_id = SISTEMA_REFERENCIA
        restriccion_orientacion.link_name = self.efector_final
        restriccion_orientacion.orientation = pose_goal.orientation
        restriccion_orientacion.absolute_x_axis_tolerance = 0.01
        restriccion_orientacion.absolute_y_axis_tolerance = 0.01
        restriccion_orientacion.absolute_z_axis_tolerance = 0.01
        restriccion_orientacion.weight = 1.0

        restricciones = Constraints()
        restricciones.position_constraints.append(restriccion_posicion)
        restricciones.orientation_constraints.append(restriccion_orientacion)
        return self.__ir_a(restricciones, wait)

    def añadir_caja_a_escena_de_planificación(self, pose_caja: Pose, name: str,
                                              tamaño: tuple = (.1, .1, .1)) -> None:
        """Añade una caja colisionable (dimensiones en m) a la escena de planificación."""
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

    def mover_trayectoria(self, poses: List[Pose], wait: bool = True) -> bool:
        """Recorre en línea recta las poses con una trayectoria cartesiana.
        Falla si el efector no puede pasar por todas (fracción < 1)."""
        poses_aux = copy.deepcopy(poses)
        poses_aux.insert(0, self.pose_actual())

        peticion = GetCartesianPath.Request()
        peticion.header.frame_id = SISTEMA_REFERENCIA
        peticion.start_state.is_diff = True
        peticion.group_name = self.group_name
        peticion.link_name = self.efector_final
        peticion.waypoints = poses_aux
        peticion.max_step = 0.01
        peticion.jump_threshold = 0.0
        peticion.avoid_collisions = True
        peticion.max_velocity_scaling_factor = FACTOR_ESCALA
        peticion.max_acceleration_scaling_factor = FACTOR_ESCALA
        respuesta = self.__esperar(self.cartesian_client.call_async(peticion))

        if respuesta.fraction != 1.0:
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = respuesta.solution
        goal_handle = self.__esperar(self.execute_client.send_goal_async(goal))
        if not goal_handle.accepted:
            return False
        if not wait:
            return True
        resultado = self.__esperar(goal_handle.get_result_async()).result
        return resultado.error_code.val == MoveItErrorCodes.SUCCESS

    def añadir_suelo(self) -> None:
        """Caja en el plano del suelo para que MoveIt no planifique por debajo."""
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        pose_suelo.orientation.w = 1.0
        self.añadir_caja_a_escena_de_planificación(pose_suelo, "suelo", (2, 2, .05))

    def mover_pinza(self, anchura_dedos: float, fuerza: float) -> bool:
        """Cierra o abre la pinza: anchura total entre los dedos, en mm.
        Devuelve si la pinza ha llegado a la anchura pedida."""
        if anchura_dedos < 0.0 or anchura_dedos > ANCHURA_MAXIMA:
            self.get_logger().warn(
                f"Anchura {anchura_dedos} fuera de rango [0, {ANCHURA_MAXIMA}] mm")
        self.gripper_action_client.wait_for_server()
        goal = GripperCommand.Goal()
        goal.command.position = anchura_dedos
        goal.command.max_effort = fuerza
        goal_handle = self.__esperar(self.gripper_action_client.send_goal_async(goal))
        if not goal_handle.accepted:
            return False
        result = self.__esperar(goal_handle.get_result_async()).result
        return result.reached_goal


if __name__ == '__main__':
    rclpy.init()

    # Identificador del robot; ver las opciones en la cabecera del fichero
    ID_ROBOT = ""

    control = ControlUR(ID_ROBOT)

    # Articulaciones iniciales
    control.mover_articulaciones(POSICION_INICIAL)
    paso = DESPLAZAMIENTO

    # Movimiento a una pose relativa a la actual
    pose_actual = control.pose_actual()
    pose_actual.position.z -= paso
    control.mover_a_pose(pose_actual)

    # Trayectoria cartesiana por varias poses
    poses = []

    # Pose 1
    pose_actual = control.pose_actual()
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

    control.mover_trayectoria(poses)

    control.ejecutor.shutdown()
    control.destroy_node()
    rclpy.shutdown()
