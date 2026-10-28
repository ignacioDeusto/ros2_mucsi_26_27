#!/usr/bin/python3
"""Control del robot del laboratorio con MoveIt 2 (mismo código para UR3e y OpenArm).

El identificador del robot (ID_ROBOT abajo) selecciona el tipo:
  - ""          -> el UR3e de la simulación individual (./launcher.sh sim 203)
  - "203" ...   -> un UR3e de una escena doble (./launcher.sh sim 203-204, 205-206)
  - "301_izq" / "301_der"   -> brazo izquierdo/derecho del cuerpo OpenArm 301
  - "302_izq" / "302_der"   -> brazo izquierdo/derecho del cuerpo OpenArm 302
                   (./launcher.sh sim 301, ./launcher.sh sim 302)

Las diferencias entre robots (número de articulaciones, efector final, acción de la
pinza...) están en la tabla TIPOS_ROBOT; el resto del código no cambia.
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

SISTEMA_REFERENCIA = "world"  # En ROS 2 el sistema de referencia raíz del robot es "world"

# Tipos de robot del laboratorio
UR = "UR"
OPENARM = "OPENARM"

# Brazos OpenArm (cada cuerpo lleva dos brazos que comparten el equipo)
BRAZOS_OPENARM = {"301_izq", "301_der", "302_izq", "302_der"}

# Articulaciones y efector final de cada tipo de robot, sin prefijo
ARTICULACIONES = {
    UR: ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
    OPENARM: [f"joint{i}" for i in range(1, 8)],
}
EFECTOR_FINAL = {
    UR: "wrist_3_link",      # Brida del UR, donde se monta la RG2
    OPENARM: "grasp_frame",  # Punto de agarre entre los dedos de la pinza del OpenArm
}
# Acción de la pinza. En los dos casos command.position es la anchura total entre los
# dedos en milímetros; el nodo de cada pinza hace la conversión interna que toque
ACCION_PINZA = {
    UR: "rg2_action_server",
    OPENARM: "gripper_action",
}
# Anchura máxima de cada pinza (mm)
ANCHURA_MAXIMA = {UR: 100.0, OPENARM: 146.2}
# Pose inicial de cada tipo de robot (articulaciones en radianes). En el OpenArm la
# posición de cero deja el brazo totalmente extendido hacia abajo (singular, no puede
# bajar más), así que la pose inicial es la del SRDF "hands_up" (codo flexionado)
POSICION_INICIAL = {
    UR: [0, -pi / 2, -pi / 2, -pi / 2, pi / 2, 0],
    OPENARM: [0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0],
}
# Desplazamiento de las trayectorias cartesianas del ejemplo (los OpenArm son más
# pequeños que los UR3e; se usa un paso menor)
DESPLAZAMIENTO = {UR: 0.1, OPENARM: 0.05}

FACTOR_ESCALA = 0.1  # Velocidad y aceleración máximas (fracción de las máximas del robot)


def tipo_de_robot(id_robot: str) -> str:
    """Devuelve el tipo de robot a partir del identificador del brazo."""
    return OPENARM if id_robot in BRAZOS_OPENARM else UR


class ControlRobot(Node):
    """Control del robot con MoveIt 2 a través de sus acciones y servicios (move_group)."""

    def __init__(self, id_robot: str = "") -> None:
        """id_robot: "" para un UR individual, "203"-"206" en las escenas dobles de UR, o
        un brazo OpenArm ("301_izq", "301_der", "302_izq", "302_der")."""
        super().__init__("control_robot")
        self.id_robot = id_robot
        self.tipo = tipo_de_robot(id_robot)
        if id_robot:
            self.group_name = f"robot_{id_robot}"
            self.efector_final = f"{id_robot}_{EFECTOR_FINAL[self.tipo]}"
            self.articulaciones = [f"{id_robot}_{j}" for j in ARTICULACIONES[self.tipo]]
            accion_pinza = f"robot_{id_robot}/{ACCION_PINZA[self.tipo]}"
        else:
            self.group_name = "robot"
            self.efector_final = EFECTOR_FINAL[self.tipo]
            self.articulaciones = list(ARTICULACIONES[self.tipo])
            accion_pinza = ACCION_PINZA[self.tipo]

        self.__estado_articulaciones = {}

        self.create_subscription(JointState, "/joint_states", self.__cb_joint_states, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.move_group_client = ActionClient(self, MoveGroup, "move_action")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self.cartesian_client = self.create_client(GetCartesianPath, "compute_cartesian_path")
        self.scene_client = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.gripper_action_client = ActionClient(self, GripperCommand, accion_pinza)

        # Se atienden los callbacks (joint_states, tf, ...) en un hilo aparte
        self.ejecutor = MultiThreadedExecutor()
        self.ejecutor.add_node(self)
        threading.Thread(target=self.ejecutor.spin, daemon=True).start()

        self.get_logger().info(f"Robot {self.tipo} ({self.group_name}): esperando a MoveIt (move_group)...")
        self.move_group_client.wait_for_server()
        self.execute_client.wait_for_server()
        self.cartesian_client.wait_for_service()
        self.scene_client.wait_for_service()
        while not all(j in self.__estado_articulaciones for j in self.articulaciones):
            time.sleep(0.05)

        self.añadir_suelo()

    def __cb_joint_states(self, msg: JointState) -> None:
        for nombre, posicion in zip(msg.name, msg.position):
            self.__estado_articulaciones[nombre] = posicion

    @staticmethod
    def __esperar(future):
        while not future.done():
            time.sleep(0.01)
        return future.result()

    def articulaciones_actuales(self) -> list:
        return [self.__estado_articulaciones[j] for j in self.articulaciones]

    def posicion_inicial(self) -> List[float]:
        """Pose inicial del tipo de robot (los valores de POSICION_INICIAL con prefijo)."""
        return list(POSICION_INICIAL[self.tipo])

    def __ir_a(self, restricciones: Constraints, wait: bool) -> bool:
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
        if len(joint_goal) != len(self.articulaciones):
            self.get_logger().error(
                f"Se esperaban {len(self.articulaciones)} articulaciones "
                f"({self.tipo}) y se han recibido {len(joint_goal)}")
            return False
        restricciones = Constraints()
        for nombre, valor in zip(self.articulaciones, joint_goal):
            restricciones.joint_constraints.append(JointConstraint(
                joint_name=nombre, position=float(valor),
                tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
        return self.__ir_a(restricciones, wait)

    def pose_actual(self) -> Pose:
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
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = SISTEMA_REFERENCIA
        pose_stamped.pose = pose
        return pose_stamped

    def mover_a_pose(self, pose_goal: Pose, wait: bool = True) -> bool:
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
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        pose_suelo.orientation.w = 1.0
        self.añadir_caja_a_escena_de_planificación(pose_suelo, "suelo", (2, 2, .05))

    def mover_pinza(self, anchura_dedos: float, fuerza: float) -> bool:
        """Cierra o abre la pinza: anchura total entre los dedos en milímetros."""
        if anchura_dedos < 0.0 or anchura_dedos > ANCHURA_MAXIMA[self.tipo]:
            self.get_logger().warn(
                f"Anchura {anchura_dedos} fuera de rango [0, {ANCHURA_MAXIMA[self.tipo]}] mm")
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

    # Identificador del robot a controlar; ver la cabecera del fichero
    ID_ROBOT = ""

    # Crear el objeto de tipo robot
    control = ControlRobot(ID_ROBOT)

    # Mover el robot a articulaciones iniciales (cada tipo tiene su pose inicial)
    control.mover_articulaciones(control.posicion_inicial())

    # Desplazamiento de las trayectorias del ejemplo según el tamaño del robot
    paso = DESPLAZAMIENTO[control.tipo]

    # Mover el robot a una pose
    pose_actual = control.pose_actual()
    pose_actual.position.z -= paso
    control.mover_a_pose(pose_actual)

    # Mover el efector final del robot en línea recta a través de varias poses
    poses = []  # Lista de poses que va a recorrer

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
