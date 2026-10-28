"""Lógica de los launch que arrancan dos robots del laboratorio a la vez en una única escena.

Se usa un único controller_manager, un único robot_state_publisher y un único move_group sobre
el URDF combinado doble_ur3e_AAA_BBB.urdf.xacro. Los grupos de MoveIt se llaman robot_<id> y
gripper_<id>; los controladores robot_<id>_scaled_joint_trajectory_controller, etc.
(los nombres de nodo de ROS 2 no pueden empezar por un número, por eso el prefijo "robot_").
"""

import copy
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from launcher_robots_lab_robotica.robots import ROBOTS

PAQUETE = "launcher_robots_lab_robotica"
ARTICULACIONES_UR = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def _controlador(id_robot, nombre):
    return f"robot_{id_robot}_{nombre}"


def _generar_srdf(ids, nombre_robot, plantilla):
    """Replica el SRDF de un robot para cada id, con prefijos en enlaces, articulaciones y grupos."""
    raiz = ET.parse(plantilla).getroot()
    salida = ET.Element("robot", name=nombre_robot)
    for id_robot in ids:
        p = f"{id_robot}_"
        for elemento in raiz:
            nuevo = copy.deepcopy(elemento)
            if elemento.tag == "group":
                nuevo.set("name", f"{elemento.get('name')}_{id_robot}")
                for hijo in nuevo:
                    hijo.set("name", p + hijo.get("name"))
            elif elemento.tag == "group_state":
                nuevo.set("group", f"{elemento.get('group')}_{id_robot}")
                for hijo in nuevo:
                    hijo.set("name", p + hijo.get("name"))
            elif elemento.tag == "end_effector":
                nuevo.set("name", f"{elemento.get('name')}_{id_robot}")
                nuevo.set("parent_link", p + elemento.get("parent_link"))
                nuevo.set("group", f"{elemento.get('group')}_{id_robot}")
                nuevo.set("parent_group", f"{elemento.get('parent_group')}_{id_robot}")
            elif elemento.tag == "disable_collisions":
                nuevo.set("link1", p + elemento.get("link1"))
                nuevo.set("link2", p + elemento.get("link2"))
            else:
                continue
            salida.append(nuevo)

    # Grupos conjuntos: planifican y ejecutan los dos robots (o las dos pinzas) a la vez
    sufijo = "_".join(ids)
    for base in ("robot", "gripper"):
        conjunto = f"{base}s_{sufijo}"
        grupo = ET.SubElement(salida, "group", name=conjunto)
        for id_robot in ids:
            ET.SubElement(grupo, "group", name=f"{base}_{id_robot}")
        estados = {}
        for estado in raiz.findall("group_state"):
            if estado.get("group") == base:
                estados.setdefault(estado.get("name"), []).append(estado)
        for nombre, plantillas in estados.items():
            combinado = ET.SubElement(salida, "group_state", name=nombre, group=conjunto)
            for id_robot in ids:
                for articulacion in plantillas[0]:
                    nueva = copy.deepcopy(articulacion)
                    nueva.set("name", f"{id_robot}_{articulacion.get('name')}")
                    combinado.append(nueva)
    ET.indent(salida)
    return ET.tostring(salida, encoding="unicode")


def _volcar(ruta, datos):
    with open(ruta, "w", encoding="utf-8") as f:
        yaml.safe_dump(datos, f, sort_keys=False, allow_unicode=True)


def _cargar(ruta):
    with open(ruta, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _config_controladores(ids, simulacion):
    gestor = {
        "update_rate": 500,
        "enforce_command_limits": False,
        "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
    }
    parametros = {"controller_manager": {"ros__parameters": gestor}}
    for id_robot in ids:
        p = f"{id_robot}_"
        articulaciones = [p + j for j in ARTICULACIONES_UR]

        trayectoria = _controlador(id_robot, "scaled_joint_trajectory_controller")
        gestor[trayectoria] = {"type": "ur_controllers/ScaledJointTrajectoryController"}
        restricciones = {"stopped_velocity_tolerance": 0.2, "goal_time": 0.0}
        restricciones.update({j: {"trajectory": 0.2, "goal": 0.1} for j in articulaciones})
        parametros[trayectoria] = {
            "ros__parameters": {
                "joints": articulaciones,
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
                "state_publish_rate": 100.0,
                "action_monitor_rate": 20.0,
                "allow_partial_joints_goal": False,
                "constraints": restricciones,
                "speed_scaling_interface_name": f"{p}speed_scaling/speed_scaling_factor",
            }
        }

        escala = _controlador(id_robot, "speed_scaling_state_broadcaster")
        gestor[escala] = {"type": "ur_controllers/SpeedScalingStateBroadcaster"}
        parametros[escala] = {"ros__parameters": {"state_publish_rate": 100.0, "tf_prefix": p}}

        if not simulacion:
            io = _controlador(id_robot, "io_and_status_controller")
            gestor[io] = {"type": "ur_controllers/GPIOController"}
            parametros[io] = {"ros__parameters": {"tf_prefix": p}}

            fuerza = _controlador(id_robot, "force_torque_sensor_broadcaster")
            gestor[fuerza] = {"type": "force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster"}
            parametros[fuerza] = {
                "ros__parameters": {
                    "sensor_name": f"{p}tcp_fts_sensor",
                    "state_interface_names": [
                        "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z",
                    ],
                    "frame_id": f"{p}tool0_controller",
                    "topic_name": f"robot_{id_robot}_ft_data",
                }
            }
    return parametros


def generar_lanzamiento_doble(ids, simulacion):
    """Devuelve la LaunchDescription de los dos robots `ids` (p. ej. ("203", "204"))."""
    sufijo = "_".join(ids)
    nombre_robot = f"doble_ur3e_{sufijo}"
    config = Path(get_package_share_directory(PAQUETE)) / "config"
    carpeta = Path(tempfile.mkdtemp(prefix=f"lab_{sufijo}_"))

    srdf = carpeta / "robots.srdf"
    srdf.write_text(_generar_srdf(ids, nombre_robot, config / "ur3e_rg2.srdf"), encoding="utf-8")

    cinematica = _cargar(config / "kinematics.yaml")
    _volcar(carpeta / "kinematics.yaml", {f"robot_{i}": cinematica["robot"] for i in ids})

    limites = _cargar(config / "joint_limits.yaml")
    limites["joint_limits"] = {
        f"{i}_{j}": v for i in ids for j, v in limites["joint_limits"].items()
    }
    _volcar(carpeta / "joint_limits.yaml", limites)

    ejecucion = _cargar(config / "moveit_controllers.yaml")
    controladores = {}
    for i in ids:
        nombre = _controlador(i, "scaled_joint_trajectory_controller")
        controladores[nombre] = {
            "action_ns": "follow_joint_trajectory",
            "type": "FollowJointTrajectory",
            "default": True,
            "joints": [f"{i}_{j}" for j in ARTICULACIONES_UR],
        }
    ejecucion["moveit_simple_controller_manager"] = {"controller_names": list(controladores), **controladores}
    _volcar(carpeta / "moveit_controllers.yaml", ejecucion)

    controladores_ros2 = carpeta / "ros2_controllers.yaml"
    _volcar(controladores_ros2, _config_controladores(ids, simulacion))

    moveit_config = (
        MoveItConfigsBuilder(robot_name=nombre_robot, package_name=PAQUETE)
        .robot_description_semantic(srdf)
        .robot_description_kinematics(file_path=carpeta / "kinematics.yaml")
        .joint_limits(file_path=carpeta / "joint_limits.yaml")
        .trajectory_execution(file_path=carpeta / "moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    warehouse = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": str(carpeta / "warehouse_ros.sqlite"),
    }

    descripcion = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(PAQUETE), "urdf", f"{nombre_robot}.urdf.xacro"]
                ),
                " use_mock_hardware:=",
                "true" if simulacion else "false",
            ]
        ),
        value_type=str,
    )

    lanzar_rviz = LaunchConfiguration("launch_rviz")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": descripcion}],
    )

    control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": descripcion}, str(controladores_ros2)],
        output="screen",
    )

    activos = ["joint_state_broadcaster"]
    for i in ids:
        activos.append(_controlador(i, "speed_scaling_state_broadcaster"))
        activos.append(_controlador(i, "scaled_joint_trajectory_controller"))
        if not simulacion:
            activos.append(_controlador(i, "io_and_status_controller"))
            activos.append(_controlador(i, "force_torque_sensor_broadcaster"))

    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--controller-manager", "/controller_manager", "--controller-manager-timeout", "60"] + activos,
        output="screen",
    )

    pinzas = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("rg2_driver"), "launch", "rg2_driver.launch.py"])
            ),
            launch_arguments={
                "robot_ip": ROBOTS[i]["robot_ip"],
                "simulate": "true" if simulacion else "false",
                "prefix": f"{i}_",
                "namespace": f"robot_{i}",
            }.items(),
        )
        for i in ids
    ]

    espera_descripcion = Node(
        package="ur_robot_driver", executable="wait_for_robot_description", output="screen"
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"robot_description": descripcion},
            warehouse,
            {"use_sim_time": False, "publish_robot_description_semantic": True},
        ],
    )

    rviz = Node(
        package="rviz2",
        condition=IfCondition(lanzar_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", str(config / "moveit_ur_doble.rviz")],
        parameters=[
            {"robot_description": descripcion},
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="true", description="¿Lanzar RViz?"),
            robot_state_publisher,
            control,
            spawner,
            *pinzas,
            espera_descripcion,
            RegisterEventHandler(
                OnProcessExit(target_action=espera_descripcion, on_exit=[move_group, rviz])
            ),
        ]
    )
