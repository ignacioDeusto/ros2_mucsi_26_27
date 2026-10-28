"""Lógica de los launch de los cuerpos OpenArm del laboratorio (dos brazos cada uno).

Cada cuerpo OpenArm es un robot con dos brazos de 7 articulaciones y una pinza cada uno.
Al contrario que en las escenas dobles de los UR (dos robots físicos unidos en el lanzador),
aquí el URDF bimanual ya es un solo robot: solo hay que renombrar los prefijos
`openarm_left_`/`openarm_right_` por `<id>_izq_`/`<id>_der_` (p. ej. `301_izq_joint1`).

Todos los datos (xacros, mallas y configuración de MoveIt) están en la carpeta
`openarm/` de este paquete, importados del repositorio openarm_description de Enactic.
Igual que en `lanzador_doble.py`, el SRDF, la cinemática, los límites y los controladores
se generan en un directorio temporal a partir de esa configuración.

Para el robot real falta el plugin de hardware (`openarm_hardware`, de Enactic, que
necesita la librería `openarm_can`): no está en el espacio de trabajo. El xacro ya soporta
`use_fake_hardware:=false` con los buses CAN de `robots.py`, así que bastaría con añadir
esos dos paquetes y crear los `robot_301.launch.py`/`robot_302.launch.py` oportunos.
"""

import copy
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from launcher_robots_lab_robotica.robots import (
    LADOS_OPENARM,
    RAD_ABIERTA_OPENARM,
    ROBOTS_OPENARM,
    SIGNO_ABIERTA_OPENARM,
)

PAQUETE = "launcher_robots_lab_robotica"
ARTICULACIONES_BRAZO = [f"joint{i}" for i in range(1, 8)]


def _ids_brazos(id_cuerpo):
    """Devuelve los identificadores de los dos brazos del cuerpo (p. ej. ("301_izq", "301_der"))."""
    return [f"{id_cuerpo}_{lado}" for lado in LADOS_OPENARM]


def _renombrar_urdf(cadena, id_cuerpo):
    """Sustituye los prefijos openarm_left_/openarm_right_ por los ids del cuerpo."""
    izq, der = _ids_brazos(id_cuerpo)
    # Los nombres de nodo no pueden empezar por un número, así que los componentes de
    # hardware (301_izq_hardware_interface) llevan el prefijo robot_, igual que los
    # controladores. Los nombres de articulaciones y enlaces sí quedan con el id solo.
    reemplazos = [
        ("openarm_left_hardware_interface", f"robot_{izq}_hardware_interface"),
        ("openarm_right_hardware_interface", f"robot_{der}_hardware_interface"),
        ("openarm_left_", f"{izq}_"),
        ("openarm_right_", f"{der}_"),
        ('<param name="arm_prefix">left_</param>', '<param name="arm_prefix">izq_</param>'),
        ('<param name="arm_prefix">right_</param>', '<param name="arm_prefix">der_</param>'),
        ('name="openarm_v20"', f'name="openarm_{id_cuerpo}"'),
    ]
    for viejo, nuevo in reemplazos:
        cadena = cadena.replace(viejo, nuevo)
    return cadena


def _generar_srdf(id_cuerpo, ids, plantilla):
    """Replica el SRDF bimanual de openarm_bimanual_moveit_config con los prefijos del cuerpo.

    Grupos: robot_<id>_izq, robot_<id>_der, gripper_<id>_izq, gripper_<id>_der y los
    combinados robots_<id_cuerpo> / grippers_<id_cuerpo> (sin IK, solo objetivos articulares).
    """
    raiz = ET.parse(plantilla).getroot()
    salida = ET.Element("robot", name=f"openarm_{id_cuerpo}")

    # Grupos del SRDF original con sus nuevos nombres
    grupos = {}
    for lado, id_brazo in zip(LADOS_OPENARM, ids):
        grupos[f"{LADOS_OPENARM[lado]}_arm"] = f"robot_{id_brazo}"
        grupos[f"{LADOS_OPENARM[lado]}_gripper"] = f"gripper_{id_brazo}"
    efectos = {}
    for lado, id_brazo in zip(LADOS_OPENARM, ids):
        efectos[f"{LADOS_OPENARM[lado]}_ee"] = f"eef_{id_brazo}"

    for elemento in raiz:
        nuevo = copy.deepcopy(elemento)
        if elemento.tag == "group":
            nuevo.set("name", grupos[elemento.get("name")])
            for hijo in nuevo:
                hijo.set("name", _prefijar(hijo.get("name"), ids))
        elif elemento.tag == "group_state":
            nuevo.set("group", grupos[elemento.get("group")])
            for hijo in nuevo:
                hijo.set("name", _prefijar(hijo.get("name"), ids))
        elif elemento.tag == "end_effector":
            nuevo.set("name", efectos[elemento.get("name")])
            nuevo.set("parent_link", _prefijar(elemento.get("parent_link"), ids))
            nuevo.set("group", grupos[elemento.get("group")])
            if elemento.get("parent_group"):
                nuevo.set("parent_group", grupos[elemento.get("parent_group")])
        elif elemento.tag == "passive_joint":
            nuevo.set("name", _prefijar(elemento.get("name"), ids))
        elif elemento.tag == "disable_collisions":
            nuevo.set("link1", _prefijar(elemento.get("link1"), ids))
            nuevo.set("link2", _prefijar(elemento.get("link2"), ids))
        else:
            continue
        salida.append(nuevo)

    # Estados nominales de cada pinza (0 = cerrada; RAD_ABIERTA = abierta, con signo por lado)
    for lado, id_brazo in zip(LADOS_OPENARM, ids):
        for nombre, valor in (("abierta", RAD_ABIERTA_OPENARM), ("cerrada", 0.0)):
            estado = ET.SubElement(salida, "group_state", name=nombre, group=f"gripper_{id_brazo}")
            articulacion = ET.SubElement(estado, "joint", name=f"{id_brazo}_finger_joint1")
            articulacion.set("value", f"{SIGNO_ABIERTA_OPENARM[lado] * valor:g}")

    # El SRDF original se generó con los base_link colapsados, así que le faltan los pares
    # del soporte de cada brazo con el cuerpo (unidos por una articulación fija). Sin ellos
    # el estado de reposo sale en colisión y OMPL no puede planificar nada
    for id_brazo in ids:
        ET.SubElement(
            salida, "disable_collisions",
            link1="openarm_body_link0", link2=f"{id_brazo}_base_link", reason="Adjacent")

    # Grupos combinados: planifican los dos brazos (o las dos pinzas) a la vez
    for base in ("robot", "gripper"):
        conjunto = f"{base}s_{id_cuerpo}"
        grupo = ET.SubElement(salida, "group", name=conjunto)
        for id_brazo in ids:
            ET.SubElement(grupo, "group", name=f"{base}_{id_brazo}")

    # Estados combinados copiando el estado del SRDF original (los dos brazos del SRDF
    # original comparten los mismos estados nominales, así que se copian sin repetir)
    vistos = set()
    for estado in raiz.findall("group_state"):
        nombre = estado.get("name")
        if nombre in vistos:
            continue
        vistos.add(nombre)
        combinado = ET.SubElement(salida, "group_state", name=nombre, group=f"robots_{id_cuerpo}")
        for id_brazo in ids:
            for articulacion in estado:
                nueva = copy.deepcopy(articulacion)
                # El estado del SRDF original puede venir del brazo izquierdo o del
                # derecho; se sustituye cualquiera de los dos prefijos por el del brazo
                nueva.set("name", _prefijar_brazo(articulacion.get("name"), id_brazo))
                combinado.append(nueva)
    for nombre, valor in (("abierta", RAD_ABIERTA_OPENARM), ("cerrada", 0.0)):
        combinado = ET.SubElement(salida, "group_state", name=nombre, group=f"grippers_{id_cuerpo}")
        for lado, id_brazo in zip(LADOS_OPENARM, ids):
            articulacion = ET.SubElement(combinado, "joint", name=f"{id_brazo}_finger_joint1")
            articulacion.set("value", f"{SIGNO_ABIERTA_OPENARM[lado] * valor:g}")

    ET.indent(salida)
    return ET.tostring(salida, encoding="unicode")


def _prefijar(nombre, ids):
    """Sustituye openarm_left_/openarm_right_ por los prefijos de los brazos dados."""
    for lado, id_brazo in zip(LADOS_OPENARM, ids):
        nombre = nombre.replace(f"openarm_{LADOS_OPENARM[lado]}_", f"{id_brazo}_")
    return nombre


def _prefijar_brazo(nombre, id_brazo):
    """Sustituye cualquiera de los prefijos openarm_left_/openarm_right_ por el del brazo dado."""
    return (nombre.replace("openarm_left_", f"{id_brazo}_")
                  .replace("openarm_right_", f"{id_brazo}_"))


def _volcar(ruta, datos):
    with open(ruta, "w", encoding="utf-8") as f:
        yaml.safe_dump(datos, f, sort_keys=False, allow_unicode=True)


def _cargar(ruta):
    with open(ruta, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _config_cinematica(ids, plantilla):
    """Cinemática KDL para cada brazo (claves left_arm/right_arm del SRDF original)."""
    cinematica = _cargar(plantilla)
    salida = {}
    for lado, id_brazo in zip(LADOS_OPENARM, ids):
        salida[f"robot_{id_brazo}"] = cinematica[f"{LADOS_OPENARM[lado]}_arm"]
    return salida


def _config_limites(ids, plantilla):
    """Límites articulares con los prefijos del cuerpo."""
    limites = _cargar(plantilla)
    limites["joint_limits"] = {
        _prefijar(nombre, ids): valor for nombre, valor in limites["joint_limits"].items()
    }
    return limites


def _config_controladores(ids):
    """Genera el diccionario de ros2_controllers.yaml para los dos brazos del cuerpo."""
    gestor = {
        "update_rate": 500,
        "enforce_command_limits": False,
        "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
    }
    parametros = {"controller_manager": {"ros__parameters": gestor}}
    for id_brazo in ids:
        trayectoria = f"robot_{id_brazo}_joint_trajectory_controller"
        gestor[trayectoria] = {"type": "joint_trajectory_controller/JointTrajectoryController"}
        articulaciones = [f"{id_brazo}_{j}" for j in ARTICULACIONES_BRAZO]
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
            }
        }

        pinza = f"robot_{id_brazo}_gripper_controller"
        # En Jazzy el controlador de pinza vive en position_controllers/effort_controllers
        gestor[pinza] = {"type": "position_controllers/GripperActionController"}
        parametros[pinza] = {
            "ros__parameters": {
                "joint": f"{id_brazo}_finger_joint1",
                "command_interfaces": ["position"],
                "allow_stalling": True,
            }
        }
    return parametros


def _config_moveit(ids):
    """Genera el diccionario de moveit_controllers.yaml (solo brazos; las pinzas se manejan
    con la acción de su nodo, igual que la RG2 en los UR)."""
    controladores = {}
    for id_brazo in ids:
        nombre = f"robot_{id_brazo}_joint_trajectory_controller"
        controladores[nombre] = {
            "action_ns": "follow_joint_trajectory",
            "type": "FollowJointTrajectory",
            "default": True,
            "joints": [f"{id_brazo}_{j}" for j in ARTICULACIONES_BRAZO],
        }
    return {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 1.2,
            "allowed_goal_duration_margin": 0.5,
            "allowed_start_tolerance": 0.01,
            "execution_duration_monitoring": False,
        },
        "moveit_simple_controller_manager": {
            "controller_names": list(controladores),
            **controladores,
        },
    }


def generar_lanzamiento_openarm(id_cuerpo, simulacion):
    """Devuelve la LaunchDescription del cuerpo OpenArm `id_cuerpo` ("301", "302").

    Si `simulacion` es True se usa el hardware simulado (mock) de ros2_control y las pinzas
    se mueven por el controlador; si no, se usaría openarm_hardware sobre los buses CAN
    configurados en robots.py (paquete no compilado todavía: solo simulación).
    """
    cfg = ROBOTS_OPENARM[id_cuerpo]
    ids = _ids_brazos(id_cuerpo)
    compartido = Path(get_package_share_directory(PAQUETE))
    datos_openarm = compartido / "openarm"
    openarm_moveit = datos_openarm / "moveit"
    carpeta = Path(tempfile.mkdtemp(prefix=f"openarm_{id_cuerpo}_"))

    procesado = xacro.process_file(
        str(datos_openarm / "assets" / "robot" / "openarm_v2.0" / "urdf" / "openarm_v20.urdf.xacro"),
        mappings={
            "robot_preset": "default_bimanual",
            "emit_grasp_frame": "true",
            "use_fake_hardware": "true" if simulacion else "false",
            "left_can_interface": cfg["left_can_interface"],
            "right_can_interface": cfg["right_can_interface"],
            "can_fd": cfg["can_fd"],
        },
    )
    descripcion = _renombrar_urdf(procesado.toprettyxml(indent="  "), id_cuerpo)

    srdf = carpeta / "openarm.srdf"
    srdf.write_text(_generar_srdf(id_cuerpo, ids, openarm_moveit / "openarm_bimanual.srdf"), encoding="utf-8")

    _volcar(carpeta / "kinematics.yaml", _config_cinematica(ids, openarm_moveit / "kinematics.yaml"))
    _volcar(carpeta / "joint_limits.yaml", _config_limites(ids, openarm_moveit / "joint_limits.yaml"))
    _volcar(carpeta / "moveit_controllers.yaml", _config_moveit(ids))
    controladores_ros2 = carpeta / "ros2_controllers.yaml"
    _volcar(controladores_ros2, _config_controladores(ids))

    moveit_config = (
        MoveItConfigsBuilder(robot_name=f"openarm_{id_cuerpo}", package_name=PAQUETE)
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
    for id_brazo in ids:
        activos += [
            f"robot_{id_brazo}_joint_trajectory_controller",
            f"robot_{id_brazo}_gripper_controller",
        ]

    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--controller-manager", "/controller_manager", "--controller-manager-timeout", "60"] + activos,
        output="screen",
    )

    # Un nodo puente por brazo: la acción /robot_<id_brazo>/gripper_action recibe la
    # anchura en mm (igual que la RG2) y la traduce al ángulo de finger_joint1
    pinzas = [
        Node(
            package=PAQUETE,
            executable="pinza_openarm_node",
            namespace=f"robot_{id_brazo}",
            output="screen",
            parameters=[{"id_brazo": id_brazo, "lado": lado}],
        )
        for lado, id_brazo in zip(LADOS_OPENARM, ids)
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
        arguments=["-d", str(compartido / "config" / "moveit.rviz")],
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
