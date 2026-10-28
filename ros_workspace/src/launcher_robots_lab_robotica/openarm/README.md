# Datos del OpenArm (301, 302)

Contenido importado del repositorio oficial de Enactic para usar los cuerpos OpenArm del
laboratorio sin depender de paquetes externos:

- `assets/robot/openarm_v2.0/`: descripción del robot (xacros, mallas, presets, límites).
  Versiones de partida: `openarm_description` 22800fe (2026-10-28).
- `assets/end_effector/pinch_gripper/`: pinza de cada brazo.
- `assets/sensor/zed/`: envoltorio del xacro de sensor ZED (lo incluye `openarm_v20_sensor.xacro`
  aunque el preset no lleve sensor; necesario para que el xacro procese).
- `moveit/`: SRDF, cinemática, límites y controladores de MoveIt de
  `openarm_bimanual_moveit_config` (8f23aea, 2026-10-21), usados como plantilla por
  `launcher_robots_lab_robotica/lanzador_openarm.py`.

Los archivos se reescribieron para referenciar `$(find launcher_robots_lab_robotica)/openarm`
y las mallas como `package://launcher_robots_lab_robotica/openarm/assets/...`.

No hay brazos OpenArm v1.0 (no se usan).

## Robot real

Con `use_fake_hardware:=false` el xacro usa el plugin `openarm_hardware/OpenArmHW` de
Enactic (no está en el espacio de trabajo). Para implementarlo:

1. Clonar `enactic/openarm_can` (librería C++ CMake; necesita `libcli11-dev` para sus
   herramientas) y `enactic/openarm_ros2` (solo el paquete `openarm_hardware`), y compilarlos.
2. En `robots.py` los buses CAN ya están configurados (301: `can0`/`can1`; 302: `can2`/`can3`).
3. Crear los `launch/robot_301.launch.py` y `launch/robot_302.launch.py`
   (`generar_lanzamiento_openarm("301", simulacion=False)`).
