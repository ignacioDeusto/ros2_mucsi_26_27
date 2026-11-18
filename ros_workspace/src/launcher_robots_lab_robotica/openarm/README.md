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

> **Estado:** preparado y verificado sin hardware (compila, el URDF usa el plugin real y el
> plugin se carga y solo falla al abrir el bus CAN). **Sin probar con el robot físico.**

`robot_301.launch.py` y `robot_302.launch.py` llaman a
`generar_lanzamiento_openarm("<id>", simulacion=False)`, que procesa el xacro con
`use_fake_hardware:=false`: cada brazo es un componente `openarm_hardware/OpenArmHW` sobre su
bus CAN-FD (de `robots.py`), con las ganancias kp/kd de
`assets/robot/openarm_v2.0/config/arm/control/control_gains.yaml`. El resto (MoveIt,
controladores, nodos de pinza, grupos `robot_301_izq`...) es idéntico a la simulación.

### Paquetes de Enactic

Están copiados en el espacio de trabajo (el commit exacto está en el `ORIGEN.md` de cada uno):

| Copia | Origen | Qué se usa |
|---|---|---|
| `ros_workspace/src/openarm_can` | `enactic/openarm_can` 26b266a (v1.4.1) | Librería SocketCAN de los motores Damiao y herramientas `openarm-can-*` |
| `ros_workspace/src/openarm_ros2` | `enactic/openarm_ros2` 8f23aea | Solo `openarm_hardware`; el resto lleva `COLCON_IGNORE` |

Se han copiado en lugar de añadirlos como submódulos git por varios motivos:

- el repositorio se clona sin `--recursive` y el launcher no inicializa submódulos;
- los `COLCON_IGNORE` dejarían el submódulo con cambios locales;
- `openarm_hardware` lleva un parche local: usa los nombres de articulación del URDF
  (`301_izq_joint1`) en lugar de construirlos como `openarm_izq_joint1`;
- así se hizo también con `openarm_description`.

Para actualizarlos hay que sustituir la carpeta por un commit nuevo, volver a aplicar el parche
(descrito en `openarm_ros2/ORIGEN.md`) y actualizar los `ORIGEN.md`.

### Requisitos

- **PC Linux nativo** (no WSL, ni Windows, ni macOS: no tienen SocketCAN) con los adaptadores
  USB-CAN de los brazos conectados. Ubuntu ya trae los módulos `can`, `can_raw` y `gs_usb`.
- Entorno `local` o `local_gpu` (red del anfitrión, contenedor `privileged`). El escritorio
  `desktop` no ve las interfaces CAN del anfitrión.
- Imagen reconstruida con `libcli11-dev`, `can-utils` e `iproute2` (están en los Dockerfiles).
  `./launcher.sh local` reconstruye la imagen si ha cambiado el Dockerfile.

### Pasos

1. **Arrancar el entorno y compilar** (la primera vez, o tras actualizar el repositorio):

   ```bash
   ./launcher.sh local          # o local_gpu
   ./launcher.sh compilar       # compila openarm_can, openarm_hardware y el resto
   ```

2. **Conectar los adaptadores y alimentar los brazos.** Comprueba los nombres de las interfaces
   con `ip -brief link show type can`. El orden `can0`, `can1`... depende del orden de conexión
   y tiene que coincidir con `robots.py`: 301 usa `can1` (izq) y `can0` (der), y 302 usa `can2`
   (izq) y `can3` (der).

3. **Levantar los buses CAN** en el anfitrión (usa `sudo`):

   ```bash
   ./launcher.sh can 301                                   # CAN-FD, 1000000 / 5000000
   ./launcher.sh can 301 --bitrate 1000000 --dbitrate 5000000
   ./launcher.sh can 301 --dry-run                         # solo muestra los comandos
   ```

   Puedes comprobar que hay tráfico con `candump can0` (dentro del contenedor o en el anfitrión)
   y que los motores responden con `openarm-can-cli -i can0 discover` (dentro del contenedor,
   tras activar el espacio de trabajo).

4. **Lanzar el robot**, con espacio libre alrededor de los brazos: **al activarse, el plugin
   lleva todas las articulaciones a la posición cero.**

   ```bash
   ./launcher.sh robot 301
   # o, dentro del contenedor: ros2 launch launcher_robots_lab_robotica robot_301.launch.py
   ```

   En el registro deben aparecer, para cada brazo, `Loaded hardware 'robot_301_izq_hardware_interface'
   from plugin 'openarm_hardware/OpenArmHW'`, `Using joint names from the URDF` y
   `Configured and activated` para los cinco controladores.

5. **Mover un brazo** con `ControlRobot` (de `ejemplos/control_robot.py`, el mismo código que
   en simulación). Conviene probarlo antes con `./launcher.sh sim 301`. Un ejemplo mínimo que
   gira 0,1 rad la primera articulación del brazo izquierdo, la devuelve a su sitio y abre y
   cierra la pinza:

   ```python
   import rclpy
   from control_robot import ControlRobot  # copia ejemplos/control_robot.py junto a este script

   rclpy.init()
   control = ControlRobot("301_izq")        # brazo izquierdo del cuerpo 301

   inicio = control.articulaciones_actuales()   # 7 valores en radianes
   objetivo = list(inicio)
   objetivo[0] += 0.1                           # joint1: +0,1 rad (unos 6°)
   control.mover_articulaciones(objetivo)
   control.mover_articulaciones(inicio)

   control.mover_pinza(60.0, 10.0)              # anchura entre dedos en mm (0-146)
   control.mover_pinza(0.0, 10.0)

   control.ejecutor.shutdown()
   control.destroy_node()
   rclpy.shutdown()
   ```

   `ejemplos/` no está montado en el contenedor: copia `control_robot.py` y el script dentro
   de `ros_workspace/` y ejecútalos con `python3`.

6. **Parar** con `Ctrl+C` en el terminal del lanzamiento. Al desactivarse, el plugin deshabilita
   los motores; sujeta los brazos si pueden caer por gravedad.

### Valores pendientes de confirmar con Enactic

- **Bitrates**: 1 Mbit/s de arbitraje y 5 Mbit/s de datos (CAN-FD) son los valores por defecto
  de `openarm-can-configure-socketcan-4-arms`, y los motores tienen que estar configurados
  igual. `openarm-can-cli ... can_configure` fija además los puntos de muestreo (sample point),
  que `levantar_can.sh` no fija.
- **Ganancias kp/kd** (`control_gains.yaml`, iguales a las de `openarm_hardware`): 70/2,75,
  70/2,5, 70/2,0, 60/2,0, 10/0,7, 10/0,6 y 10/0,5 para joint1-joint7. Las de la pinza son las
  internas del plugin (kp 5, kd 0,1).
