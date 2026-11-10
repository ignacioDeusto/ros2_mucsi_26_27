# ur3e_kinematics — Práctica 1

Extracción de la posición, velocidad y aceleración de las articulaciones del UR3e
mientras ejecuta tres trayectorias (Joint, Lineal y Circular).

## Paquetes

| Paquete | Tipo | Contenido |
|---|---|---|
| `ur3e_kinematics` | ament_python | Nodos `trayectorias` y `lector_articulaciones` |
| `ur3e_kinematics_interfaces` | ament_cmake | Mensaje personalizado `EstadoCinematico.msg` |

El mensaje va en un paquete aparte porque los `.msg` se generan con CMake y un
paquete `ament_python` no puede generarlos.

## Nodos

- **`trayectorias`**: mueve el robot con MoveIt al 10 % de su velocidad y aceleración
  máximas y ejecuta, en orden:
  - `joint`: movimiento articular de ida y vuelta.
  - `lineal`: cuadrado vertical de 10 cm.
  - `circular`: circunferencia horizontal de 5 cm de radio.

  Antes de cada trayectoria publica su nombre en `/trayectoria_actual`.
- **`lector_articulaciones`**: se suscribe a `/joint_states` y calcula la velocidad y
  la aceleración por derivación numérica, con una muestra cada 50 ms. Solo guarda
  datos mientras hay una trayectoria en curso y el robot se está moviendo. Publica
  cada muestra en `/estado_cinematico` y, al terminar cada trayectoria, la guarda
  en un CSV.

## Compilar

Dentro del contenedor:

```bash
cd ~/ros_workspace
colcon build --packages-select ur3e_kinematics_interfaces ur3e_kinematics
source install/setup.bash
```

## Ejecutar

Cada comando en una terminal distinta, en este orden:

```bash
# 1. Simulación del robot
ros2 launch launcher_robots_lab_robotica sim_203.launch.py

# 2. Lector (dejarlo en marcha antes de lanzar las trayectorias)
ros2 run ur3e_kinematics lector_articulaciones

# 3. Trayectorias (termina solo al acabar las tres)
ros2 run ur3e_kinematics trayectorias
```

## Resultados

**CSV**: se guarda un archivo por trayectoria en `~/ros_workspace/datos_cinematicos/`
(`joint.csv`, `lineal.csv` y `circular.csv`). Esa carpeta es la compartida con el
host. Cada fila es una muestra con `tiempo_s` y, por cada articulación, su
posición (rad), velocidad (rad/s) y aceleración (rad/s²). Cada ejecución
sobrescribe los archivos anteriores.

**Mensaje personalizado**: mientras se ejecutan las trayectorias:

```bash
ros2 topic echo /estado_cinematico
ros2 interface show ur3e_kinematics_interfaces/msg/EstadoCinematico
```
