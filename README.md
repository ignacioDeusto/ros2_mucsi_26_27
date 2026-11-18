# Robótica y Automatización Inteligente - Entorno de desarrollo

Este repositorio proporciona un entorno de trabajo basado en ROS 2 Jazzy (Robot Operating System) para la asignatura de Robótica y Automatización Inteligente del Máster de Computación y Sistemas Inteligentes en la Universidad de Deusto. El entorno está empaquetado en contenedores Docker para asegurar la portabilidad y facilidad de uso.

## Índice

1. [Requisitos](#1-requisitos)
2. [Perfiles de Docker](#2-perfiles-de-docker)
3. [Robots disponibles](#3-robots-disponibles)
4. [Método A: scripts `launcher`](#4-método-a-scripts-launcher-recomendado)
5. [Método B: manual](#5-método-b-manual)
6. [Problemas frecuentes](#6-problemas-frecuentes)
7. [Notas de desarrollo](#7-notas-de-desarrollo)
8. [Diferencias respecto a ROS 1 (25_26)](#8-diferencias-respecto-a-la-versión-ros-1-25_26)
9. [Arquitectura](#9-arquitectura)
10. [Estado de verificación](#10-estado-de-verificación)
11. [Anexos: Docker, ROS 2 y colcon](#anexos)

## Resumen: dos formas de trabajar

| | Método A: scripts | Método B: manual |
|---|---|---|
| Qué se usa | `launcher.sh` (Linux/macOS/WSL) o `launcher.bat` (Windows) | `docker compose` y, opcionalmente, VS Code Dev Containers |
| Para quién | Quien quiera empezar rápido | Quien quiera controlar cada paso |
| Ejemplo | `./launcher.sh desktop` y `./launcher.sh sim 203` | `docker compose up -d desktop` y `ros2 launch launcher_robots_lab_robotica sim_203.launch.py` |

Ambos métodos usan las mismas imágenes y el mismo espacio de trabajo (`ros_workspace/`), así que se pueden combinar.

## 1. Requisitos

### En el equipo local (fuera del contenedor)

1. [Docker](https://www.docker.com/get-started) ([instrucciones de instalación](https://docs.docker.com/get-docker/)). En Windows y macOS, Docker Desktop. También se puede instalar con `./launcher.sh instalar-docker`.
2. [Git](https://git-scm.com/) para clonar el repositorio.
3. [Visual Studio Code](https://code.visualstudio.com/) (opcional con el método A, recomendado con el B).
4. Extensiones de VS Code **instaladas en el anfitrión**:
   - [Remote Development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) (`ms-vscode-remote.vscode-remote-extensionpack`): incluye *Dev Containers*, que permite abrir o conectarse a un contenedor desde VS Code.
   - [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) (`ms-azuretools.vscode-docker`): permite ver, arrancar y parar contenedores desde VS Code.
5. Opcional, solo para el perfil `local_gpu` (Linux con X11 y GPU NVIDIA): [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) y los drivers de NVIDIA.

### Dentro del contenedor

Estas extensiones se instalan **en el contenedor** (no en el host (windows, wsl, linux o mac)):

- [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python) (`ms-python.python`).
- [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros) (`ms-iot.vscode-ros`).

Si abres el repositorio con *Dev Containers: Reopen in Container* (usa `.devcontainer/devcontainer.json`), se instalan automáticamente. Si te conectas a un contenedor ya en marcha con *Attach to Running Container*, instálalas una vez desde el panel de extensiones con **Install in Container**.

### Sistema

- **Linux / macOS / Windows**: Docker y VS Code funcionan en todos ellos, aunque algunas características varían (ver los perfiles).
- **GPU** (opcional): solo Linux con X11 y una GPU NVIDIA compatible.

## 2. Perfiles de Docker

El archivo `docker-compose.yml` define tres servicios. Los nombres de los servicios coinciden con los de los perfiles y con los comandos del script `launcher`.

| | `desktop` | `local` | `local_gpu` |
|---|---|---|---|
| Perfil de Compose | (por defecto) | `local` | `local_gpu` |
| Dónde funciona | Windows, macOS, Linux; x86 y ARM | Linux con X11 (o WSL) | Linux con X11 (o WSL) |
| Cómo se ve la interfaz gráfica | En el navegador (noVNC): http://localhost:6081, contraseña `laboratorio` | En la pantalla del equipo | En la pantalla del equipo |
| GPU NVIDIA | No | No | Sí |
| Red | Puertos 6081 (noVNC) y 5901 (VNC) | `network_mode: host`, `privileged` | `network_mode: host`, `privileged` |
| Límites de recursos | 8 GB y 4 CPUs (`LAB_MEM`, `LAB_CPUS`) | Sin límites | Sin límites |
| Imagen | `mucsi_26_27_desktop_novnc` | `local` | `local_gpu` |
| Uso previsto | Uso personal y simulación | PCs del laboratorio | PCs del laboratorio con GPU |

**¿Cuál uso?**

- En un PC MAC, `desktop`, siempre.
- En un PC con WSL o Linux, sin NVIDIA: `local`. Con NVIDIA y `nvidia-container-toolkit`: `local_gpu`.
- En un PC del laboratorio sin NVIDIA: `local`. Con NVIDIA y `nvidia-container-toolkit`: `local_gpu`.
- Para conectarte al robot real desde Windows o macOS no se publican los puertos del driver (50030-50062); hazlo desde un PC del laboratorio o Linux.
- El OpenArm real necesita SocketCAN (buses CAN del kernel): solo funciona en un **Linux nativo** con `local`/`local_gpu`. WSL, Windows y macOS no sirven; ahí usa la simulación.

Los perfiles `local` y `local_gpu` requieren:

- Sistema anfitrión Linux (o WSL) con el sistema de ventanas X.
- (`local_gpu`) Gráfica NVIDIA con `nvidia-container-toolkit` y sus drivers.

Solo se usa un perfil a la vez. Solo `./ros_workspace` se comparte con el anfitrión (en `/home/laboratorio/ros_workspace`); el usuario del contenedor es `laboratorio` (uid 1000). `ejemplos/` **no** está montado en el contenedor.

## 3. Robots disponibles

### Robots UR3e + pinza RG2 (203-206)

| Robot | Simulación | Robot real | IP | Puertos (reverse / script_sender / trajectory) |
|---|---|---|---|---|
| 203 | `sim_203.launch.py` | `robot_203.launch.py` | 10.172.21.203 | 50030 / 50031 / 50032 |
| 204 | `sim_204.launch.py` | `robot_204.launch.py` | 10.172.21.204 | 50040 / 50041 / 50042 |
| 205 | `sim_205.launch.py` | `robot_205.launch.py` | 10.172.21.205 | 50050 / 50051 / 50052 |
| 206 | `sim_206.launch.py` | `robot_206.launch.py` | 10.172.21.206 | 50060 / 50061 / 50062 |

El puerto del TCP de la herramienta es 54321 para todos. Los datos están en `ros_workspace/src/launcher_robots_lab_robotica/launcher_robots_lab_robotica/robots.py`; un robot nuevo necesita una entrada ahí, su calibración, su xacro y sus archivos de lanzamiento.

### Parejas (dos robots en la misma escena)

| Pareja | Simulación | Robot real |
|---|---|---|
| 203 + 204 | `sim_203_204.launch.py` | `robot_203_204.launch.py` (**sin probar**) |
| 205 + 206 | `sim_205_206.launch.py` | `robot_205_206.launch.py` (**sin probar**) |

Una pareja usa un único `move_group` y un único RViz. Los robots reales de una pareja no incluyen el control del dashboard: hay que liberar los frenos y arrancar el programa *External Control* desde la consola de cada robot.

### OpenArm (301 y 302)

Cada cuerpo OpenArm es un robot con **dos brazos de 7 articulaciones** (izquierdo y derecho) y una pinza cada uno. La simulación usa mock hardware; el robot real usa el plugin `openarm_hardware` de Enactic (sobre la librería `openarm_can`) con un bus CAN-FD por brazo.

| Cuerpo | Brazos (`ControlRobot`) | Simulación | Robot real (**sin probar con el robot físico**) | Buses CAN (izq / der) |
|---|---|---|---|---|
| 301 | `301_izq`, `301_der` | `sim_301.launch.py` (`./launcher.sh sim 301`) | `robot_301.launch.py` (`./launcher.sh can 301` y `./launcher.sh robot 301`) | `can0` / `can1` |
| 302 | `302_izq`, `302_der` | `sim_302.launch.py` (`./launcher.sh sim 302`) | `robot_302.launch.py` (`./launcher.sh can 302` y `./launcher.sh robot 302`) | `can2` / `can3` |

Los buses se configuran en `robots.py` (`ROBOTS_OPENARM`). Antes de lanzar el robot real hay que levantarlos en el anfitrión con `./launcher.sh can <id>` (CAN-FD, 1 Mbit/s y 5 Mbit/s de datos por defecto). **Al activarse, el plugin lleva los brazos a la posición cero**: deja espacio libre alrededor. Los pasos completos están en `ros_workspace/src/launcher_robots_lab_robotica/openarm/README.md`, sección "Robot real".

#### Lanzar el OpenArm real: comandos exactos

Se necesita un **PC Linux nativo** (no WSL, Windows ni macOS) con los adaptadores USB-CAN de los dos brazos conectados y los brazos alimentados. Los ejemplos usan el cuerpo 301; para el 302 cambia `301` por `302` (buses `can2`/`can3`).

**1. Obtener el código** (solo la primera vez; en un PC que ya lo tenga, `git pull`):

```bash
git clone -b dev https://github.com/ignacioDeusto/ros2_mucsi_26_27.git
cd ros2_mucsi_26_27
```

**2. Arrancar el entorno del laboratorio y compilar** (terminal 1, en el anfitrión). `local` reconstruye la imagen si cambió el Dockerfile; con GPU NVIDIA usa `local_gpu`:

```bash
./launcher.sh local
./launcher.sh compilar
```

**3. Comprobar los adaptadores** (anfitrión). Deben aparecer `can0` (brazo izquierdo) y `can1` (brazo derecho); si salen en otro orden, desconéctalos y conéctalos en orden:

```bash
ip -brief link show type can
```

**4. Levantar los buses CAN** (anfitrión, pide la contraseña de `sudo`):

```bash
./launcher.sh can 301
```

Se puede ver antes qué va a hacer con `./launcher.sh can 301 --dry-run`, y cambiar las velocidades con `--bitrate 1000000 --dbitrate 5000000`.

**5. Comprobar que responden los motores sin moverlos** (terminal 2, dentro del contenedor). Deben responder los motores de cada brazo:

```bash
./launcher.sh terminal
openarm-can-cli -i can0 discover
openarm-can-cli -i can1 discover
exit
```

**6. Lanzar el robot** (terminal 1). ⚠️ **Al arrancar, los dos brazos van a la posición cero:** deja espacio libre alrededor y ten a mano el corte de alimentación.

```bash
./launcher.sh robot 301
```

En la salida deben aparecer `Loaded hardware 'robot_301_izq_hardware_interface' from plugin 'openarm_hardware/OpenArmHW'` (y lo mismo para `der`), `Using joint names from the URDF` y `Configured and activated` para los cinco controladores. Se abre RViz.

**7. Primer movimiento** (terminal 2, con el robot lanzado). Copia los ejemplos al espacio de trabajo, ya que `ejemplos/` no está montado en el contenedor:

```bash
cp ejemplos/control_robot.py ejemplos/prueba_openarm.py ros_workspace/
./launcher.sh terminal
cd ~/ros_workspace
python3 prueba_openarm.py 301_izq 0.05
```

El script gira `joint1` del brazo izquierdo 0,05 rad, lo devuelve a su sitio y abre y cierra la pinza a 30 mm. Para el brazo derecho: `python3 prueba_openarm.py 301_der 0.05`. Conviene probar antes lo mismo en simulación (`./launcher.sh sim 301`).

**8. Parar** con `Ctrl+C` en el terminal 1. Los motores se deshabilitan, así que **sujeta los brazos** si pueden caer. Para apagar el entorno: `./launcher.sh parar`.

## 4. Método A: scripts `launcher` (recomendado la primera vez)

El repositorio incluye un script que hace todo lo necesario. Solo se necesita Git y Docker.

| Sistema | Script |
|---|---|
| Windows | `launcher.bat` (desde PowerShell o CMD) |
| Linux / macOS / WSL | `./launcher.sh` |

Los ejemplos usan `launcher.sh`; en Windows sustituir por `launcher.bat` (`.\launcher.bat`).

### Pasos

1. **Clonar el repositorio**: `git clone https://github.com/ignacioDeusto/ros2_mucsi_26_27.git` y entrar en la carpeta.
2. **Instalar Docker** (si no está instalado): `./launcher.sh instalar-docker`.
   - Windows: usa `winget` para instalar Docker Desktop (requiere WSL 2; puede pedir reiniciar).
   - macOS: usa Homebrew; si no lo tienes, indica la descarga manual.
   - Linux: usa el script oficial de Docker y `sudo`.
   - Después hay que abrir Docker Desktop una vez (Windows y macOS) o volver a iniciar sesión (Linux).
3. **Iniciar el entorno**: `./launcher.sh desktop`. La primera vez construye la imagen (varios minutos; bastante más en ARM), compila el espacio de trabajo y abre el escritorio en el navegador (`http://localhost:6081`, contraseña `laboratorio`).
4. **Lanzar una simulación**: `./launcher.sh sim 203`. En el escritorio se abrirá RViz con el robot y MoveIt. Detener con `Ctrl+C`.
5. **Robot real**: `./launcher.sh robot 203` (requiere estar en la red del laboratorio).
6. **Parar**: `./launcher.sh parar` (conserva el contenido del contenedor).

En los PCs del laboratorio o personales con (Linux o WSL con X11) se puede usar el entorno local en lugar del navegador: `./launcher.sh local` o `./launcher.sh local_gpu`.

### Comandos

| Comando | Qué hace |
|---|---|
| `instalar-docker` | Instala Docker si no lo tienes |
| `desktop` | Arranca el entorno de escritorio y lo abre en el navegador (noVNC) |
| `local` | Arranca el entorno del laboratorio en la pantalla (solo Linux con X11) |
| `local_gpu` | Igual que `local`, con GPU NVIDIA |
| `sim <id>` | Lanza la simulación: UR `203`-`206`, parejas `203-204`/`205-206`, u OpenArm `301`/`302` |
| `robot <id>` | Lanza el robot real: UR `203`-`206`, parejas UR (sin probar) u OpenArm `301`/`302` (antes hay que ejecutar `can <id>`) |
| `can <id> [--bitrate N] [--dbitrate N] [--no-fd]` | Levanta en el anfitrión los buses CAN del OpenArm `301`/`302` (solo Linux nativo; no WSL). Por defecto CAN-FD 1000000/5000000. Equivale a `herramientas/levantar_can.sh <id>` |
| `terminal` | Abre una terminal dentro del contenedor |
| `compilar` | Recompila el espacio de trabajo (`colcon build --symlink-install`) |
| `parar` | Para el contenedor (conserva su contenido) |
| `ayuda` | Muestra la ayuda |

`sim` y `robot` ejecutan dentro del contenedor `ros2 launch launcher_robots_lab_robotica {sim|robot}_<id>.launch.py`. `can` se ejecuta en el anfitrión (no en el contenedor), porque los buses CAN son del kernel del anfitrión, y usa `sudo`. Tras modificar paquetes, ejecuta `compilar`.

## 5. Método B: manual

Equivale a lo que hace el script, pero paso a paso. Cada paso indica su equivalente con `launcher`.

### 5.1. Clonar el repositorio

```bash
git clone https://github.com/ignacioDeusto/ros2_mucsi_26_27.git
cd ros2_mucsi_26_27
```

### 5.2. Construir la imagen y arrancar el contenedor

Hay dos opciones (elige una).

**Opción B1: `docker compose`**

Perfil `desktop`:
```bash
docker compose build
docker compose up -d desktop
```
(Equivale a `./launcher.sh desktop`.)

Perfiles del laboratorio (`local` o `local_gpu`), solo Linux con X11:
```bash
xhost +local:
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" DISPLAY="${DISPLAY:-:0}"
docker compose --profile local up -d --build local        # o: --profile local_gpu ... local_gpu
```
(Equivale a `./launcher.sh local` / `local_gpu`.)

> **Importante**
> - `XAUTHORITY` debe estar definida, aunque solo vayas a ejecutar `docker compose build`: si no, Compose interpreta `0` como un volumen sin definir y falla la validación de todo el proyecto.
> - No ejecutes `docker compose up --build` **sin indicar el servicio** con un perfil del laboratorio: `desktop` no tiene perfil, así que también se construiría y arrancaría.
> - En ARM (Apple Silicon...) la primera construcción es más lenta porque `ur_rtde` se compila.

Para parar el contenedor sin perder su contenido: `docker compose stop` (equivale a `./launcher.sh parar`). **No uses `docker compose down`** salvo que quieras eliminar el contenedor.

**Opción B2: VS Code Dev Containers**

Requiere las extensiones del anfitrión (sección 1).

- *Reopen in Container*: abre la carpeta del repositorio en VS Code y ejecuta **Dev Containers: Reopen in Container**. Usa `.devcontainer/devcontainer.json` (servicio `desktop`), reenvía el puerto 6081 e instala en el contenedor las extensiones Python y ROS.
- *Attach to Running Container*: si ya lo has arrancado con B1, ejecuta **Dev Containers: Attach to Running Container** (o usa el panel de la extensión Docker) y abre `/home/laboratorio/ros_workspace`.

<p align="center">
    <img src="pictures/encender_contenedor.gif" alt="lanzar">
</p>
<p align="center">
    <img src="pictures/conectarse_contenedor_vscode.gif" alt="Conectarse">
</p>

Con `desktop`, se recomienda desarrollar y lanzar desde VS Code y usar el escritorio de http://localhost:6081 (contraseña `laboratorio`) para ver la salida gráfica.

### 5.3. Abrir una terminal en el contenedor

- Desde VS Code: nueva terminal (ya está dentro del contenedor).
- Sin VS Code: `docker compose exec -u laboratorio desktop bash` (con perfil: `docker compose --profile local exec -u laboratorio local bash`). Equivale a `./launcher.sh terminal`.

### 5.4. Construir el espacio de trabajo (primera vez)

```bash
cd /home/laboratorio/ros_workspace # Entrar al directorio
```
```bash
sudo apt update # Actualizar paquetes de Linux
```
```bash
rosdep update # Actualizar dependencias de ROS2
```
```bash
rosdep install --from-paths src --ignore-src -r -y # Instalar dependencias de ROS2
```
```bash
colcon build --symlink-install # Compilar paquetes de ROS2
```

(`./launcher.sh compilar` equivale al último comando y se usa para recompilar.)

### 5.5. Activar el espacio de trabajo

```bash
source /opt/ros/jazzy/setup.bash
source /home/laboratorio/ros_workspace/install/setup.bash
```

> **IMPORTANTE:** la activación se debe realizar en **cada nueva terminal** en la que se quiera usar algo del espacio de trabajo. Si siempre se va a utilizar el mismo, se puede incluir en `/home/laboratorio/.bashrc`.

### 5.6. Lanzar un robot

```bash
ros2 launch launcher_robots_lab_robotica sim_206.launch.py
```

Debería aparecer RViz con el robot; si se selecciona el grupo de planificación `robot_206`, se puede planificar y ejecutar trayectorias. Otros ejemplos:

```bash
ros2 launch launcher_robots_lab_robotica sim_205_206.launch.py   # pareja (./launcher.sh sim 203-204)
```
```bash
ros2 launch launcher_robots_lab_robotica robot_206.launch.py      # robot real (./launcher.sh robot 203)
```
```bash
ros2 launch launcher_robots_lab_robotica sim_301.launch.py        # OpenArm 301 (./launcher.sh sim 301)
```
```bash
ros2 launch launcher_robots_lab_robotica robot_301.launch.py      # OpenArm 301 real (antes: ./launcher.sh can 301 en el anfitrión)
```

## 6. Problemas frecuentes

### Docker y permisos

| Síntoma | Solución |
|---|---|
| "Docker no está en ejecución" | Abre Docker Desktop y espera a que indique que está listo. |
| Linux: "permission denied" con docker | `sudo usermod -aG docker $USER` y reinicia la sesión. |
| Permisos al compilar en Linux | El usuario del contenedor tiene uid 1000; si el tuyo es otro, ejecuta `chmod -R a+rwX ros_workspace`. |
| Windows: scripts bloqueados | Usa `launcher.bat`, que ejecuta PowerShell sin cambiar la política del sistema. |
| `launcher.sh` no es ejecutable | `bash launcher.sh desktop` o `chmod +x launcher.sh`. |

### Compose y perfiles

| Síntoma | Solución |
|---|---|
| Se construyen o arrancan dos entornos a la vez | No ejecutes `docker compose up --build` sin indicar el servicio: `desktop` no tiene perfil y se incluye siempre. Usa los comandos del launcher o indica el servicio. |
| Fallo de validación del proyecto (incluso con `docker compose build`) | Con perfiles del laboratorio, define `XAUTHORITY` (p. ej. `export XAUTHORITY=$HOME/.Xauthority`). El launcher lo hace por ti. |
| `--dry-run` modifica imágenes | `docker compose --dry-run` no hace de comprobación de solo lectura: sigue construyendo y etiquetando imágenes. |
| Los perfiles `local` fallan fuera de Linux | Solo funcionan en Linux con X11 (o WSL). En otros sistemas usa `desktop`. |

### Memoria y rendimiento

| Síntoma | Solución |
|---|---|
| El equipo o el contenedor se bloquea (código 137 = falta de memoria) | El contenedor `desktop` está limitado a 8 GB y 4 CPUs. Cámbialo con `LAB_MEM` y `LAB_CPUS` (por ejemplo `LAB_MEM=6g LAB_CPUS=2 ./launcher.sh desktop`; en PowerShell `$env:LAB_MEM="6g"`). En Windows, limita además WSL 2 con `%UserProfile%\.wslconfig` (`[wsl2]`, `memory=10GB`, `processors=6`, `swap=4GB`) y ejecuta `wsl --shutdown`. En Docker Desktop (macOS) ajusta Settings > Resources. |

### Pantalla y entorno gráfico

| Síntoma | Solución |
|---|---|
| Pantalla en blanco en el navegador | Espera unos segundos y recarga; el escritorio tarda en arrancar. |
| Con `local`/`local_gpu` no se abre ninguna ventana | Ejecuta `xhost +local:` y comprueba `DISPLAY` y `XAUTHORITY`. |

### ROS 2

| Síntoma | Solución |
|---|---|
| `ros2: command not found` o no se encuentra el paquete `launcher_robots_lab_robotica` | Activa el espacio de trabajo en esa terminal (`source ~/ros_workspace/install/setup.bash`) tras haberlo compilado. |
| Cambios en paquetes no se aplican | Ejecuta `./launcher.sh compilar` (o `colcon build --symlink-install`) y vuelve a activar. |
| OMPL no encuentra solución con los grupos `robots_203_204` | Usa una tolerancia de ~0.01 rad (0.001 falla) y objetivos en el espacio articular. |
| No existe `ejemplos/` en el contenedor | No está montado; solo `ros_workspace` se comparte. Copia lo necesario dentro de `ros_workspace`. |

### Robots reales

| Síntoma | Solución |
|---|---|
| No conecta con el robot | Comprueba que estás en la red del laboratorio (10.172.21.x) y que en el robot corre el programa *External Control*. |
| Desde Windows/macOS no funciona el robot real | No están publicados los puertos del driver (50030-50062); usa un PC del laboratorio con `local` o `local_gpu`. |
| Las parejas reales (`robot_203_204`, `robot_205_206`) no se comportan como se espera | Están sin probar y no incluyen el control del dashboard: libera los frenos y arranca *External Control* desde la consola de cada robot. |

### OpenArm real (301, 302)

| Síntoma | Solución |
|---|---|
| `./launcher.sh can 301` dice "Estás en WSL y su kernel no incluye SocketCAN" | El kernel de WSL (y el de Docker Desktop) no trae CAN, así que desde WSL, Windows o macOS no se puede mover el OpenArm real. Usa un PC Linux nativo del laboratorio con `local`/`local_gpu`; aquí trabaja con `./launcher.sh sim 301`. |
| "El kernel de este equipo no tiene soporte de CAN" | El Linux no tiene los módulos `can`/`can_raw`. Usa un kernel de distribución estándar (Ubuntu los incluye). |
| "No existe la interfaz can0" | El adaptador USB-CAN de ese brazo no está conectado o no tiene driver (`gs_usb`). Comprueba con `ip -brief link show type can`. Los nombres `can0`, `can1`... dependen del orden de conexión: si no coinciden con `robots.py`, conéctalos en orden o cambia `robots.py`. |
| Al lanzar `robot_301`: `Failed to initialize socket for interface: can0` | Los buses no están levantados (`./launcher.sh can 301`) o el contenedor no ve la red del anfitrión (usa `local`/`local_gpu`, no `desktop`). |
| Al lanzar `robot_301`: `Could not find ... openarm_hardware/OpenArmHW` | El paquete `openarm_hardware` no está compilado. Reconstruye la imagen (`./launcher.sh local` lo hace solo si cambió el Dockerfile) para tener `libcli11-dev` y ejecuta `./launcher.sh compilar`. |
| Los brazos se mueven al lanzar `robot_301` | Es lo esperado: al activarse, `openarm_hardware` lleva todas las articulaciones a la posición cero. |
| `candump can0` no muestra tráfico | Comprueba la alimentación de los motores, el cableado y que todos los nodos usen el mismo bitrate (`./launcher.sh can 301 --bitrate ... --dbitrate ...`). |

## 7. Notas de desarrollo

- Todo lo que se instale, ejecute o modifique dentro del contenedor (fuera de `ros_workspace`) se pierde si el contenedor se elimina. Si solo se para, se conserva.
- El directorio `ros_workspace` se comparte con el anfitrión: los cambios se ven en ambos lados. En el contenedor está en `/home/laboratorio/ros_workspace`.
- No confundir el anfitrión con el contenedor (estarán en instancias separadas de VS Code).
- No ejecutes `docker compose down` salvo que quieras eliminar el contenedor. Para pararlo, `docker compose stop` o `./launcher.sh parar`.
- Si modificas los scripts `launcher.sh`, `launcher.ps1` o `launcher.bat`, cambia los tres a la vez: son implementaciones paralelas.

## 8. Diferencias respecto a la versión ROS 1 (25_26)

- ROS 2 Jazzy sobre Ubuntu 24.04; compilación con `colcon`.
- Se usan el driver oficial de UR (`ur_robot_driver`) y MoveIt 2 instalados desde apt; ya no se incluyen paquetes vendorizados.
- El sistema de referencia raíz del robot es `world` (antes `base_link`).
- Robots reales: `robot_203.launch.py` (203-206). Simulación: `sim_203.launch.py`. Parejas: `sim_203_204`, `sim_205_206` (ver la sección 3).
- La pinza RG2 se maneja con el paquete `rg2_driver` (`simulate:=true` en simulación). Requiere `ur_rtde` (`pip`, ya incluido en la imagen). La articulación `joint0` se ha reorientado respecto a ROS 1 porque `tool0` de ROS 2 está girado respecto al de ROS 1.
- Los ejemplos usan `rclpy` + `move_group` (acciones `move_action`/`execute_trajectory`) en lugar de `moveit_commander`. `ejemplos/control_robot.py` acepta un id de robot opcional (`ControlRobot("203")`) para las escenas dobles de UR, y los ids de brazo OpenArm (`ControlRobot("301_izq")`, `"301_der"`, `"302_izq"`, `"302_der"`); el resto de la API es idéntica para los dos tipos de robot.

## 9. Arquitectura

- `dockerfiles/`: imagen basada en ROS 2 Jazzy + escritorio XFCE con noVNC (puerto 6081, contraseña `laboratorio`), multiarquitectura (x86_64/arm64). `docker-compose.yml` define los perfiles y los límites de recursos (`LAB_MEM`, `LAB_CPUS`).
- `ros_workspace/src/launcher_robots_lab_robotica`: xacros (UR3e + RG2, con `ros2_control`), calibraciones por robot, configuración de MoveIt 2 y los lanzadores. `lanzador.py` genera el lanzamiento de un robot; `lanzador_doble.py` el de una pareja (SRDF, controladores y MoveIt combinados en una carpeta temporal); `lanzador_openarm.py` el de un cuerpo OpenArm (renombra los prefijos `openarm_left_`/`openarm_right_` del URDF y SRDF bimanuales, y genera SRDF, cinemática, límites y controladores). El nodo `pinza_openarm` sirve la acción de la pinza OpenArm en milímetros y publica la articulación mimética `finger_joint2`. Los archivos de `launch/` son envoltorios finos.
- `ros_workspace/src/rg2_driver`: servidor de acción `GripperCommand` de la pinza RG2 (con `prefix` y `namespace` para las escenas dobles).
- `ros_workspace/src/launcher_robots_lab_robotica/openarm/`: datos del OpenArm importados del repositorio original de Enactic (xacros y mallas del v2.0, pinza, envoltorio ZED, plantillas de MoveIt; ver `openarm/README.md`), con las rutas reescritas al paquete.
- `ros_workspace/src/openarm_can` y `ros_workspace/src/openarm_ros2`: copias de los repositorios de Enactic para el OpenArm real (la librería CAN y el plugin `openarm_hardware`; el resto de paquetes de `openarm_ros2` llevan `COLCON_IGNORE`). El commit exacto y los cambios locales están en el `ORIGEN.md` de cada uno.
- `herramientas/levantar_can.sh`: levanta los buses CAN de un cuerpo OpenArm (lo usa `./launcher.sh can <id>`).
- `launcher.sh` / `launcher.ps1` / `launcher.bat`: scripts multiplataforma (instalar Docker, `desktop`, `local`, `local_gpu`, `sim`, `robot`, `can`, `terminal`, `compilar`, `parar`).

<!-- ## 10. Estado de verificación

**Probado** (simulación, Docker Desktop en Windows con WSL2, x86_64): construcción de la imagen y del workspace, `sim_203`..`sim_206`, `sim_203_204`, `sim_205_206` (incluidos los grupos conjuntos en 203+204), ejemplos de MoveIt, escritorio noVNC.

**Sin probar**: robots reales (individuales y por parejas), cámara USB, pinza real, compilación completa en arm64/Apple Silicon, perfiles `local`/`local_gpu`, `.devcontainer`, `instalar-docker` en todas las plataformas. En ARM, `ur_rtde` se compila desde código fuente. -->

## Anexos

### A. ¿Qué es Docker?

Docker es una plataforma que permite ejecutar aplicaciones dentro de **contenedores**: entornos aislados que incluyen el sistema de archivos, las librerías y las dependencias necesarias, pero que comparten el núcleo del sistema anfitrión (son más ligeros que una máquina virtual).

- **Imagen**: plantilla de solo lectura con todo el software instalado (aquí, Ubuntu 24.04 + ROS 2 Jazzy + MoveIt + el escritorio). Se construye a partir de un `Dockerfile` (`dockerfiles/`).
- **Contenedor**: instancia en ejecución de una imagen. Lo que se cambia dentro persiste mientras el contenedor exista (aunque esté parado), pero se pierde si se elimina.
- **Volumen compartido (bind mount)**: carpeta del anfitrión visible dentro del contenedor. Aquí, `./ros_workspace`.
- **Docker Compose**: herramienta que describe en un archivo (`docker-compose.yml`) uno o varios contenedores y su configuración (puertos, volúmenes, límites de recursos). Los *perfiles* (`profiles`) permiten activar servicios opcionales.

Gracias a Docker, todo el alumnado trabaja con el mismo entorno ROS 2 sin instalar nada más en su equipo.

### B. ¿Qué es ROS 2?

ROS 2 (*Robot Operating System 2*) no es un sistema operativo, sino un conjunto de librerías y herramientas de código abierto para desarrollar software de robots. Esta asignatura usa la distribución **Jazzy Jalisco** (Ubuntu 24.04). Sus conceptos básicos son:

- **Nodo**: programa que realiza una tarea concreta (leer un sensor, planificar una trayectoria, controlar un robot).
- **Topic**: canal de comunicación de tipo publicador/suscriptor por el que los nodos intercambian mensajes (por ejemplo, `/joint_states`).
- **Servicio**: comunicación petición/respuesta.
- **Acción**: comunicación para tareas largas, con realimentación y posibilidad de cancelación (por ejemplo, `move_action` o el servidor de la pinza RG2).
- **Paquete**: unidad de organización del código (aquí, `launcher_robots_lab_robotica` o `rg2_driver`).
- **Archivo de lanzamiento (`launch`)**: script que arranca varios nodos con su configuración con un solo comando: `ros2 launch <paquete> <archivo>`.
- **Espacio de trabajo (workspace)**: carpeta con los paquetes propios (`ros_workspace/src`) y los resultados de su compilación.

Herramientas utilizadas en esta asignatura: **RViz** (visualización), **MoveIt 2** (planificación de movimientos) y **ros2_control** (control de los robots).

### C. ¿Qué es Colcon?

`colcon` es la herramienta de compilación de ROS 2: construye todos los paquetes de un espacio de trabajo respetando sus dependencias. Se ejecuta desde la raíz del espacio de trabajo y genera las carpetas `build/`, `install/` y `log/`.

```bash
cd ~/ros_workspace
colcon build --symlink-install
source install/setup.bash
```