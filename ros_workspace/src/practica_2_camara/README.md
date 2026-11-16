# Práctica 2: cámara y captura

El paquete incluye un lanzador de `usb_cam` y un nodo que guarda una tanda de diez imágenes recibidas consecutivamente. La calibración y la corrección de imágenes quedan pendientes.

En la raíz del repositorio están la [guía de cámara](../../../GUIA_CAMARA.md) y la [guía de calibración](../../../GUIA_CALIBRACION.md). El tablero de referencia está en `recursos/tablero_calibracion.png`.

## Compilar

En el contenedor habitual, con el entorno de ROS cargado:

```bash
cd ~/ros_workspace
colcon build --symlink-install --packages-select practica_2_camara
source install/setup.bash
```

## Publicar imágenes y capturar una tanda

La cámara debe estar disponible y ser accesible dentro del contenedor. En una terminal:

```bash
source ~/ros_workspace/install/setup.bash
ros2 launch practica_2_camara camara.launch.py
```

El lanzador espera la cámara en `/dev/video0` y solicita MJPEG a 640 × 360 y 30 FPS. Publica en `/camera/image_raw`. Estos valores se pueden cambiar para otra cámara, por ejemplo:

```bash
ros2 launch practica_2_camara camara.launch.py video_device:=/dev/video2 image_width:=640 image_height:=480
```

En otra terminal del mismo contenedor:

```bash
source ~/ros_workspace/install/setup.bash
ros2 run practica_2_camara capturar_imagenes
```

El nodo convierte los diez mensajes recibidos con `cv_bridge` y guarda los PNG en una carpeta nueva dentro de `~/ros_workspace/capturas`. Para usar otro topic o carpeta:

```bash
ros2 run practica_2_camara capturar_imagenes --ros-args -p image_topic:=/camera/image_raw -p output_dir:=/home/laboratorio/ros_workspace/capturas
```

El programa compara los píxeles y cancela la tanda si alguna imagen es exactamente igual a otra. Este criterio no garantiza diferencias visuales apreciables, porque el ruido de la cámara puede cambiar los píxeles de una escena estática: mueve un objeto durante la captura y revisa las imágenes. Son diez mensajes consecutivos **recibidos** por el nodo; ROS podría haber descartado fotogramas antes de recibirlos.

## Prueba con Docker Desktop y la cámara integrada en Windows

El servicio `desktop` habitual no tiene acceso a la cámara. Para probar sin recrearlo, primero conecta la cámara a WSL con `usbipd` y comprueba que `/dev/video0` existe en `docker-desktop`. Detén cualquier otra prueba que esté utilizando la cámara. Los siguientes comandos se ejecutan en PowerShell desde la raíz del proyecto y usan un contenedor temporal con el workspace montado:

```powershell
$workspace = (Resolve-Path .\ros_workspace).Path
$videoGid = (wsl -d docker-desktop -- stat -c '%g' /dev/video0).Trim()
docker run --rm -it --name practica-camara --user laboratorio --group-add $videoGid --device=/dev/video0:/dev/video0 --mount "type=bind,source=$workspace,target=/home/laboratorio/ros_workspace" --entrypoint bash mucsi_26_27_desktop_novnc -lc "source /opt/ros/jazzy/setup.bash && source /home/laboratorio/ros_workspace/install/setup.bash && ros2 launch practica_2_camara camara.launch.py"
```

Deja el publicador abierto y, desde otra ventana de PowerShell, inicia la captura:

```powershell
docker exec -it -u laboratorio practica-camara bash -lc "source /opt/ros/jazzy/setup.bash && source /home/laboratorio/ros_workspace/install/setup.bash && ros2 run practica_2_camara capturar_imagenes"
```

Las capturas quedan en `ros_workspace/capturas` en Windows. Pulsa Ctrl+C en la ventana del publicador cuando termines; el contenedor temporal se eliminará, pero las capturas se conservarán en el workspace. Si el dispositivo o el formato cambia, ajusta los argumentos del lanzador.
