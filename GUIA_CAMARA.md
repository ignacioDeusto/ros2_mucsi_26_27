# Guía rápida: cámara y captura en Windows

El entorno se ha probado con la cámara integrada **ACER HD User Facing** y con la cámara **HD USB Camera**. Las dos llegaron a publicar imágenes mediante `usb_cam`; con la integrada también se probó el nodo de captura. Otras cámaras pueden necesitar una resolución o formato distinto.

## 1. Preparar Docker y el proyecto

Abre Docker Desktop y espera a que arranque. Cierra la aplicación Cámara de Windows si la estabas usando. En PowerShell, entra en el proyecto:

```powershell
cd "C:\Users\jmart\OneDrive\Escritorio\Master Mucsi\Asignaturas 1º Cuatri\Robotica y automatización\mucsi-robotics-practice-02"
```

Si el contenedor `mucsi-robotics-practice-02-desktop-1` ya está en marcha, no hace falta iniciarlo de nuevo. Compruébalo con `docker ps`. Si existe pero está parado, arráncalo con `docker compose start desktop`. Si todavía no existe, créalo con `.\lab.bat iniciar`; la primera vez puede tardar. Este último comando reconstruye la imagen y puede recrear un contenedor existente, por lo que los cambios guardados fuera del workspace podrían perderse.

Con el contenedor en marcha, compila el paquete:

```powershell
.\lab.bat compilar
```

## 2. Conectar la cámara a WSL

`usbipd-win` ya se instaló en este ordenador. Si se necesita en otro equipo, se instala una vez con:

```powershell
winget install --interactive --exact dorssel.usbipd-win
```

En PowerShell, comprueba que hay alguna distribución WSL 2 en ejecución y localiza la cámara:

```powershell
wsl --list --verbose
& "C:\Program Files\usbipd-win\usbipd.exe" list
```

Si ninguna distribución WSL 2 está en ejecución, abre Ubuntu desde Inicio y déjalo abierto. En la lista de USB, identifica el **BUSID** por el nombre: en las pruebas anteriores era `2-6` para la integrada y `2-4` para la USB. **Vuelve a comprobarlo: puede cambiar.**

Si la cámara aparece como `Not shared`, abre PowerShell **como administrador** y sustituye `BUSID` por el identificador que muestra tu lista:

```powershell
& "C:\Program Files\usbipd-win\usbipd.exe" bind --busid BUSID
```

Este paso de compartir el dispositivo suele hacerse una sola vez. Luego, desde PowerShell normal o como administrador, conecta la cámara a WSL y comprueba que aparezca como `Attached`:

```powershell
& "C:\Program Files\usbipd-win\usbipd.exe" attach --wsl --busid BUSID
& "C:\Program Files\usbipd-win\usbipd.exe" list
```

Mientras esté conectada a WSL, la cámara no estará disponible para las aplicaciones de Windows. Comprueba que Docker ve el dispositivo:

```powershell
wsl -d docker-desktop -- ls -l /dev/video0 /dev/video1
```

En las pruebas se usó `/dev/video0`; si tu cámara aparece con otro número, adapta los siguientes comandos. Si ya tienes otra prueba de cámara ejecutándose, detenla con Ctrl+C en su ventana para liberar el dispositivo.

## 3. Publicar imágenes

En PowerShell, desde la carpeta del proyecto, inicia un contenedor temporal con la cámara y el workspace. No reemplaza el contenedor habitual, que debe estar en marcha con el nombre `mucsi-robotics-practice-02-desktop-1`. Compruébalo con `docker ps`; si el nombre es distinto, sustitúyelo en el argumento `--network`:

```powershell
$workspace = (Resolve-Path .\ros_workspace).Path
$videoGid = (wsl -d docker-desktop -- stat -c '%g' /dev/video0).Trim()
docker run --rm -it --name practica-camara --network container:mucsi-robotics-practice-02-desktop-1 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --user laboratorio --group-add $videoGid --device=/dev/video0:/dev/video0 --mount "type=bind,source=$workspace,target=/home/laboratorio/ros_workspace" --entrypoint bash mucsi_26_27_desktop_novnc -lc "source /opt/ros/jazzy/setup.bash && source /home/laboratorio/ros_workspace/install/setup.bash && ros2 launch practica_2_camara camara.launch.py"
```

Déjalo abierto. El lanzador solicita MJPEG a **640 × 360**, configuración probada con la cámara integrada; el topic previsto es `/camera/image_raw`. Con la USB se comprobó la publicación a **320 × 240**; para usar esa resolución, añade `image_width:=320 image_height:=240` al final del comando `ros2 launch` dentro de las comillas.

La conexión de red compartida y Cyclone DDS permiten que las herramientas ROS del escritorio de la práctica, accesible en `http://localhost:6081`, reciban las imágenes. Al ejecutar una herramienta ROS en el escritorio, usa también `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

La prueba anterior realizada copiando el script al contenedor `prueba_camara_integrada` usaba `/image_raw`. En esta guía se utiliza el lanzador del paquete, que añade el espacio de nombres `/camera`.

## 4. Capturar y ver diez imágenes

Sin cerrar el publicador, abre **otra ventana de PowerShell**. Mueve un objeto delante de la cámara mientras ejecutas:

```powershell
docker exec -it -u laboratorio -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp practica-camara bash -lc "source /opt/ros/jazzy/setup.bash && source /home/laboratorio/ros_workspace/install/setup.bash && ros2 run practica_2_camara capturar_imagenes"
```

Debe indicar `Imagen 1/10 recibida` hasta `Imagen 10/10 recibida` y después `Tanda guardada`. Las imágenes se conservan en una carpeta nueva dentro de `ros_workspace\capturas` del proyecto. Para abrirlas en el explorador, desde PowerShell en la carpeta del proyecto:

```powershell
explorer .\ros_workspace\capturas
```

Revisa que las imágenes sean visualmente diferentes. El nodo rechaza duplicados exactos de píxeles, pero el ruido de una escena inmóvil también puede alterar los píxeles.

## 5. Terminar y devolver la cámara a Windows

Pulsa **Ctrl+C** en la ventana del publicador. Se elimina el contenedor temporal; las capturas permanecen en `ros_workspace\capturas`. Después, en PowerShell, usando el BUSID comprobado en el paso 2:

```powershell
& "C:\Program Files\usbipd-win\usbipd.exe" detach --busid BUSID
```

La cámara vuelve a quedar disponible en Windows. La calibración y la corrección de imágenes todavía están pendientes.
