# Resumen de la práctica 2

## Contenido
1. [Preparación](#1-preparación)
2. [Parte A — Tablero simulado](#2-parte-a--tablero-simulado)
3. [Parte B — Cámara real](#3-parte-b--cámara-real)
4. [Cerrar](#4-cerrar)
5. [Problemas encontrados](#5-problemas-encontrados)

Todos los comandos en PowerShell, desde la raíz del proyecto.

## 1. Preparación
```powershell
docker ps                      # mucsi-robotics-practice-02-desktop-1 en marcha
docker compose start desktop   # si está parado
.\lab.bat compilar
```
Escritorio: http://localhost:6081 (contraseña `laboratorio`).

## 2. Parte A — Tablero simulado
Sin cámara. Tablero virtual 9×6 esquinas, cuadrado 0,025 m.

**2.1 Publicar el tablero** (ventana A)
```powershell
docker exec -it -u laboratorio -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp mucsi-robotics-practice-02-desktop-1 bash -lc "source /opt/ros/jazzy/setup.bash && source /home/laboratorio/ros_workspace/install/setup.bash && ros2 run practica_2_camara tablero_simulado"
```

**2.2 Calibrar** (ventana B) → CALIBRATE → SAVE
```powershell
docker exec -it -u laboratorio -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp mucsi-robotics-practice-02-desktop-1 bash -lc "source /opt/ros/jazzy/setup.bash && source /home/laboratorio/ros_workspace/install/setup.bash && ros2 launch practica_2_camara calibracion.launch.py board_size:=9x6 square_size:=0.025 image_topic:=/simulacion/image_raw"
```

**2.3 Extraer `ost.yaml`**
```powershell
New-Item -ItemType Directory -Force -Path .\ros_workspace\calibracion_simulada | Out-Null
docker cp mucsi-robotics-practice-02-desktop-1:/tmp/calibrationdata.tar.gz .\ros_workspace\calibracion_simulada\calibrationdata.tar.gz
tar -xzf .\ros_workspace\calibracion_simulada\calibrationdata.tar.gz -C .\ros_workspace\calibracion_simulada ost.yaml
```
Resultado de ejemplo: `resultados_simulacion/ost_simulado.yaml`.

**2.4 Corregir** (con 2.1 abierto)
```powershell
docker exec -it -u laboratorio -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp mucsi-robotics-practice-02-desktop-1 bash -lc "source /opt/ros/jazzy/setup.bash && source /home/laboratorio/ros_workspace/install/setup.bash && ros2 run practica_2_camara corregir_imagen --ros-args -p image_topic:=/simulacion/image_raw -p calibration_file:=/home/laboratorio/ros_workspace/calibracion_simulada/ost.yaml"
```

## 3. Parte B — Cámara real
Resolución usada: **640×480** (la webcam no soporta 640×360).

**3.1 Conectar la cámara a WSL**
```powershell
& "C:\Program Files\usbipd-win\usbipd.exe" list                     # localizar BUSID (aquí 1-4)
& "C:\Program Files\usbipd-win\usbipd.exe" bind --busid 1-4          # una vez, como administrador
& "C:\Program Files\usbipd-win\usbipd.exe" attach --wsl --busid 1-4
wsl -d docker-desktop -- ls -l /dev/video0                            # debe existir
```

**3.2 Publicar la cámara** (ventana A, dejar abierta)
```powershell
$workspace = (Resolve-Path .\ros_workspace).Path
$videoGid = (wsl -d docker-desktop -- stat -c '%g' /dev/video0).Trim()
docker run --rm -it --name practica-camara --network container:mucsi-robotics-practice-02-desktop-1 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --user laboratorio --group-add $videoGid --device=/dev/video0:/dev/video0 --mount "type=bind,source=$workspace,target=/home/laboratorio/ros_workspace" --entrypoint bash mucsi_26_27_desktop_novnc -lc "source /opt/ros/jazzy/setup.bash && source /home/laboratorio/ros_workspace/install/setup.bash && ros2 launch practica_2_camara camara.launch.py image_width:=640 image_height:=480"
```

**3.3 Comprobar formatos e imagen**
```powershell
docker exec -it practica-camara bash -lc "v4l2-ctl -d /dev/video0 --list-formats-ext"
```
```bash
# terminal del escritorio noVNC
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

**3.4 Capturar 10 imágenes** (ventana B) → `ros_workspace\capturas`
```powershell
docker exec -it -u laboratorio -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp practica-camara bash -lc "source /opt/ros/jazzy/setup.bash && source /home/laboratorio/ros_workspace/install/setup.bash && ros2 run practica_2_camara capturar_imagenes"
```

**3.5 Calibrar** (ventana B) → CALIBRATE → SAVE (no COMMIT)
```powershell
docker exec -it -u laboratorio -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp mucsi-robotics-practice-02-desktop-1 bash -lc "source /opt/ros/jazzy/setup.bash && source /home/laboratorio/ros_workspace/install/setup.bash && ros2 launch practica_2_camara calibracion.launch.py board_size:=6x9 square_size:=<lado_en_m>"
```
Tablero `recursos/tablero_calibracion.png`, 6×9 esquinas en vertical (`9x6` en horizontal).

**3.6 Extraer `ost.yaml`**
```powershell
New-Item -ItemType Directory -Force -Path .\ros_workspace\calibracion | Out-Null
docker cp mucsi-robotics-practice-02-desktop-1:/tmp/calibrationdata.tar.gz .\ros_workspace\calibracion\calibrationdata.tar.gz
tar -xzf .\ros_workspace\calibracion\calibrationdata.tar.gz -C .\ros_workspace\calibracion ost.yaml
Get-Content .\ros_workspace\calibracion\ost.yaml
```
Obtenido: fx ≈ 642, fy ≈ 637, cx ≈ 324, cy ≈ 249 px; distorsión pequeña.

**3.7 Corregir** (con 3.2 abierto) → ventana original | corregida y topic `/camera/image_corregida`
```powershell
docker exec -it -u laboratorio -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp mucsi-robotics-practice-02-desktop-1 bash -lc "source /opt/ros/jazzy/setup.bash && source /home/laboratorio/ros_workspace/install/setup.bash && ros2 run practica_2_camara corregir_imagen"
```
Parámetros: `calibration_file`, `image_topic`, `output_topic`, `alpha` (0 recorta bordes negros, 1 conserva todo el campo de visión), `mostrar`. Para ver la corrección con claridad:
```powershell
docker exec -it -u laboratorio -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp mucsi-robotics-practice-02-desktop-1 bash -lc "source /opt/ros/jazzy/setup.bash && source /home/laboratorio/ros_workspace/install/setup.bash && ros2 run practica_2_camara corregir_imagen --ros-args -p alpha:=1.0"
```
Resultado: la webcam tiene poca distorsión; con `alpha:=0` la corregida solo sale ligeramente ampliada.

## 4. Cerrar
Ctrl+C en todas las ventanas.
```powershell
& "C:\Program Files\usbipd-win\usbipd.exe" detach --busid 1-4
```

## 5. Problemas encontrados
| Síntoma | Causa | Solución |
|---|---|---|
| `cannot open directory [/sys/class/video4linux/]` | El contenedor `desktop` no ve la cámara | Contenedor temporal de 3.2 |
| Imagen negra; captura cancelada por imágenes idénticas | 640×360 no soportado | `image_width:=640 image_height:=480` |
| El calibrador no detecta el tablero | Inclinado, sin margen blanco, reflejos | De frente, quieto, borde blanco, brillo moderado |
| `FileNotFoundError: .../calibracion/ost.yaml` | Carpeta borrada al descartar cambios sin seguimiento | Repetir 3.6 (el `.tar.gz` sigue en `/tmp` del contenedor) y commitear `ost.yaml` |
| `git status` marca archivos no tocados | Solo cambios de fin de línea (CRLF) | `git restore <archivo>`; nunca `git add .` |

Captura, calibración y corrección: **misma resolución**. `calibrationdata.tar.gz` no se sube (contiene las imágenes).
