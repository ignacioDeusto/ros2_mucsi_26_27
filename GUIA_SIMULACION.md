# Prueba de calibración simulada

Esta prueba genera un tablero virtual en distintas posiciones y lo publica por ROS. Sirve para practicar y comprobar `camera_calibration` sin cámara ni móvil. El resultado corresponde a una cámara simulada; **no es una calibración de la cámara real**.

## 1. Preparar el entorno

Abre Docker Desktop y PowerShell en la raíz del proyecto. El contenedor `mucsi-robotics-practice-02-desktop-1` debe estar en marcha. Si ya existe pero está parado, puedes arrancarlo con `docker compose start desktop`. Si aún no existe, créalo con `.\lab.bat iniciar`.

Compila el paquete:

```powershell
.\lab.bat compilar
```

Abre [http://localhost:6081](http://localhost:6081) y accede al escritorio con la contraseña `laboratorio`. Si está bloqueado, desbloquéalo.

## 2. Publicar el tablero simulado

Ejecuta en PowerShell y deja la ventana abierta:

```powershell
docker exec -it -u laboratorio -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp mucsi-robotics-practice-02-desktop-1 bash -lc "source /opt/ros/jazzy/setup.bash && source /home/laboratorio/ros_workspace/install/setup.bash && ros2 run practica_2_camara tablero_simulado"
```

El programa publica imágenes en `/simulacion/image_raw`. Utiliza un tablero de 9 × 6 esquinas interiores y cuadrados virtuales de 0,025 m.

## 3. Calibrar

Abre otra ventana de PowerShell y ejecuta:

```powershell
docker exec -it -u laboratorio -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp mucsi-robotics-practice-02-desktop-1 bash -lc "source /opt/ros/jazzy/setup.bash && source /home/laboratorio/ros_workspace/install/setup.bash && ros2 launch practica_2_camara calibracion.launch.py board_size:=9x6 square_size:=0.025 image_topic:=/simulacion/image_raw"
```

En el escritorio del navegador debe aparecer el tablero cambiando de posición automáticamente. Espera a que el botón **CALIBRATE** se active y púlsalo. Al terminar el cálculo podrás pulsar **SAVE**.

**Antes de pulsar SAVE:** el calibrador escribe siempre `/tmp/calibrationdata.tar.gz` en ese contenedor, y puede sobrescribir una calibración anterior. Si ya habías guardado una calibración real allí, cópiala a un lugar seguro antes de continuar. No pulses COMMIT: esta prueba no debe modificar los datos de una cámara real.

## 4. Conservar el resultado simulado

Tras pulsar SAVE, desde PowerShell en la raíz del proyecto:

```powershell
New-Item -ItemType Directory -Force -Path .\ros_workspace\calibracion_simulada | Out-Null
docker cp mucsi-robotics-practice-02-desktop-1:/tmp/calibrationdata.tar.gz .\ros_workspace\calibracion_simulada\calibrationdata.tar.gz
tar -xzf .\ros_workspace\calibracion_simulada\calibrationdata.tar.gz -C .\ros_workspace\calibracion_simulada ost.yaml
explorer .\ros_workspace\calibracion_simulada
```

El archivo `ost.yaml` contiene parámetros **simulados**. No lo uses para corregir las imágenes de la cámara integrada o USB. Pulsa Ctrl+C en las dos ventanas de PowerShell para terminar la prueba.

## Prueba automática

El archivo `ros_workspace/src/practica_2_camara/test/test_calibracion_simulada.py` comprueba con el calibrador real de ROS que se aceptan suficientes vistas, se calcula una matriz próxima a la conocida de la cámara virtual y se produce un `ost.yaml` válido. La prueba guarda el archivo en memoria y no modifica los parámetros de una cámara real.

En una ejecución de prueba, el calibrador aceptó 40 de 75 vistas generadas y alcanzó la condición que habilita **CALIBRATE**. Para una cámara virtual con `fx = fy = 620`, `cx = 320` y `cy = 240` píxeles, recuperó aproximadamente `fx = 620,345`, `fy = 620,416`, `cx = 320,126` y `cy = 239,635`. La prueba automática verificó también el YAML incluido en el archivo de guardado. Estos valores solo describen la cámara simulada.

Además, se probó la interfaz gráfica en un entorno aislado: tras aceptar 40 vistas, se habilitó **CALIBRATE**, se completó el cálculo y **SAVE** produjo un archivo de parámetros. El ejemplo obtenido está en `resultados_simulacion/ost_simulado.yaml`.
