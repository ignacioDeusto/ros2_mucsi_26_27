# Calibrar la cámara

La calibración obtiene parámetros de la cámara. La corrección posterior de imágenes con OpenCV se realizará en la siguiente parte de la práctica.

## 1. Preparar el tablero

Abre en el móvil `recursos/tablero_calibracion.png`, o descarga la [imagen original](https://github.com/bbugly/camera-calibration/blob/main/checkerboard_7x10_squares_25mm.png) en el móvil. Muestra el tablero completo, sin recortes ni deformaciones y con un margen blanco alrededor si la aplicación lo permite.

El tablero tiene **6 × 9 esquinas interiores** en su orientación vertical. Mide con una regla el lado real de un cuadrado tal como aparece en la pantalla y exprésalo en metros. Por ejemplo, 10 mm son 0,010 m. El valor del nombre original del archivo no representa necesariamente el tamaño en tu móvil. Mantén el mismo zoom durante toda la calibración.

Una pantalla pequeña puede producir reflejos, parpadeos o patrones de interferencia; si las esquinas no se detectan bien, una impresión plana es una alternativa más fiable.

## 2. Arrancar la cámara

Sigue `GUIA_CAMARA.md` hasta dejar abierto el publicador del paso 3. El escritorio habitual debe estar en marcha y el publicador debe usar la red compartida y Cyclone DDS que indica la guía. Abre el escritorio en [http://localhost:6081](http://localhost:6081), contraseña `laboratorio`. Si la pantalla está bloqueada, desbloquéala.

## 3. Abrir el calibrador

En otra ventana de PowerShell, ejecuta el siguiente comando. Sustituye `0.010` por la medida real de tu cuadrado en metros:

```powershell
docker exec -it -u laboratorio -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp mucsi-robotics-practice-02-desktop-1 bash -lc "source /opt/ros/jazzy/setup.bash && source /home/laboratorio/ros_workspace/install/setup.bash && ros2 launch practica_2_camara calibracion.launch.py board_size:=6x9 square_size:=0.010"
```

El calibrador debe abrirse en el escritorio del navegador. Si tu contenedor habitual tiene otro nombre, cambia `mucsi-robotics-practice-02-desktop-1` por el nombre mostrado por `docker ps`. El tamaño `6x9` corresponde al tablero vertical; si se gira para mostrarlo en horizontal, utiliza `9x6`.

Muestra el tablero a la cámara y muévelo lentamente por el centro, los bordes y las esquinas de la imagen; acércalo, aléjalo e inclínalo en diferentes direcciones. Mantén el tablero quieto brevemente en cada posición. Cuando el botón **CALIBRATE** se active, púlsalo y espera a que termine. Si las esquinas no se detectan o la imagen corregida no es reconocible, revisa la iluminación y las medidas y repite la toma.

## 4. Guardar los resultados

Pulsa **SAVE** en la ventana del calibrador. Este botón crea `/tmp/calibrationdata.tar.gz` dentro del contenedor de escritorio; no lo deja automáticamente en el proyecto. Sin cerrar el contenedor, ejecuta desde PowerShell en la raíz del proyecto:

```powershell
New-Item -ItemType Directory -Force -Path .\ros_workspace\calibracion | Out-Null
docker cp mucsi-robotics-practice-02-desktop-1:/tmp/calibrationdata.tar.gz .\ros_workspace\calibracion\calibrationdata.tar.gz
tar -xzf .\ros_workspace\calibracion\calibrationdata.tar.gz -C .\ros_workspace\calibracion ost.yaml
explorer .\ros_workspace\calibracion
```

`ost.yaml` contiene los parámetros calculados. El archivo comprimido también puede contener imágenes usadas para calibrar: revísalas antes de compartirlas. Guarda resultados de distintas cámaras o resoluciones con nombres separados para no sobrescribirlos. La calibración se realiza por cámara y resolución; no reutilices sin verificar los parámetros de la cámara USB para la integrada, ni viceversa.

`SAVE` guarda los resultados, pero no aplica por sí solo la corrección a las imágenes. Para terminar, detén el calibrador y el publicador con Ctrl+C y desconecta la cámara de WSL como indica la guía de cámara.
