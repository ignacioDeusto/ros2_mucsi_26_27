# Tablero de calibración

`tablero_calibracion.png` procede del proyecto [bbugly/camera-calibration](https://github.com/bbugly/camera-calibration/blob/main/checkerboard_7x10_squares_25mm.png) y se distribuye bajo la licencia MIT incluida en `LICENSE-tablero.txt`.

El tablero tiene 7 × 10 cuadrados: son 6 × 9 esquinas interiores (o 9 × 6 si se gira). Al mostrarlo en un móvil, el tamaño real de cada cuadrado dependerá de la pantalla y del zoom: mide la distancia entre varias esquinas consecutivas y divídela por el número de cuadrados medidos. Usa el resultado en metros con `camera_calibration` y mantén la misma escala durante la calibración.

Muestra el tablero completo sin deformarlo ni recortarlo, evita reflejos y comprueba que las esquinas se detecten. Una impresión plana y sin brillos suele resultar más fiable que una pantalla pequeña.
