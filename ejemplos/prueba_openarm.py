#!/usr/bin/python3
"""Primera prueba de movimiento de un brazo OpenArm (simulado o real).

Gira la primera articulación del brazo una cantidad pequeña, la devuelve a su sitio y abre y
cierra un poco la pinza. Necesita control_robot.py en la misma carpeta.

Uso (dentro del contenedor, con el robot o la simulación ya lanzados):
    python3 prueba_openarm.py               # brazo 301_izq, 0,05 rad
    python3 prueba_openarm.py 301_der 0.1   # otro brazo y otro ángulo (rad)
"""

import sys

import rclpy

from control_robot import ControlRobot

BRAZO = sys.argv[1] if len(sys.argv) > 1 else "301_izq"
ANGULO = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05   # rad (0,05 rad ≈ 3°)

rclpy.init()
control = ControlRobot(BRAZO)

inicio = control.articulaciones_actuales()          # 7 valores en radianes
print(f"Articulaciones de {BRAZO}: {[round(v, 3) for v in inicio]}")

objetivo = list(inicio)
objetivo[0] += ANGULO
print(f"joint1 {ANGULO:+} rad")
control.mover_articulaciones(objetivo)
print("Vuelta a la posición inicial")
control.mover_articulaciones(inicio)

print("Pinza: abrir a 30 mm y cerrar")
control.mover_pinza(30.0, 10.0)
control.mover_pinza(0.0, 10.0)

control.ejecutor.shutdown()
control.destroy_node()
rclpy.shutdown()
