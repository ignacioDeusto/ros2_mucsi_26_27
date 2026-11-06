"""Elementos de comunicación compartidos entre el nodo de trayectorias y el lector."""

from rclpy.qos import DurabilityPolicy, QoSProfile

TOPIC_TRAYECTORIA_ACTUAL = "/trayectoria_actual"
SIN_TRAYECTORIA = ""  # Se publica entre trayectorias para indicar que no hay que grabar

# TRANSIENT_LOCAL guarda el último mensaje y se lo entrega a los suscriptores
# que se conecten tarde: así el lector no se pierde el anuncio de la trayectoria
QOS_TRAYECTORIA = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

