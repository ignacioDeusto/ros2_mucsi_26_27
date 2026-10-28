"""Configuración de cada uno de los robots del laboratorio."""

ROBOTS = {
    "203": {
        "robot_ip": "10.172.21.203",
        "reverse_port": "50030",
        "script_sender_port": "50031",
        "trajectory_port": "50032",
    },
    "204": {
        "robot_ip": "10.172.21.204",
        "reverse_port": "50040",
        "script_sender_port": "50041",
        "trajectory_port": "50042",
    },
    "205": {
        "robot_ip": "10.172.21.205",
        "reverse_port": "50050",
        "script_sender_port": "50051",
        "trajectory_port": "50052",
    },
    "206": {
        "robot_ip": "10.172.21.206",
        "reverse_port": "50060",
        "script_sender_port": "50061",
        "trajectory_port": "50062",
    },
}

TOOL_TCP_PORT = "54321"

# Cuerpos OpenArm: cada cuerpo lleva dos brazos (izquierdo y derecho) que comparten el
# equipo. Son simulaciones (mock hardware); el robot real necesitaría openarm_hardware
# más la librería openarm_can y un bus CAN por brazo.
ROBOTS_OPENARM = {
    "301": {
        "arm_type": "openarm_v2.0",
        "left_can_interface": "can0",
        "right_can_interface": "can1",
        "can_fd": "true",
    },
    "302": {
        "arm_type": "openarm_v2.0",
        "left_can_interface": "can2",
        "right_can_interface": "can3",
        "can_fd": "true",
    },
}

# Sufijos de los brazos de un cuerpo OpenArm (los nombres de nodo no pueden empezar por
# un número ni contener puntos, y se usa "izq"/"der" para distinguir cada brazo)
LADOS_OPENARM = {"izq": "left", "der": "right"}

# Pinza del OpenArm (pinch_gripper de openarm_description, dedos rotativos):
# el pivote de cada dedo está a 18 mm del plano de simetría y la cara de agarre a
# 95,9 mm del pivote; con el dedo a |θ| rad la anchura entre dedos es
#   2 · (18 mm · (1 − cos |θ|) + 95,9 mm · sen |θ|)
# que con la pinza totalmente abierta (|θ| = 0,7854 rad) da unos 146,2 mm. La
# conversión es lineal en ese recorrido (±8 mm según el punto de contacto).
RAD_ABIERTA_OPENARM = 0.7854          # finger_joint1 con la pinza totalmente abierta
SIGNO_ABIERTA_OPENARM = {"izq": 1.0, "der": -1.0}  # Signo de finger_joint1 al abrir
ANCHURA_MAXIMA_OPENARM_MM = 146.2     # Anchura total entre dedos totalmente abierta
