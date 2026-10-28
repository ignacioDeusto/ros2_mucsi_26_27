from launcher_robots_lab_robotica.lanzador_openarm import generar_lanzamiento_openarm


def generate_launch_description():
    return generar_lanzamiento_openarm("302", simulacion=True)
