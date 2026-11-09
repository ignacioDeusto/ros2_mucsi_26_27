# Estado cinemático del brazo UR3e en un instante de una trayectoria.
# Los tres arrays de valores siguen el orden de nombres_articulaciones.

std_msgs/Header header            # Marca de tiempo de la muestra
string trayectoria                # Trayectoria en la que se tomó la muestra
string[] nombres_articulaciones   # Articulaciones en orden base -> muñeca
float64[] posiciones              # rad
float64[] velocidades             # rad/s
float64[] aceleraciones           # rad/s^2
