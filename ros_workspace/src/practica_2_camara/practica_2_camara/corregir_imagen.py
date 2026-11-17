"""Corrige la distorsión de las imágenes de la cámara con los parámetros de camera_calibration."""

from dataclasses import dataclass
from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml

VENTANA = "Original (izquierda) | Corregida (derecha)"
# alpha = 0 recorta todos los píxeles negros; alpha = 1 conserva todo el campo de visión.
ALPHA_POR_DEFECTO = 0.0
ESPERA_TECLADO_MS = 1


@dataclass(frozen=True)
class ParametrosCamara:
    """Parámetros intrínsecos de una cámara para una resolución concreta."""

    ancho: int
    alto: int
    matriz_camara: np.ndarray
    distorsion: np.ndarray


def cargar_calibracion(ruta: Path) -> ParametrosCamara:
    """Lee el ost.yaml que genera el botón SAVE de camera_calibration."""
    with ruta.open(encoding="utf-8") as archivo:
        datos = yaml.safe_load(archivo)
    matriz = np.asarray(datos["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    distorsion = np.asarray(datos["distortion_coefficients"]["data"], dtype=np.float64)
    return ParametrosCamara(
        ancho=int(datos["image_width"]),
        alto=int(datos["image_height"]),
        matriz_camara=matriz,
        distorsion=distorsion,
    )


class CorrectorDistorsion:
    """Calcula una sola vez los mapas de corrección y los aplica a cada imagen."""

    def __init__(self, parametros: ParametrosCamara, alpha: float) -> None:
        """Construye las tablas de píxeles que usa cv2.remap."""
        tamano = (parametros.ancho, parametros.alto)
        # Nueva matriz de cámara para la imagen corregida: alpha decide cuánto
        # del borde curvado se conserva y roi es la zona sin píxeles negros.
        self.matriz_nueva, self.roi = cv2.getOptimalNewCameraMatrix(
            parametros.matriz_camara, parametros.distorsion, tamano, alpha, tamano
        )
        # cv2.undistort recalcularía estos mapas en cada imagen; calcularlos una
        # vez y usar remap es mucho más barato con un flujo de 30 FPS.
        self._mapa_x, self._mapa_y = cv2.initUndistortRectifyMap(
            parametros.matriz_camara,
            parametros.distorsion,
            None,
            self.matriz_nueva,
            tamano,
            cv2.CV_32FC1,
        )

    def corregir(self, imagen: np.ndarray) -> np.ndarray:
        """Devuelve la imagen sin distorsión, con el mismo tamaño que la original."""
        return cv2.remap(imagen, self._mapa_x, self._mapa_y, cv2.INTER_LINEAR)


class NodoCorreccion(Node):
    """Se suscribe a las imágenes de la cámara, las corrige y publica el resultado."""

    def __init__(self) -> None:
        """Carga la calibración y crea el suscriptor y el publicador."""
        super().__init__("corrector_imagen")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/camera/image_corregida")
        self.declare_parameter(
            "calibration_file", "~/ros_workspace/calibracion/ost.yaml"
        )
        self.declare_parameter("alpha", ALPHA_POR_DEFECTO)
        self.declare_parameter("mostrar", True)

        entrada = self.get_parameter("image_topic").value
        salida = self.get_parameter("output_topic").value
        ruta = Path(self.get_parameter("calibration_file").value).expanduser()
        alpha = float(self.get_parameter("alpha").value)
        self._mostrar: bool = bool(self.get_parameter("mostrar").value)

        self._parametros = cargar_calibracion(ruta)
        self._corrector = CorrectorDistorsion(self._parametros, alpha)
        self._bridge = CvBridge()
        self._publicador = self.create_publisher(Image, salida, qos_profile_sensor_data)
        self.create_subscription(
            Image, entrada, self._recibir_imagen, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Calibración {ruta} ({self._parametros.ancho}x{self._parametros.alto}); "
            f"corrigiendo {entrada} -> {salida}"
        )

    def _recibir_imagen(self, mensaje: Image) -> None:
        """Convierte con cv_bridge, corrige y vuelve a publicar una imagen."""
        try:
            imagen = self._bridge.imgmsg_to_cv2(mensaje, desired_encoding="bgr8")
        except CvBridgeError as error:
            self.get_logger().error(f"No se ha podido convertir la imagen: {error}")
            return

        alto, ancho = imagen.shape[:2]
        # La calibración solo es válida para la resolución en la que se hizo:
        # la distancia focal y el centro óptico están expresados en píxeles.
        if (ancho, alto) != (self._parametros.ancho, self._parametros.alto):
            self.get_logger().error(
                f"La imagen es {ancho}x{alto} pero la calibración es "
                f"{self._parametros.ancho}x{self._parametros.alto}",
                throttle_duration_sec=5.0,
            )
            return

        corregida = self._corrector.corregir(imagen)
        salida = self._bridge.cv2_to_imgmsg(corregida, encoding="bgr8")
        salida.header = (
            mensaje.header
        )  # conserva la marca de tiempo y el frame originales
        self._publicador.publish(salida)

        if self._mostrar:
            cv2.imshow(VENTANA, np.hstack((imagen, corregida)))
            cv2.waitKey(ESPERA_TECLADO_MS)


def main(args: list[str] | None = None) -> None:
    """Ejecuta el nodo de corrección hasta que se interrumpa."""
    rclpy.init(args=args)
    nodo = None
    try:
        nodo = NodoCorreccion()
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
