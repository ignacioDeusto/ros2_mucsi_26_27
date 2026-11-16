"""Capture a batch of ten consecutive, distinct ROS camera images."""

from hashlib import sha256
from pathlib import Path
import tempfile
from typing import Any

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


NUMERO_IMAGENES = 10


class CapturadorImagenes(Node):
    """Collect and save one batch without silently skipping repeated frames."""

    def __init__(self) -> None:
        """Configure the source topic and the destination directory."""
        super().__init__('capturador_imagenes')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_dir', '~/ros_workspace/capturas')
        topic = self.get_parameter('image_topic').get_parameter_value().string_value
        directorio = self.get_parameter('output_dir').get_parameter_value().string_value
        if not topic or not directorio:
            raise ValueError('image_topic y output_dir no pueden estar vacíos')

        self._directorio = Path(directorio).expanduser()
        self._bridge = CvBridge()
        self._imagenes: list[Any] = []
        self._huellas: set[bytes] = set()
        self.terminado = False
        self.correcto = False
        self.create_subscription(Image, topic, self._recibir_imagen, qos_profile_sensor_data)
        self.get_logger().info(f'Esperando {NUMERO_IMAGENES} imágenes consecutivas en {topic}')

    def _recibir_imagen(self, mensaje: Image) -> None:
        """Convert each received frame and reject an exactly repeated image."""
        if self.terminado:
            return
        try:
            imagen = self._bridge.imgmsg_to_cv2(mensaje, desired_encoding='bgr8')
        except (CvBridgeError, ValueError) as error:
            self.get_logger().error(f'No se ha podido convertir la imagen: {error}')
            self.terminado = True
            return

        huella = sha256(imagen.tobytes()).digest()
        if huella in self._huellas:
            self.get_logger().error(
                'Se ha recibido una imagen idéntica a otra de la tanda. '
                'Captura cancelada: mueve la escena y vuelve a intentarlo.'
            )
            self.terminado = True
            return

        self._huellas.add(huella)
        self._imagenes.append(imagen.copy())
        self.get_logger().info(f'Imagen {len(self._imagenes)}/{NUMERO_IMAGENES} recibida')
        if len(self._imagenes) == NUMERO_IMAGENES:
            self._guardar_tanda()
            self.terminado = True

    def _guardar_tanda(self) -> None:
        """Write images as PNG files in a new, uniquely named directory."""
        try:
            self._directorio.mkdir(parents=True, exist_ok=True)
            destino = Path(tempfile.mkdtemp(prefix='captura_', dir=self._directorio))
            for indice, imagen in enumerate(self._imagenes, start=1):
                archivo = destino / f'imagen_{indice:02d}.png'
                if not cv2.imwrite(str(archivo), imagen):
                    raise OSError(f'No se ha podido escribir {archivo}')
        except (OSError, cv2.error) as error:
            self.get_logger().error(f'Error al guardar la tanda: {error}')
            return

        self.correcto = True
        self.get_logger().info(f'Tanda guardada en {destino}')


def main(args: list[str] | None = None) -> None:
    """Run until one batch has been captured or an error occurs."""
    rclpy.init(args=args)
    nodo = None
    try:
        nodo = CapturadorImagenes()
        while rclpy.ok() and not nodo.terminado:
            rclpy.spin_once(nodo, timeout_sec=1.0)
        if not nodo.correcto:
            raise SystemExit(1)
    except KeyboardInterrupt:
        pass
    finally:
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
