"""Publish physically projected chessboards for calibration practice."""

import math

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


# One fixed, ideal pinhole camera and one rigid 10 x 7 board for all views.
WIDTH = 640
HEIGHT = 480
SQUARE = 0.025
BOARD_COLS = 10
BOARD_ROWS = 7
PIXELS_PER_SQUARE = 100
CAMERA_MATRIX = np.array([[620., 0., WIDTH / 2],
                          [0., 620., HEIGHT / 2],
                          [0., 0., 1.]], dtype=np.float64)


def crear_textura() -> np.ndarray:
    """Build a crisp board with a white half-square border for detection."""
    half = PIXELS_PER_SQUARE // 2
    width = (BOARD_COLS + 1) * PIXELS_PER_SQUARE
    height = (BOARD_ROWS + 1) * PIXELS_PER_SQUARE
    board = np.full((height, width), 255, dtype=np.uint8)
    for row in range(BOARD_ROWS):
        for col in range(BOARD_COLS):
            if (row + col) % 2 == 0:
                y = half + row * PIXELS_PER_SQUARE
                x = half + col * PIXELS_PER_SQUARE
                board[y:y + PIXELS_PER_SQUARE, x:x + PIXELS_PER_SQUARE] = 0
    return board


def proyectar_tablero(rng: np.random.Generator, textura: np.ndarray) -> np.ndarray:
    """Render one random planar pose using fixed camera intrinsics."""
    half_width = (BOARD_COLS + 1) * SQUARE / 2
    half_height = (BOARD_ROWS + 1) * SQUARE / 2
    corners_3d = np.array([
        [-half_width, -half_height, 0.],
        [half_width, -half_height, 0.],
        [half_width, half_height, 0.],
        [-half_width, half_height, 0.],
    ], dtype=np.float64)
    image_corners = np.float32([
        [0, 0], [textura.shape[1] - 1, 0],
        [textura.shape[1] - 1, textura.shape[0] - 1], [0, textura.shape[0] - 1],
    ])

    for _ in range(200):
        # A pose is generated in three dimensions, not by an arbitrary warp:
        # every frame therefore follows the same pinhole camera geometry.
        ax, ay, az = np.deg2rad(rng.uniform([-65., -65., -25.], [65., 65., 25.]))
        rx = np.array([[1, 0, 0], [0, math.cos(ax), -math.sin(ax)],
                       [0, math.sin(ax), math.cos(ax)]])
        ry = np.array([[math.cos(ay), 0, math.sin(ay)], [0, 1, 0],
                       [-math.sin(ay), 0, math.cos(ay)]])
        rz = np.array([[math.cos(az), -math.sin(az), 0],
                       [math.sin(az), math.cos(az), 0], [0, 0, 1]])
        rotation, _ = cv2.Rodrigues(rz @ ry @ rx)
        depth = float(rng.uniform(0.43, 1.10))
        projected, _ = cv2.projectPoints(
            corners_3d, rotation, np.array([0., 0., depth]), CAMERA_MATRIX, None)
        projected = projected.reshape(-1, 2)
        lo = projected.min(axis=0)
        hi = projected.max(axis=0)
        margin = 8.
        min_shift = np.array([margin, margin]) - lo
        max_shift = np.array([WIDTH - margin, HEIGHT - margin]) - hi
        if np.any(min_shift >= max_shift):
            continue
        # Choose a target shift, then recompute using a real 3D translation.
        # Shifting 2D corners directly would change the effective camera model
        # for tilted boards because the corners have different depths.
        shift = rng.uniform(min_shift, max_shift)
        translation = np.array([
            shift[0] * depth / CAMERA_MATRIX[0, 0],
            shift[1] * depth / CAMERA_MATRIX[1, 1], depth,
        ])
        projected, _ = cv2.projectPoints(corners_3d, rotation, translation, CAMERA_MATRIX, None)
        projected = projected.reshape(-1, 2)
        if (np.any(projected.min(axis=0) < margin) or
                np.any(projected.max(axis=0) > np.array([WIDTH, HEIGHT]) - margin)):
            continue
        transform = cv2.getPerspectiveTransform(image_corners, projected.astype(np.float32))
        canvas = cv2.warpPerspective(
            textura, transform, (WIDTH, HEIGHT), flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT, borderValue=180)
        return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    raise RuntimeError('No se pudo generar una posición válida del tablero')


class TableroSimulado(Node):
    """Publish reproducible, varied views on a separate simulated topic."""

    def __init__(self) -> None:
        """Create the publisher and a timer; no physical camera is used."""
        super().__init__('tablero_simulado')
        self.declare_parameter('image_topic', '/simulacion/image_raw')
        self.declare_parameter('periodo', 0.5)
        self.declare_parameter('semilla', 42)
        topic = self.get_parameter('image_topic').value
        period = self.get_parameter('periodo').value
        seed = self.get_parameter('semilla').value
        if not topic or period <= 0:
            raise ValueError('El topic no puede estar vacío y periodo debe ser positivo')
        self._rng = np.random.default_rng(seed)
        self._textura = crear_textura()
        self._bridge = CvBridge()
        self._publisher = self.create_publisher(Image, topic, qos_profile_sensor_data)
        self.create_timer(period, self._publicar)
        self.get_logger().info(
            f'Publicando vistas SIMULADAS en {topic}; tablero 9x6, cuadrado 0.025 m')

    def _publicar(self) -> None:
        """Publish a new image of the same board and simulated camera."""
        imagen = proyectar_tablero(self._rng, self._textura)
        mensaje = self._bridge.cv2_to_imgmsg(imagen, encoding='bgr8')
        mensaje.header.stamp = self.get_clock().now().to_msg()
        mensaje.header.frame_id = 'camara_simulada'
        self._publisher.publish(mensaje)


def main(args: list[str] | None = None) -> None:
    """Run the simulated camera until interrupted."""
    rclpy.init(args=args)
    nodo = None
    try:
        nodo = TableroSimulado()
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
