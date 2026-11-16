"""Exercise ROS camera calibration using known synthetic camera geometry."""

from io import BytesIO
import tarfile

from camera_calibration.calibrator import ChessboardInfo, MonoCalibrator
from cv_bridge import CvBridge
import numpy as np
from practica_2_camara.tablero_simulado import CAMERA_MATRIX
from practica_2_camara.tablero_simulado import crear_textura
from practica_2_camara.tablero_simulado import proyectar_tablero
from practica_2_camara.tablero_simulado import SQUARE
import yaml


def test_calibracion_con_geometria_conocida() -> None:
    """Detect varied views, enable calibration, and save plausible parameters."""
    tablero = ChessboardInfo(pattern='chessboard', n_cols=9, n_rows=6, dim=SQUARE)
    calibrador = MonoCalibrator([tablero])
    bridge = CvBridge()
    rng = np.random.default_rng(42)
    textura = crear_textura()

    # Reuse the publicador's generator, not the real camera or its data.
    for _ in range(200):
        imagen = proyectar_tablero(rng, textura)
        mensaje = bridge.cv2_to_imgmsg(imagen, encoding='bgr8')
        calibrador.handle_msg(mensaje)
        if calibrador.goodenough:
            break

    # This is the same condition that enables CALIBRATE in the ROS GUI.
    assert calibrador.goodenough, 'No se reunieron suficientes vistas distintas'
    calibrador.do_calibration()
    assert calibrador.calibrated
    # The checkerboard is rendered by a fixed, known camera matrix. A tolerance
    # allows for interpolation and sub-pixel corner detection in OpenCV.
    np.testing.assert_allclose(calibrador.intrinsics, CAMERA_MATRIX, atol=8.0)

    # Exercise the calibrator's SAVE format entirely in memory; do not overwrite
    # /tmp/calibrationdata.tar.gz, which may contain someone's real calibration.
    memoria = BytesIO()
    with tarfile.open(fileobj=memoria, mode='w:gz') as archivo:
        calibrador.do_tarfile_save(archivo)
    memoria.seek(0)
    with tarfile.open(fileobj=memoria, mode='r:gz') as archivo:
        contenido = archivo.extractfile('ost.yaml')
        assert contenido is not None
        parametros = yaml.safe_load(contenido.read())

    assert (parametros['image_width'], parametros['image_height']) == (640, 480)
    matriz_guardada = np.asarray(parametros['camera_matrix']['data']).reshape(3, 3)
    np.testing.assert_allclose(matriz_guardada, CAMERA_MATRIX, atol=8.0)
