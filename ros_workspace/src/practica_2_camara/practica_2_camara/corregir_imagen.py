"""Corrige la distorsión de las imágenes de la cámara con los parámetros de camera_calibration."""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml


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
