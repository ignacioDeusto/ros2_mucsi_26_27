"""Exportación a CSV de las muestras cinemáticas guardadas en cada trayectoria."""

import csv
from pathlib import Path

from ur3e_kinematics.muestra_cinematica import MuestraCinematica


class ExportadorCsv:
    """Escribe un archivo CSV por trayectoria con la evolución temporal de cada articulación."""

    def __init__(self, directorio: Path, nombres_articulaciones: list[str]) -> None:
        """Prepara el exportador para escribir en el directorio indicado."""
        self._directorio = directorio
        self._cabecera = self._crear_cabecera(nombres_articulaciones)

    def guardar(self, nombre_trayectoria: str, muestras: list[MuestraCinematica]) -> Path:
        """Escribe las muestras en <directorio>/<nombre_trayectoria>.csv y devuelve su ruta.

        Si el archivo ya existe se sobrescribe, para quedarse con la última ejecución.
        """
        self._directorio.mkdir(parents=True, exist_ok=True)
        ruta = self._directorio / f"{nombre_trayectoria}.csv"
        with ruta.open("w", newline="", encoding="utf-8") as archivo:
            escritor = csv.writer(archivo)
            escritor.writerow(self._cabecera)
            for muestra in muestras:
                escritor.writerow(
                    [muestra.tiempo_s, *muestra.posiciones,
                     *muestra.velocidades, *muestra.aceleraciones]
                )
        return ruta

    @staticmethod
    def _crear_cabecera(nombres_articulaciones: list[str]) -> list[str]:
        """Columna de tiempo y, por cada articulación, posición, velocidad y aceleración.

        El orden de las columnas coincide con el de los campos de MuestraCinematica.
        """
        return (
            ["tiempo_s"]
            + [f"posicion_{nombre}_rad" for nombre in nombres_articulaciones]
            + [f"velocidad_{nombre}_rad_s" for nombre in nombres_articulaciones]
            + [f"aceleracion_{nombre}_rad_s2" for nombre in nombres_articulaciones]
        )
