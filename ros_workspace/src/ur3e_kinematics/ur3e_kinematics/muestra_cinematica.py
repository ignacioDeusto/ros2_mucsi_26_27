"""Estructura de datos con el estado cinemático del brazo en un instante."""

from dataclasses import dataclass


@dataclass
class MuestraCinematica:
    """Posición, velocidad y aceleración de las articulaciones en un instante.

    Las listas siguen el orden de las articulaciones del brazo (base -> muñeca).
    """

    tiempo_s: float
    posiciones: list[float]
    velocidades: list[float]
    aceleraciones: list[float]
