#!/usr/bin/env bash
# Levanta los buses CAN de un cuerpo OpenArm (un bus por brazo) leyendo sus interfaces de
# robots.py. Se ejecuta en el ANFITRIÓN Linux al que están conectados los adaptadores
# USB-CAN (los perfiles local/local_gpu comparten su red, así que el contenedor los verá).
#
# Uso: herramientas/levantar_can.sh <301|302> [opciones]     (o ./launcher.sh can <id> [opciones])
#   -b, --bitrate <bps>    Velocidad de arbitraje (por defecto 1000000)
#   -d, --dbitrate <bps>   Velocidad de datos CAN-FD (por defecto 5000000)
#       --no-fd            CAN clásico (sin CAN-FD; ignora --dbitrate)
#   -n, --dry-run          Muestra los comandos sin ejecutarlos
#   -h, --help             Muestra esta ayuda
#
# Los valores por defecto son los del script de Enactic
# (ros_workspace/src/openarm_can/setup/openarm-can-configure-socketcan-4-arms).
set -euo pipefail

RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOTS_PY="$RAIZ/ros_workspace/src/launcher_robots_lab_robotica/launcher_robots_lab_robotica/robots.py"

BITRATE=1000000
DBITRATE=5000000
FD=1
SIMULAR=0
ID=""

die() { echo "ERROR: $*" >&2; exit 1; }

ayuda() { sed -n '2,13p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
    case "$1" in
        -b|--bitrate) BITRATE="${2:-}"; shift 2 ;;
        -d|--dbitrate) DBITRATE="${2:-}"; shift 2 ;;
        --no-fd) FD=0; shift ;;
        -n|--dry-run) SIMULAR=1; shift ;;
        -h|--help) ayuda; exit 0 ;;
        -*) die "Opción desconocida: $1 (usa --help)" ;;
        *) [ -z "$ID" ] || die "Sobra el argumento: $1"; ID="$1"; shift ;;
    esac
done

[ -n "$ID" ] || { ayuda; exit 1; }
[[ "$BITRATE" =~ ^[0-9]+$ ]] || die "Bitrate no válido: '$BITRATE'"
[[ "$DBITRATE" =~ ^[0-9]+$ ]] || die "Dbitrate no válido: '$DBITRATE'"
[ -f "$ROBOTS_PY" ] || die "No se encuentra $ROBOTS_PY"

# Interfaces del cuerpo: las líneas "*_can_interface" del bloque "<id>": { ... } de ROBOTS_OPENARM
mapfile -t INTERFACES < <(awk -v id="\"$ID\":" '
    /^ROBOTS_OPENARM *=/ { dentro = 1; next }
    dentro && /^}/ { exit }
    dentro && $1 == id { cuerpo = 1; next }
    cuerpo && /}/ { exit }
    cuerpo && /_can_interface"/ { gsub(/[",]/, "", $2); print $2 }
' "$ROBOTS_PY")

if [ "${#INTERFACES[@]}" -eq 0 ]; then
    disponibles=$(grep -oE '^    "[0-9]+": \{' "$ROBOTS_PY" | tr -dc '0-9\n' | grep '^3' | tr '\n' ' ' || true)
    die "El cuerpo OpenArm '$ID' no existe en robots.py (disponibles: ${disponibles:-ninguno})"
fi

# Con --dry-run solo se muestran los comandos (sirve en cualquier sistema)
if [ "$SIMULAR" = 1 ]; then
    ejecutar() { echo "+ $*"; }
else
    [ "$(uname -s)" = Linux ] || die "Los buses CAN solo se pueden levantar en Linux (el PC del laboratorio conectado al OpenArm)."

    # WSL: su kernel no trae SocketCAN, así que no hay nada que hacer aquí
    if grep -qiE 'microsoft|wsl' /proc/sys/kernel/osrelease 2>/dev/null; then
        die "Estás en WSL y su kernel no incluye SocketCAN: no se pueden usar los buses CAN del OpenArm.
       Usa un PC Linux nativo del laboratorio. Aquí puedes trabajar en simulación: ./launcher.sh sim $ID"
    fi

    SUDO=""
    if [ "$(id -u)" -ne 0 ]; then
        command -v sudo >/dev/null 2>&1 || die "Hace falta ser root o tener sudo para configurar los buses CAN."
        SUDO=sudo
    fi
    command -v ip >/dev/null 2>&1 || die "Falta el comando 'ip' (paquete iproute2)."

    # Kernel con SocketCAN: /proc/net/can aparece al cargar el módulo can
    if [ ! -d /proc/net/can ]; then
        $SUDO modprobe can 2>/dev/null || true
        $SUDO modprobe can_raw 2>/dev/null || true
    fi
    [ -d /proc/net/can ] || die "El kernel de este equipo no tiene soporte de CAN (SocketCAN): no se pudo cargar el módulo 'can'.
       Comprueba que es un Linux nativo con los módulos can, can_raw y el del adaptador (p. ej. gs_usb)."

    ejecutar() { echo "+ $*"; $SUDO "$@"; }
fi

if [ "$FD" = 1 ]; then
    echo "Cuerpo $ID: ${INTERFACES[*]} con CAN-FD (bitrate $BITRATE, dbitrate $DBITRATE)"
else
    echo "Cuerpo $ID: ${INTERFACES[*]} con CAN clásico (bitrate $BITRATE)"
fi

for interfaz in "${INTERFACES[@]}"; do
    if [ "$SIMULAR" = 0 ] && ! ip link show "$interfaz" >/dev/null 2>&1; then
        presentes=$(ip -brief link show type can 2>/dev/null | awk '{print $1}' | tr '\n' ' ')
        die "No existe la interfaz $interfaz. ¿Está conectado y encendido el adaptador USB-CAN de ese brazo?
       Interfaces CAN presentes: ${presentes:-ninguna}. El orden can0, can1... depende del orden de conexión de los adaptadores."
    fi
    ejecutar ip link set "$interfaz" down
    if [ "$FD" = 1 ]; then
        ejecutar ip link set "$interfaz" type can bitrate "$BITRATE" dbitrate "$DBITRATE" fd on
    else
        ejecutar ip link set "$interfaz" type can bitrate "$BITRATE" fd off
    fi
    ejecutar ip link set "$interfaz" up
done

if [ "$SIMULAR" = 0 ]; then
    for interfaz in "${INTERFACES[@]}"; do ip -details -brief link show "$interfaz"; done
    echo "Buses levantados. Lanza el robot con: ./launcher.sh robot $ID"
fi
