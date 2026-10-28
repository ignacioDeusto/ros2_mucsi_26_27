#!/usr/bin/env bash
# Utilidad del entorno de Robótica y Automatización Inteligente (Linux, macOS y WSL).
# Uso: ./launcher.sh <comando> [opciones]     (./launcher.sh ayuda para ver los comandos)
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

PUERTO=6081
COMPOSE=(docker compose)

die() { echo "ERROR: $*" >&2; exit 1; }

comprobar_docker() {
    command -v docker >/dev/null 2>&1 || die "Docker no está instalado. Ejecuta: ./launcher.sh instalar-docker"
    docker info >/dev/null 2>&1 || die "Docker está instalado pero no está en ejecución. Abre Docker Desktop (o inicia el servicio) y vuelve a intentarlo."
    docker compose version >/dev/null 2>&1 || die "Falta 'docker compose' (v2). Actualiza Docker."
}

servicio_activo() {
    SERVICIO=$("${COMPOSE[@]}" ps --services --status running 2>/dev/null | head -n1 || true)
    if [ -z "$SERVICIO" ] && [ "$(uname -s)" = Linux ]; then
        COMPOSE=(docker compose --profile local --profile local_gpu)
        SERVICIO=$("${COMPOSE[@]}" ps --services --status running 2>/dev/null | head -n1 || true)
    fi
    [ -n "$SERVICIO" ] || die "No hay ningún contenedor en marcha. Ejecuta primero: ./launcher.sh desktop"
}

ros_cmd() {
    echo "source /opt/ros/jazzy/setup.bash; [ -f ~/ros_workspace/install/setup.bash ] && source ~/ros_workspace/install/setup.bash; $1"
}

abrir_navegador() {
    local url="http://localhost:${PUERTO}/vnc.html?autoconnect=true&password=laboratorio&resize=scale"
    echo "Escritorio: $url"
    if command -v open >/dev/null 2>&1; then open "$url" || true
    elif command -v xdg-open >/dev/null 2>&1; then xdg-open "$url" >/dev/null 2>&1 || true
    fi
}

cmd_instalar_docker() {
    if command -v docker >/dev/null 2>&1; then
        echo "Docker ya está instalado: $(docker --version)"
        return
    fi
    case "$(uname -s)" in
        Linux)
            read -r -p "Se instalará Docker con el script oficial (get.docker.com) usando sudo. ¿Continuar? [s/N] " r
            [[ "$r" =~ ^[sS]$ ]] || exit 0
            curl -fsSL https://get.docker.com | sudo sh
            sudo usermod -aG docker "$USER"
            echo "Docker instalado. Cierra la sesión y vuelve a entrar (o reinicia) para poder usarlo sin sudo."
            ;;
        Darwin)
            if command -v brew >/dev/null 2>&1; then
                read -r -p "Se instalará Docker Desktop con Homebrew. ¿Continuar? [s/N] " r
                [[ "$r" =~ ^[sS]$ ]] || exit 0
                brew install --cask docker
                echo "Abre la aplicación Docker Desktop una vez para completar la instalación."
            else
                echo "Instala Docker Desktop desde https://www.docker.com/products/docker-desktop/ (elige Apple Silicon o Intel según tu Mac)."
            fi
            ;;
        *) die "Sistema no soportado por este script. En Windows usa launcher.bat instalar-docker" ;;
    esac
}

cmd_iniciar() {
    local servicio="$1"
    comprobar_docker
    if [ "$servicio" != desktop ]; then
        [ "$(uname -s)" = Linux ] || die "Los perfiles de laboratorio solo funcionan en Linux con X11."
        xhost +local: >/dev/null 2>&1 || true
        # No hace falta exportar XAUTHORITY: docker-compose.yml usa el fallback
        # ${XAUTHORITY:-/dev/null} y el acceso al servidor X lo autoriza xhost.
        export DISPLAY="${DISPLAY:-:0}"
        COMPOSE=(docker compose --profile "$servicio")
        if [ "$servicio" = local_gpu ]; then
            local ficheros=()
            # Gráfica NVIDIA: inyectar las librerías del driver con el toolkit.
            # En WSL2 Docker Desktop gestiona la GPU sin toolkit.
            if command -v nvidia-smi >/dev/null 2>&1; then
                if grep -qi microsoft /proc/version || command -v nvidia-ctk >/dev/null 2>&1 || command -v nvidia-container-runtime >/dev/null 2>&1; then
                    ficheros+=( docker-compose.nvidia.yml )
                    echo "GPU NVIDIA detectada: aplicando docker-compose.nvidia.yml"
                else
                    echo "AVISO: hay drivers de NVIDIA pero falta nvidia-container-toolkit; RViz usará renderizado por software. Instálalo (docs.nvidia.com/datacenter/cloud-native/container-toolkit) o usa ./launcher.sh local"
                fi
            fi
            # WSL2: la GPU se accede por /dev/dxg con el driver d3d12 de Mesa,
            # que necesita las librerías de DirectX de WSLg.
            if grep -qi microsoft /proc/version; then
                ficheros+=( docker-compose.wsl.yml )
                echo "WSL2 detectado: aplicando docker-compose.wsl.yml"
            elif [ -d /dev/dri ]; then
                echo "Gráfica AMD/Intel detectada vía /dev/dri (Mesa del contenedor)"
            fi
            if [ "${#ficheros[@]}" -gt 0 ]; then
                COMPOSE+=( -f docker-compose.yml )
                for f in "${ficheros[@]}"; do
                    COMPOSE+=( -f "$f" )
                done
            fi
        fi
    fi
    "${COMPOSE[@]}" up -d --build "$servicio"
    if ! "${COMPOSE[@]}" exec -T -u laboratorio "$servicio" test -f /home/laboratorio/ros_workspace/install/setup.bash; then
        echo "Primera ejecución: compilando el espacio de trabajo..."
        cmd_compilar
    fi
    if [ "$servicio" = desktop ]; then
        echo -n "Esperando al escritorio"
        for _ in $(seq 1 60); do
            curl -fs -o /dev/null "http://localhost:${PUERTO}" && break
            echo -n "."; sleep 1
        done
        echo
        abrir_navegador
    fi
    echo "Listo. Lanza una simulación con: ./launcher.sh sim 203"
}

cmd_compilar() {
    comprobar_docker
    servicio_activo; local s=$SERVICIO
    "${COMPOSE[@]}" exec -T -u laboratorio "$s" bash -lc "source /opt/ros/jazzy/setup.bash; cd ~/ros_workspace && colcon build --symlink-install"
}

cmd_parar() {
    comprobar_docker
    "${COMPOSE[@]}" stop
    if [ "$(uname -s)" = Linux ]; then docker compose --profile local --profile local_gpu stop 2>/dev/null || true; fi
}

cmd_terminal() {
    comprobar_docker
    servicio_activo; local s=$SERVICIO
    "${COMPOSE[@]}" exec -u laboratorio "$s" bash
}

cmd_lanzar() {
    local tipo="$1" id="${2:-}"
    if [ "$tipo" = sim ]; then
        [[ "$id" =~ ^(20[3-6]|301|302|203-204|205-206)$ ]] || die "Indica el robot: 203, 204, 205, 206, 301, 302 o una pareja UR (203-204, 205-206)"
    else
        [[ "$id" =~ ^(20[3-6]|203-204|205-206)$ ]] || die "Indica el robot UR: 203, 204, 205, 206 o una pareja (203-204, 205-206). Los OpenArm (301, 302) todavía no están conectados al hardware real"
    fi
    id="${id/-/_}"
    comprobar_docker
    servicio_activo; local s=$SERVICIO
    "${COMPOSE[@]}" exec -it -u laboratorio "$s" bash -c "$(ros_cmd "ros2 launch launcher_robots_lab_robotica ${tipo}_${id}.launch.py")"
}

cmd_ayuda() {
    cat <<'EOF'
Uso: ./launcher.sh <comando>

  instalar-docker        Instala Docker si no lo tienes
  desktop                Arranca el entorno de escritorio y lo abre en el navegador (noVNC)
  local                  Arranca el entorno del laboratorio en la pantalla (solo Linux con X11)
  local_gpu              Igual que local, con aceleración gráfica (NVIDIA, AMD/Intel vía Mesa o WSL2 vía DirectX; solo Linux con X11)
  sim <203-206|301|302|203-204|205-206>
                         Lanza la simulación de un UR, de una pareja de UR o de un cuerpo OpenArm (Ctrl+C para detenerla)
  robot <203-206|203-204|205-206>
                         Lanza el robot real UR (o la pareja, sin probar); los OpenArm aún no tienen robot real
  terminal               Abre una terminal dentro del contenedor
  compilar               Recompila el espacio de trabajo (colcon build)
  parar                  Para el contenedor (conserva su contenido)
EOF
}

case "${1:-ayuda}" in
    instalar-docker) cmd_instalar_docker ;;
    desktop|local|local_gpu) cmd_iniciar "$1" ;;
    sim) cmd_lanzar sim "${2:-}" ;;
    robot) cmd_lanzar robot "${2:-}" ;;
    terminal) cmd_terminal ;;
    compilar) cmd_compilar ;;
    parar) cmd_parar ;;
    ayuda|-h|--help) cmd_ayuda ;;
    *) cmd_ayuda; exit 1 ;;
esac
