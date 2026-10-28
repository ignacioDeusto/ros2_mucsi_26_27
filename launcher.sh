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
    [ -n "$SERVICIO" ] || die "No hay ningún contenedor en marcha. Ejecuta primero: ./launcher.sh iniciar"
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
    local servicio=desktop
    case "${1:-}" in
        "") ;;
        --lab) servicio=local_laboratorio ;;
        --lab-gpu) servicio=local_laboratorio_gpu ;;
        *) die "Opción desconocida: $1 (usa --lab o --lab-gpu)" ;;
    esac
    comprobar_docker
    if [ "$servicio" != desktop ]; then
        [ "$(uname -s)" = Linux ] || die "Los perfiles de laboratorio solo funcionan en Linux con X11."
        xhost +local: >/dev/null 2>&1 || true
        local perfil=local; [ "$servicio" = local_laboratorio_gpu ] && perfil=local_gpu
        export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" DISPLAY="${DISPLAY:-:0}"
        COMPOSE=(docker compose --profile "$perfil")
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
    [[ "$id" =~ ^(20[3-6]|203-204|205-206)$ ]] || die "Indica el robot: 203, 204, 205, 206 o una pareja (203-204, 205-206)"
    id="${id/-/_}"
    comprobar_docker
    servicio_activo; local s=$SERVICIO
    "${COMPOSE[@]}" exec -it -u laboratorio "$s" bash -c "$(ros_cmd "ros2 launch launcher_robots_lab_robotica ${tipo}_${id}.launch.py")"
}

cmd_ayuda() {
    cat <<'EOF'
Uso: ./launcher.sh <comando>

  instalar-docker        Instala Docker si no lo tienes
  iniciar [--lab|--lab-gpu]
                         Construye (la primera vez) y arranca el entorno y abre el escritorio en el navegador.
                         --lab / --lab-gpu: perfiles del laboratorio (solo Linux con X11)
  sim <203-206|203-204|205-206>
                         Lanza la simulación de un robot o de una pareja (Ctrl+C para detenerla)
  robot <203-206|203-204|205-206>
                         Lanza el robot real (o la pareja, sin probar)
  terminal               Abre una terminal dentro del contenedor
  compilar               Recompila el espacio de trabajo (colcon build)
  parar                  Para el contenedor (conserva su contenido)
EOF
}

case "${1:-ayuda}" in
    instalar-docker) cmd_instalar_docker ;;
    iniciar) shift; cmd_iniciar "$@" ;;
    sim) cmd_lanzar sim "${2:-}" ;;
    robot) cmd_lanzar robot "${2:-}" ;;
    terminal) cmd_terminal ;;
    compilar) cmd_compilar ;;
    parar) cmd_parar ;;
    ayuda|-h|--help) cmd_ayuda ;;
    *) cmd_ayuda; exit 1 ;;
esac
