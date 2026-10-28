# Utilidad del entorno de Robótica y Automatización Inteligente (Windows).
# Uso: .\launcher.ps1 <comando> [opciones]     (.\launcher.ps1 ayuda para ver los comandos)
param(
    [Parameter(Position = 0)][string]$Comando = "ayuda",
    [Parameter(Position = 1)][string]$Opcion = ""
)

$ErrorActionPreference = "Continue"
Set-Location $PSScriptRoot
$Puerto = 6081
$Compose = @("compose")

function Fallo($msg) { Write-Host "ERROR: $msg" -ForegroundColor Red; exit 1 }

function Comprobar-Docker {
    if (-not (Get-Command docker -ErrorAction SilentlyContinue)) { Fallo "Docker no está instalado. Ejecuta: .\launcher.bat instalar-docker" }
    docker info *> $null
    if ($LASTEXITCODE -ne 0) { Fallo "Docker no está en ejecución. Abre Docker Desktop, espera a que arranque y vuelve a intentarlo." }
}

function Servicio-Activo {
    $s = (docker @Compose ps --services --status running | Select-Object -First 1)
    if (-not $s) { Fallo "No hay ningún contenedor en marcha. Ejecuta primero: .\launcher.bat desktop" }
    return $s
}

function Cmd-Ros($c) {
    return "source /opt/ros/jazzy/setup.bash; [ -f ~/ros_workspace/install/setup.bash ] && source ~/ros_workspace/install/setup.bash; $c"
}

function Instalar-Docker {
    if (Get-Command docker -ErrorAction SilentlyContinue) { Write-Host "Docker ya está instalado: $(docker --version)"; return }
    if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
        Fallo "winget no está disponible. Instala Docker Desktop desde https://www.docker.com/products/docker-desktop/"
    }
    $r = Read-Host "Se instalará Docker Desktop con winget. ¿Continuar? [s/N]"
    if ($r -notmatch '^[sS]$') { return }
    winget install -e --id Docker.DockerDesktop
    Write-Host "Instalado. Reinicia el equipo si Docker Desktop lo solicita (necesita WSL 2; si no lo tienes ejecuta 'wsl --install' como administrador) y abre Docker Desktop una vez."
}

function Compilar {
    Comprobar-Docker
    $s = Servicio-Activo
    docker @Compose exec -T -u laboratorio $s bash -lc "source /opt/ros/jazzy/setup.bash; cd ~/ros_workspace && colcon build --symlink-install"
    if ($LASTEXITCODE -ne 0) { Fallo "Falló la compilación." }
}

function Iniciar($servicio) {
    if ($servicio -ne "desktop") { Fallo "Los perfiles de laboratorio solo funcionan en Linux con X11." }
    Comprobar-Docker
    docker @Compose up -d --build $servicio
    if ($LASTEXITCODE -ne 0) { Fallo "No se pudo arrancar el contenedor." }
    docker @Compose exec -T -u laboratorio $servicio test -f /home/laboratorio/ros_workspace/install/setup.bash
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Primera ejecución: compilando el espacio de trabajo..."
        Compilar
    }
    Write-Host -NoNewline "Esperando al escritorio"
    for ($i = 0; $i -lt 60; $i++) {
        try { Invoke-WebRequest "http://localhost:$Puerto" -UseBasicParsing -TimeoutSec 2 | Out-Null; break }
        catch { Write-Host -NoNewline "."; Start-Sleep 1 }
    }
    Write-Host ""
    $url = "http://localhost:$Puerto/vnc.html?autoconnect=true&password=laboratorio&resize=scale"
    Write-Host "Escritorio: $url"
    Start-Process $url
    Write-Host "Listo. Lanza una simulación con: .\launcher.bat sim 203"
}

function Lanzar($tipo) {
    if ($tipo -eq "sim") {
        if ($Opcion -notmatch '^(20[3-6]|301|302|203-204|205-206)$') { Fallo "Indica el robot: 203, 204, 205, 206, 301, 302 o una pareja UR (203-204, 205-206)" }
    } else {
        if ($Opcion -notmatch '^(20[3-6]|203-204|205-206)$') { Fallo "Indica el robot UR: 203, 204, 205, 206 o una pareja (203-204, 205-206). Los OpenArm (301, 302) todavía no están conectados al hardware real" }
    }
    $id = $Opcion -replace '-', '_'
    Comprobar-Docker
    $s = Servicio-Activo
    docker @Compose exec -it -u laboratorio $s bash -c (Cmd-Ros "ros2 launch launcher_robots_lab_robotica ${tipo}_$id.launch.py")
}

function Ayuda {
    @"
Uso: .\launcher.bat <comando>

  instalar-docker        Instala Docker Desktop si no lo tienes
  desktop                Construye (la primera vez) y arranca el entorno y abre el escritorio en el navegador
  sim <203-206|301|302|203-204|205-206>
                         Lanza la simulación de un UR, de una pareja de UR o de un cuerpo OpenArm (Ctrl+C para detenerla)
  robot <203-206|203-204|205-206>
                         Lanza el robot real UR (o la pareja, sin probar); los OpenArm aún no tienen robot real
  terminal               Abre una terminal dentro del contenedor
  compilar               Recompila el espacio de trabajo (colcon build)
  parar                  Para el contenedor (conserva su contenido)

  Los entornos local y local_gpu (pantalla del laboratorio) solo existen en Linux con X11.
"@ | Write-Host
}

switch ($Comando) {
    "instalar-docker" { Instalar-Docker }
    { $_ -in "desktop", "local", "local_gpu" } { Iniciar $_ }
    "sim" { Lanzar "sim" }
    "robot" { Lanzar "robot" }
    "terminal" { Comprobar-Docker; $s = Servicio-Activo; docker @Compose exec -u laboratorio $s bash }
    "compilar" { Compilar }
    "parar" { Comprobar-Docker; docker @Compose stop }
    { $_ -in "ayuda", "-h", "--help" } { Ayuda }
    default { Ayuda; exit 1 }
}
