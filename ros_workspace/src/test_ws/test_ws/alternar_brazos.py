#!/usr/bin/python3
"""Sube y baja los dos brazos de un cuerpo OpenArm alternándolos, con un desfase configurable.

Cada brazo sube el codo (joint4) a --angulo rad, se queda arriba --espera s y vuelve a bajar.
Los brazos se alternan: el derecho empieza a subir --pausa s después de que el izquierdo
empiece a bajar, y viceversa, hasta que cada brazo ha subido --ciclos veces:
  --pausa 0      uno baja mientras el otro sube (a la vez)
  --pausa 0.5    el otro espera medio segundo antes de subir
  --pausa -0.3   el otro empieza a subir 0,3 s ANTES de que el primero baje (solapados:
                 durante ese rato están los dos arriba)
Con pausa negativa cada brazo pasa menos tiempo abajo (abajo = espera + 2·pausa), así que
la pausa mínima es −espera/2; si se pide menos se ajusta a ese mínimo y se avisa.

Se calcula la trayectoria completa de cada brazo y se envía directamente a su
joint_trajectory_controller (no a MoveIt, que no admite movimientos solapados), con la misma
hora de inicio para los dos. Solo se mueve el codo; el resto de articulaciones se mantiene
en la posición de partida. La velocidad y la aceleración se limitan a --velocidad veces las
máximas del codo de joint_limits.yaml (perfil quíntico: arranca y para sin tirones). Al
terminar, o con Ctrl+C / Detener del HMI, los dos brazos vuelven a la posición de partida.

Registro (dos brazos) en
~/ros_workspace/registros_openarm/<fecha>_<cuerpo>_alternar/ (en el anfitrión:
ros_workspace/registros_openarm/): eventos.log, articulaciones.csv (columnas izq_* y der_*,
con la fase de cada brazo) y motores_<bus>.csv para cada bus.

Uso (dentro del contenedor, con el robot lanzado):
  python3 alternar_brazos.py                                   # 5 ciclos, 30 % de velocidad
  python3 alternar_brazos.py --pausa -0.3 --espera 1           # solapados
  python3 alternar_brazos.py --ciclos 3 --angulo 1.0 --velocidad 0.5 --pausa 0.5
"""

import argparse
import csv
import logging
import math
import shutil
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARTICULACIONES = [f"joint{i}" for i in range(1, 8)]
ARTICULACION_CODO = "joint4"   # La que sube y baja (las demás se quedan como están)
CARPETA_REGISTROS = Path.home() / "ros_workspace" / "registros_openarm"
VELOCIDAD_AVISO = 0.5          # Por encima se avisa: más par (y corriente) y más error de seguimiento
LADOS_ROBOTS = {"izq": "left", "der": "right"}   # Claves de robots.py

# Estado de un motor Damiao: nibble alto del primer byte de su trama de respuesta
ESTADOS_MOTOR = {
    0x0: "deshabilitado",
    0x1: "habilitado",
    0x8: "sobretension",
    0x9: "subtension",
    0xA: "sobrecorriente",
    0xB: "sobretemperatura_MOS",
    0xC: "sobretemperatura_bobina",
    0xD: "perdida_comunicacion",
    0xE: "sobrecarga",
}


# --- Registro: log, estado de los motores CAN y articulaciones ------------------------------

def crear_log(carpeta: Path) -> logging.Logger:
    """Log en pantalla y en <carpeta>/eventos.log."""
    log = logging.getLogger("alternar_brazos")
    log.setLevel(logging.INFO)
    formato = logging.Formatter("%(asctime)s.%(msecs)03d %(levelname)-7s %(message)s", "%H:%M:%S")
    for manejador in (logging.FileHandler(carpeta / "eventos.log", encoding="utf-8"),
                      logging.StreamHandler()):
        manejador.setFormatter(formato)
        log.addHandler(manejador)
    return log


def interfaz_can(cuerpo: str, lado: str):
    """Bus CAN del brazo según robots.py (o None si no se puede leer)."""
    try:
        from launcher_robots_lab_robotica.robots import ROBOTS_OPENARM
        return ROBOTS_OPENARM[cuerpo][f"{LADOS_ROBOTS[lado]}_can_interface"]
    except (ImportError, KeyError):
        return None


class MonitorCAN:
    """Lee con candump las respuestas de los motores de un bus y anota cada cambio de estado
    en motores_<bus>.csv (habilitado, deshabilitado o código de error)."""

    def __init__(self, interfaz: str, ruta_csv: Path, log: logging.Logger) -> None:
        self.interfaz = interfaz
        self.log = log
        self.estados = {}          # id motor -> código de estado
        self.cambios = 0
        self.tramas = 0
        self.proceso = None
        self.fichero = open(ruta_csv, "w", newline="", encoding="utf-8")
        self.csv = csv.writer(self.fichero)
        self.csv.writerow(["t_unix", "bus", "motor", "codigo", "estado", "estado_anterior"])

    def arrancar(self) -> None:
        if shutil.which("candump") is None:
            self.log.warning("No está candump (paquete can-utils): no se registran los motores")
            return
        if not Path(f"/sys/class/net/{self.interfaz}").exists():
            self.log.warning(f"No existe {self.interfaz} (¿simulación?): no se registran los motores")
            return
        self.proceso = subprocess.Popen(
            ["candump", "-t", "a", self.interfaz],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        threading.Thread(target=self._leer, daemon=True).start()
        self.log.info(f"Registrando los motores de {self.interfaz} con candump")

    def _leer(self) -> None:
        # Línea de candump -t a: "(1794941292.290039)  can0  011   [08]  11 7F E8 ..."
        for linea in self.proceso.stdout:
            partes = linea.split()
            if len(partes) < 5:
                continue
            try:
                id_trama = int(partes[2], 16)
                primer_byte = int(partes[4], 16)
                t = float(partes[0].strip("()"))
            except ValueError:
                continue
            if not 0x11 <= id_trama <= 0x18:   # Solo respuestas de los motores 1-8
                continue
            self.tramas += 1
            motor = id_trama - 0x10
            codigo = primer_byte >> 4
            anterior = self.estados.get(motor)
            if codigo == anterior:
                continue
            self.estados[motor] = codigo
            nombre = ESTADOS_MOTOR.get(codigo, f"desconocido_{codigo:X}")
            nombre_anterior = "-" if anterior is None else ESTADOS_MOTOR.get(anterior, f"{anterior:X}")
            self.csv.writerow([f"{t:.6f}", self.interfaz, motor, f"{codigo:X}", nombre, nombre_anterior])
            self.fichero.flush()
            if anterior is not None:
                self.cambios += 1
                self.log.warning(f"Motor {motor} de {self.interfaz}: {nombre_anterior} -> {nombre}")

    def parar(self) -> None:
        if self.proceso is not None:
            self.proceso.terminate()
            try:
                self.proceso.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proceso.kill()
            if self.tramas == 0:
                self.log.warning(f"No se ha recibido ninguna respuesta de los motores en {self.interfaz}")
            elif self.cambios:
                self.log.warning(f"{self.cambios} cambios de estado de los motores de {self.interfaz} "
                                 f"(ver motores_{self.interfaz}.csv)")
            else:
                estados = ", ".join(f"{m}={ESTADOS_MOTOR.get(c, c)}" for m, c in sorted(self.estados.items()))
                self.log.info(f"Motores de {self.interfaz} sin cambios de estado ({estados})")
        self.fichero.close()


class RegistroBrazos(Node):
    """Nodo que lee /joint_states y el estado de los controladores de trayectoria de los
    brazos y lo guarda en articulaciones.csv (consigna, real, error, velocidad y par de cada
    articulación, con la fase actual). Sirve también de nodo para los clientes de acción."""

    def __init__(self, cuerpo: str, lados: list, carpeta: Path, log: logging.Logger,
                 hz_registro: float) -> None:
        super().__init__("alternar_brazos")
        self.log = log
        self.lados = list(lados)
        self.brazos = [f"{cuerpo}_{lado}" for lado in self.lados]
        # Articulaciones en el orden de posiciones(): las 7 de cada brazo, uno detrás de otro
        self.articulaciones = [f"{b}_{j}" for b in self.brazos for j in ARTICULACIONES]
        self.cerrojo = threading.Lock()
        self.joint_states = {}      # nombre -> (posición, velocidad, par)
        self.controlador = {}       # nombre -> (consigna, real, error)
        self.error_maximo = {j: 0.0 for j in self.articulaciones}
        self.fase = "inicio"

        self.fichero = open(carpeta / "articulaciones.csv", "w", newline="", encoding="utf-8")
        self.csv = csv.writer(self.fichero)
        cabecera = ["t_unix", "fase"]
        for lado in self.lados:
            for j in ARTICULACIONES:
                cabecera += [f"{lado}_{j}_{c}" for c in ("consigna", "real", "error", "vel", "par")]
        self.csv.writerow(cabecera)

        self.create_subscription(JointState, "/joint_states", self._cb_joint_states, 50)
        for brazo in self.brazos:
            self.create_subscription(
                JointTrajectoryControllerState,
                f"/robot_{brazo}_joint_trajectory_controller/controller_state",
                self._cb_controlador, 50)
        self.create_timer(1.0 / hz_registro, self._registrar)

        self.ejecutor = MultiThreadedExecutor()
        self.ejecutor.add_node(self)
        self.hilo = threading.Thread(target=self.ejecutor.spin, daemon=True)
        self.hilo.start()

        self.log.info("Esperando a /joint_states de los brazos...")
        while not all(j in self.joint_states for j in self.articulaciones):
            time.sleep(0.05)

    def _cb_joint_states(self, msg: JointState) -> None:
        with self.cerrojo:
            for i, nombre in enumerate(msg.name):
                vel = msg.velocity[i] if i < len(msg.velocity) else float("nan")
                par = msg.effort[i] if i < len(msg.effort) else float("nan")
                self.joint_states[nombre] = (msg.position[i], vel, par)

    def _cb_controlador(self, msg: JointTrajectoryControllerState) -> None:
        with self.cerrojo:
            for i, nombre in enumerate(msg.joint_names):
                if nombre not in self.error_maximo or i >= len(msg.error.positions):
                    continue
                error = msg.error.positions[i]
                self.controlador[nombre] = (msg.reference.positions[i], msg.feedback.positions[i], error)
                self.error_maximo[nombre] = max(self.error_maximo[nombre], abs(error))

    def _registrar(self) -> None:
        nan = float("nan")
        with self.cerrojo:
            if self.fichero.closed:
                return
            fila = [f"{time.time():.4f}", self.fase]
            for j in self.articulaciones:
                consigna, real, error = self.controlador.get(j, (nan, nan, nan))
                _, vel, par = self.joint_states.get(j, (nan, nan, nan))
                fila += [f"{v:.5f}" for v in (consigna, real, error, vel, par)]
            self.csv.writerow(fila)
            self.fichero.flush()

    def posiciones(self) -> list:
        with self.cerrojo:
            return [self.joint_states[j][0] for j in self.articulaciones]

    def esperar(self, segundos: float, fase: str) -> None:
        self.fase = fase
        time.sleep(segundos)

    def resumen(self) -> None:
        for brazo in self.brazos:
            self.log.info(f"Error máximo de seguimiento de {brazo} (rad): " + ", ".join(
                f"{j}={self.error_maximo[f'{brazo}_{j}']:.3f}" for j in ARTICULACIONES))

    def cerrar(self) -> None:
        self.ejecutor.shutdown()
        self.hilo.join()
        with self.cerrojo:
            self.fichero.close()
        self.destroy_node()


def fraccion(texto: str) -> float:
    """Tipo de argparse para --velocidad: número en (0, 1]."""
    valor = float(texto)
    if not 0.0 < valor <= 1.0:
        raise argparse.ArgumentTypeError("debe estar entre 0 (excluido) y 1")
    return valor


# --- Trayectorias ---------------------------------------------------------------------------

LADOS = ["izq", "der"]   # Orden de las posiciones de RegistroBrazos: 7 del izq y 7 del der
VELOCIDAD = 0.3          # Fracción de la velocidad y aceleración máximas por defecto
CODO = ARTICULACIONES.index(ARTICULACION_CODO)
# Límites del codo si no se puede leer joint_limits.yaml (los de openarm_v2.0)
VEL_MAX_CODO = 5.445426  # rad/s
ACC_MAX_CODO = 5.0       # rad/s²
ARRANQUE = 0.5           # s desde el envío hasta el inicio (para que los dos empiecen a la vez)


def limites_codo():
    """(velocidad, aceleración) máximas del codo según joint_limits.yaml de MoveIt."""
    try:
        from ament_index_python.packages import get_package_share_directory
        ruta = (Path(get_package_share_directory("launcher_robots_lab_robotica"))
                / "openarm" / "moveit" / "joint_limits.yaml")
        lim = yaml.safe_load(ruta.read_text())["joint_limits"][f"openarm_left_{ARTICULACION_CODO}"]
        return float(lim["max_velocity"]), float(lim["max_acceleration"])
    except Exception:  # noqa: BLE001 - si falta el paquete se usan los valores conocidos
        return VEL_MAX_CODO, ACC_MAX_CODO


def duracion_movimiento(distancia: float, vel: float, acc: float) -> float:
    """Duración de un movimiento quíntico (velocidad y aceleración nulas en los extremos)
    que no supera vel ni acc: pico de velocidad 1,875·d/T y de aceleración 5,774·d/T²."""
    d = abs(distancia)
    if d < 1e-6:
        return 0.5
    return max(1.875 * d / vel, math.sqrt(5.774 * d / acc), 0.5)


def calendario(ciclos: int, t_mov: float, espera: float, pausa: float):
    """Momentos de inicio de cada subida y bajada de cada brazo (s desde el arranque).

    Izquierdo: sube en k·C y baja en k·C + T + espera. Derecho: sube `pausa` s después de
    que baje el izquierdo y baja T + espera después. Periodo C = 2·(T + espera + pausa).
    """
    periodo = 2 * (t_mov + espera + pausa)
    eventos = {"izq": [], "der": []}
    for k in range(ciclos):
        sube_izq = k * periodo
        baja_izq = sube_izq + t_mov + espera
        sube_der = baja_izq + pausa
        baja_der = sube_der + t_mov + espera
        eventos["izq"].append((sube_izq, baja_izq))
        eventos["der"].append((sube_der, baja_der))
    return eventos


def trayectoria(nombres, partida, arriba, eventos, t_mov) -> JointTrajectory:
    """JointTrajectory de un brazo: partida, y para cada (sube, baja) los puntos de inicio y
    fin de cada movimiento del codo (velocidad y aceleración 0 en cada punto: quíntico)."""
    abajo_codo = partida[CODO]
    claves = [(0.0, abajo_codo)]
    for sube, baja in eventos:
        claves += [(sube, abajo_codo), (sube + t_mov, arriba), (baja, arriba), (baja + t_mov, abajo_codo)]
    # Puntos con tiempos estrictamente crecientes (si la espera es 0, la llegada arriba y el
    # inicio de la bajada coinciden: se deja uno)
    puntos = []
    for t, codo in claves:
        t += ARRANQUE
        if puntos and t <= puntos[-1][0] + 1e-3:
            puntos[-1] = (puntos[-1][0], codo)
            continue
        puntos.append((t, codo))
    msg = JointTrajectory()
    msg.joint_names = list(nombres)
    for t, codo in puntos:
        posiciones = list(partida)
        posiciones[CODO] = codo
        punto = JointTrajectoryPoint()
        punto.positions = [float(p) for p in posiciones]
        punto.velocities = [0.0] * len(posiciones)
        punto.accelerations = [0.0] * len(posiciones)
        segundos = int(t)
        punto.time_from_start = Duration(sec=segundos, nanosec=int((t - segundos) * 1e9))
        msg.points.append(punto)
    return msg


def fase_en(t: float, eventos, t_mov: float) -> str:
    """Fase de cada brazo en el instante t (para la columna fase del CSV)."""
    partes = []
    for lado in LADOS:
        estado = "abajo"
        for k, (sube, baja) in enumerate(eventos[lado], start=1):
            if sube <= t < sube + t_mov:
                estado = f"sube{k}"
            elif sube + t_mov <= t < baja:
                estado = f"arriba{k}"
            elif baja <= t < baja + t_mov:
                estado = f"baja{k}"
        partes.append(f"{lado}_{estado}")
    return "_".join(partes)


def enviar(clientes, trayectorias, log, inicio_ros):
    """Envía las trayectorias a la vez (misma hora de inicio). Devuelve los manejadores."""
    envios = []
    for lado in LADOS:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trayectorias[lado]
        goal.trajectory.header.stamp = inicio_ros
        envios.append((lado, clientes[lado].send_goal_async(goal)))
    manejadores = {}
    for lado, envio in envios:
        while not envio.done():
            time.sleep(0.01)
        manejador = envio.result()
        if not manejador.accepted:
            log.error(f"El controlador del brazo {lado} ha rechazado la trayectoria")
        manejadores[lado] = manejador
    return manejadores


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Alterna los dos brazos OpenArm con un desfase configurable (puede ser negativo)")
    parser.add_argument("--cuerpo", default="301", help="Cuerpo OpenArm (por defecto 301)")
    parser.add_argument("--ciclos", type=int, default=5,
                        help="Veces que sube cada brazo (por defecto 5)")
    parser.add_argument("--angulo", type=float, default=1.5,
                        help=f"Posición de {ARTICULACION_CODO} arriba, en rad (por defecto 1.5)")
    parser.add_argument("--espera", type=float, default=0.5,
                        help="Tiempo de cada brazo arriba, en s (por defecto 0.5)")
    parser.add_argument("--pausa", type=float, default=0.0,
                        help="Desde que un brazo empieza a bajar hasta que el otro empieza a "
                             "subir, en s; negativa = el otro sube antes (por defecto 0)")
    parser.add_argument("--hz", type=float, default=50.0,
                        help="Frecuencia de registro de articulaciones.csv (por defecto 50)")
    parser.add_argument("--velocidad", type=fraccion, default=VELOCIDAD,
                        help="Fracción de la velocidad y aceleración máximas del codo, entre 0 y 1 "
                             f"(por defecto {VELOCIDAD})")
    args = parser.parse_args()
    if args.ciclos < 1 or args.espera < 0:
        parser.error("--ciclos debe ser ≥ 1 y --espera ≥ 0")

    marca = datetime.now().strftime("%Y%m%d_%H%M%S")
    carpeta = CARPETA_REGISTROS / f"{marca}_{args.cuerpo}_alternar"
    carpeta.mkdir(parents=True, exist_ok=True)
    log = crear_log(carpeta)
    log.info(f"Registro en {carpeta}")

    pausa = args.pausa
    if pausa < -args.espera / 2:
        pausa = -args.espera / 2
        log.warning(f"Pausa {args.pausa} s demasiado negativa: cada brazo tendría que volver a "
                    f"subir antes de terminar de bajar. Se usa la mínima, {pausa:g} s "
                    "(−espera/2); para solapar más, sube --espera")

    vel_max, acc_max = limites_codo()
    monitores = []
    for lado in LADOS:
        bus = interfaz_can(args.cuerpo, lado)
        if bus:
            monitores.append(MonitorCAN(bus, carpeta / f"motores_{bus}.csv", log))
            monitores[-1].arrancar()
    rclpy.init()
    nodo = None
    try:
        nodo = RegistroBrazos(args.cuerpo, LADOS, carpeta, log, args.hz)
        partida = nodo.posiciones()
        n = len(ARTICULACIONES)
        partidas = {lado: partida[k * n:(k + 1) * n] for k, lado in enumerate(LADOS)}
        nombres = {lado: nodo.articulaciones[k * n:(k + 1) * n] for k, lado in enumerate(LADOS)}

        distancia = max(abs(args.angulo - partidas[lado][CODO]) for lado in LADOS)
        t_mov = duracion_movimiento(distancia, vel_max * args.velocidad, acc_max * args.velocidad)
        eventos = calendario(args.ciclos, t_mov, args.espera, pausa)
        total = eventos["der"][-1][1] + t_mov
        log.info(f"Cuerpo {args.cuerpo}: {args.ciclos} ciclos, {ARTICULACION_CODO} a {args.angulo} "
                 f"rad, cada subida/bajada {t_mov:.2f} s, arriba {args.espera:g} s, pausa entre "
                 f"brazos {pausa:g} s, velocidad {args.velocidad:.0%}; duración {total:.1f} s")
        if args.velocidad > VELOCIDAD_AVISO:
            log.warning(f"Velocidad {args.velocidad:.0%}: por encima del {VELOCIDAD_AVISO:.0%} los "
                        "motores piden más corriente y el error de seguimiento crece")
        if pausa < 0:
            log.info(f"Solapados: cada brazo empieza a subir {-pausa:g} s antes de que baje el otro")

        clientes = {lado: ActionClient(
            nodo, FollowJointTrajectory,
            f"/robot_{args.cuerpo}_{lado}_joint_trajectory_controller/follow_joint_trajectory")
            for lado in LADOS}
        for lado, cliente in clientes.items():
            if not cliente.wait_for_server(timeout_sec=10.0):
                raise RuntimeError(f"No responde el controlador de trayectoria del brazo {lado}")

        trayectorias = {lado: trayectoria(nombres[lado], partidas[lado], args.angulo,
                                          eventos[lado], t_mov) for lado in LADOS}
        inicio = time.time() + 0.3
        inicio_ros = rclpy.time.Time(nanoseconds=int(inicio * 1e9),
                                     clock_type=nodo.get_clock().clock_type).to_msg()
        manejadores = {}
        try:
            manejadores = enviar(clientes, trayectorias, log, inicio_ros)
            resultados = {lado: m.get_result_async() for lado, m in manejadores.items() if m.accepted}
            fin = inicio + ARRANQUE + total + 5.0
            while not all(r.done() for r in resultados.values()) and time.time() < fin:
                nodo.fase = fase_en(time.time() - inicio - ARRANQUE, eventos, t_mov)
                time.sleep(0.02)
            for lado, r in resultados.items():
                if not r.done():
                    log.error(f"Brazo {lado}: la trayectoria no ha terminado a tiempo")
                    continue
                res = r.result().result
                if res.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                    log.info(f"Brazo {lado}: secuencia completada")
                else:
                    log.error(f"Brazo {lado}: error {res.error_code} {res.error_string}")
        except KeyboardInterrupt:
            log.warning("Interrumpido (Ctrl+C / Detener): se para y se vuelve a la posición de partida")
            for m in manejadores.values():
                if m.accepted:
                    m.cancel_goal_async()
        finally:
            # Vuelta a la posición de partida desde donde esté cada brazo (si ya están, es
            # un movimiento nulo)
            nodo.fase = "vuelta_partida"
            actuales = nodo.posiciones()
            vueltas = {}
            for k, lado in enumerate(LADOS):
                d = abs(actuales[k * n + CODO] - partidas[lado][CODO])
                if d < 0.02:
                    continue
                t = duracion_movimiento(d, vel_max * args.velocidad, acc_max * args.velocidad)
                vuelta = trayectoria(nombres[lado], partidas[lado], partidas[lado][CODO], [], t)
                punto = vuelta.points[0]
                punto.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
                vueltas[lado] = vuelta
            if vueltas:
                log.info("Volviendo a la posición de partida: " + ", ".join(vueltas))
                envios = []
                for lado, vuelta in vueltas.items():
                    goal = FollowJointTrajectory.Goal()
                    goal.trajectory = vuelta
                    envios.append(clientes[lado].send_goal_async(goal))
                limite = time.time() + 10.0
                while time.time() < limite and not all(e.done() for e in envios):
                    time.sleep(0.02)
                esperas = [e.result().get_result_async() for e in envios
                           if e.done() and e.result().accepted]
                while time.time() < limite and not all(r.done() for r in esperas):
                    time.sleep(0.02)
            nodo.esperar(1.0, "final")
        nodo.resumen()
    finally:
        for monitor in monitores:
            monitor.parar()
        if nodo is not None:
            nodo.cerrar()
        rclpy.shutdown()
        log.info(f"Registro guardado en {carpeta}")


if __name__ == "__main__":
    main()
