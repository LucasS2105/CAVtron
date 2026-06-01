"""
communication/wifi_comm.py
===========================
Servidor de comunicação WiFi para monitoramento e controle remoto.

O RPi Zero 2 W possui WiFi integrado (802.11n), o que permite
telemetria e teleoperação sem fio durante testes e competições.

Arquitetura:
    HTTP/REST  (porta 8080) — Painel de controle e consulta de estado
    WebSocket  (porta 8081) — Telemetria em tempo real (push do RPi → cliente)

Endpoints REST:
    GET  /status          → Estado atual do sistema (JSON)
    GET  /telemetry       → Última leitura de sensores (JSON)
    POST /command         → Envia comando ao robô (JSON body)
    POST /pid             → Atualiza ganhos PID em tempo real (JSON body)
    GET  /stats           → Estatísticas de operação (JSON)
    POST /calibrate       → Dispara calibração do sensor de linha

WebSocket (ws://IP:8081):
    O servidor envia telemetria JSON a cada TELEM_INTERVAL_S segundos.
    O cliente pode enviar comandos JSON em qualquer momento.

Exemplo de payload de telemetria:
    {
        "timestamp": 1234567890.123,
        "state":     "FOLLOW_LINE",
        "line": {
            "position": -0.25,
            "detected": true,
            "lost":     false
        },
        "pid": {
            "output": 18.5,
            "p": 8.0, "i": 0.1, "d": 10.4
        },
        "victim": {
            "detected": false,
            "x": 0.0, "y": 0.0, "area": 0.0
        },
        "motors": {
            "left_duty": 47.8,
            "right_duty": 54.9
        }
    }

Exemplo de payload de comando:
    { "command": "MOVE",  "direction": "FWD", "speed": 150 }
    { "command": "STOP" }
    { "command": "GRIP",  "action": "CAPTURE" }
    { "command": "STATE", "state": 1 }
    { "command": "PID",   "kp": 30.0, "ki": 0.3, "kd": 20.0 }

Segurança:
    Este servidor não implementa autenticação — adequado apenas para
    redes locais isoladas durante desenvolvimento e competição.
    Para uso em redes abertas, adicionar API key nos headers.

Dependências:
    pip install flask flask-sock   (REST + WebSocket)

Uso:
    server = WifiComm(robot_controller)
    server.start()          # Inicia em threads separadas (non-blocking)
    ...
    server.stop()
"""

import json
import time
import logging
import threading
from typing import Optional, TYPE_CHECKING

logger = logging.getLogger(__name__)

# Importação condicional — Flask é opcional
try:
    from flask import Flask, request, jsonify
    from flask_sock import Sock
    FLASK_AVAILABLE = True
except ImportError:
    FLASK_AVAILABLE = False
    logger.warning("[WifiComm] Flask/flask-sock não instalados. WiFi desabilitado.")
    logger.warning("           Instale com: pip install flask flask-sock")

if TYPE_CHECKING:
    from main import RobotController


# ============================================================
#  CONSTANTES
# ============================================================

HTTP_PORT: int  = 8080
WS_PORT: int    = 8081
TELEM_INTERVAL_S: float = 0.1   # Taxa de telemetria WebSocket: 10 Hz


# ============================================================
#  SERVIDOR WIFI
# ============================================================

class WifiComm:
    """
    Servidor HTTP/WebSocket para controle e monitoramento remoto.

    Args:
        robot      : Instância do RobotController para acesso ao estado.
        http_port  : Porta do servidor REST (padrão: 8080).
        ws_port    : Porta do servidor WebSocket (padrão: 8081).
        host       : Interface de escuta ('0.0.0.0' = todas as interfaces).
    """

    def __init__(self,
                 robot: "RobotController",
                 http_port: int = HTTP_PORT,
                 ws_port: int   = WS_PORT,
                 host: str      = '0.0.0.0'):
        self._robot     = robot
        self._http_port = http_port
        self._ws_port   = ws_port
        self._host      = host
        self._running   = False

        # Clientes WebSocket ativos
        self._ws_clients: list = []
        self._ws_lock = threading.Lock()

        # Threads dos servidores
        self._http_thread: Optional[threading.Thread] = None
        self._ws_thread: Optional[threading.Thread]   = None
        self._telem_thread: Optional[threading.Thread] = None

        # Estatísticas
        self._stats = {
            'commands_received': 0,
            'telem_sent':        0,
            'ws_connections':    0,
        }

        if FLASK_AVAILABLE:
            self._app  = Flask(__name__)
            self._sock = Sock(self._app)
            self._setup_routes()
        else:
            self._app = None

    # ----------------------------------------------------------
    #  CICLO DE VIDA
    # ----------------------------------------------------------

    def start(self):
        """
        Inicia os servidores HTTP e WebSocket em threads separadas.

        Não bloqueia — retorna imediatamente após iniciar as threads.
        """
        if not FLASK_AVAILABLE:
            logger.warning("[WifiComm] Flask não disponível. WiFi não iniciado.")
            return

        self._running = True

        # Thread do servidor HTTP/REST + WebSocket (Flask unificado)
        self._http_thread = threading.Thread(
            target=self._run_flask,
            name='WifiCommHTTP',
            daemon=True
        )
        self._http_thread.start()

        # Thread de telemetria periódica via WebSocket
        self._telem_thread = threading.Thread(
            target=self._telemetry_loop,
            name='WifiCommTelemetry',
            daemon=True
        )
        self._telem_thread.start()

        logger.info(
            f"[WifiComm] Servidores iniciados:\n"
            f"  REST : http://{self._host}:{self._http_port}\n"
            f"  WS   : ws://{self._host}:{self._http_port}/ws"
        )

    def stop(self):
        """Encerra os servidores e threads."""
        self._running = False
        logger.info("[WifiComm] Servidor parado.")

    # ----------------------------------------------------------
    #  ROTAS FLASK (REST + WebSocket)
    # ----------------------------------------------------------

    def _setup_routes(self):
        """Registra todas as rotas HTTP e WebSocket no app Flask."""
        app  = self._app
        sock = self._sock
        robot = self._robot

        # ---- GET /status ----
        @app.route('/status', methods=['GET'])
        def get_status():
            """Retorna o estado atual do sistema."""
            return jsonify({
                'state'    : robot._state.name,
                'running'  : robot._running,
                'uptime_s' : time.monotonic() - robot._state_entry_time,
                'timestamp': time.time(),
            })

        # ---- GET /telemetry ----
        @app.route('/telemetry', methods=['GET'])
        def get_telemetry():
            """Retorna a última leitura completa de sensores."""
            return jsonify(self._build_telemetry_payload())

        # ---- GET /stats ----
        @app.route('/stats', methods=['GET'])
        def get_stats():
            """Retorna estatísticas de operação do servidor WiFi."""
            return jsonify({**self._stats, 'timestamp': time.time()})

        # ---- POST /command ----
        @app.route('/command', methods=['POST'])
        def post_command():
            """
            Executa um comando no robô.

            Body JSON:
                { "command": "MOVE",  "direction": "FWD", "speed": 150 }
                { "command": "STOP" }
                { "command": "GRIP",  "action": "OPEN" }
                { "command": "STATE", "state": 1 }
            """
            body = request.get_json(silent=True)
            if not body:
                return jsonify({'error': 'Body JSON inválido'}), 400

            result = self._handle_command(body)
            self._stats['commands_received'] += 1

            if 'error' in result:
                return jsonify(result), 400
            return jsonify(result)

        # ---- POST /pid ----
        @app.route('/pid', methods=['POST'])
        def post_pid():
            """
            Atualiza os ganhos PID em tempo real.

            Body JSON:
                { "kp": 30.0, "ki": 0.3, "kd": 20.0 }
            """
            body = request.get_json(silent=True)
            if not body:
                return jsonify({'error': 'Body JSON inválido'}), 400

            kp = body.get('kp')
            ki = body.get('ki')
            kd = body.get('kd')

            if None in (kp, ki, kd):
                return jsonify({'error': 'Campos kp, ki, kd obrigatórios'}), 400

            try:
                robot._pid.set_gains(float(kp), float(ki), float(kd))
                logger.info(f"[WifiComm] PID atualizado: Kp={kp} Ki={ki} Kd={kd}")
                return jsonify({'ok': True, 'kp': kp, 'ki': ki, 'kd': kd})
            except Exception as e:
                return jsonify({'error': str(e)}), 500

        # ---- POST /calibrate ----
        @app.route('/calibrate', methods=['POST'])
        def post_calibrate():
            """Dispara calibração do sensor de linha em thread separada."""
            def _do_calibrate():
                try:
                    robot._line.calibrate()
                    logger.info("[WifiComm] Calibração concluída.")
                except Exception as e:
                    logger.error(f"[WifiComm] Erro na calibração: {e}")

            t = threading.Thread(target=_do_calibrate, daemon=True)
            t.start()
            return jsonify({'ok': True, 'message': 'Calibração iniciada (assíncrona)'})

        # ---- WebSocket /ws ----
        @sock.route('/ws')
        def websocket(ws):
            """
            Endpoint WebSocket para telemetria e comandos bidirecional.

            O servidor envia telemetria automaticamente a cada TELEM_INTERVAL_S.
            O cliente pode enviar comandos JSON a qualquer momento.
            """
            with self._ws_lock:
                self._ws_clients.append(ws)
                self._stats['ws_connections'] += 1

            logger.info(f"[WifiComm] WebSocket conectado. Total: {len(self._ws_clients)}")

            try:
                while True:
                    # Aguarda mensagem do cliente (não bloqueante com timeout)
                    data = ws.receive(timeout=1.0)
                    if data is None:
                        continue

                    try:
                        cmd = json.loads(data)
                        result = self._handle_command(cmd)
                        ws.send(json.dumps(result))
                        self._stats['commands_received'] += 1
                    except json.JSONDecodeError:
                        ws.send(json.dumps({'error': 'JSON inválido'}))

            except Exception as e:
                logger.debug(f"[WifiComm] WebSocket desconectado: {e}")
            finally:
                with self._ws_lock:
                    if ws in self._ws_clients:
                        self._ws_clients.remove(ws)

    # ----------------------------------------------------------
    #  HANDLER DE COMANDOS (REST e WebSocket compartilham)
    # ----------------------------------------------------------

    def _handle_command(self, body: dict) -> dict:
        """
        Executa um comando recebido via REST ou WebSocket.

        Args:
            body: Dicionário com o comando e parâmetros.
        Returns:
            Dicionário de resposta {'ok': True} ou {'error': '...'}.
        """
        robot = self._robot
        cmd   = body.get('command', '').upper()

        try:
            if cmd == 'MOVE':
                direction = body.get('direction', 'STOP')
                speed     = int(body.get('speed', 0))
                robot.send_move(direction, speed)

            elif cmd == 'STOP':
                robot.send_move('STOP', 0)

            elif cmd == 'GRIP':
                robot.send_grip(body.get('action', 'OPEN'))

            elif cmd == 'LIFT':
                robot.send_grip(body.get('action', 'UP'))

            elif cmd == 'STATE':
                from config import RobotState
                state_id = int(body.get('state', 0))
                robot.set_state(RobotState(state_id))

            elif cmd == 'CAPTURE':
                robot.capture_victim()

            elif cmd == 'CALIBRATE':
                robot._line.calibrate()

            elif cmd == 'PING':
                return {'ok': True, 'pong': True, 'timestamp': time.time()}

            else:
                return {'error': f"Comando desconhecido: '{cmd}'"}

            return {'ok': True, 'command': cmd}

        except Exception as e:
            logger.error(f"[WifiComm] Erro ao executar comando '{cmd}': {e}")
            return {'error': str(e)}

    # ----------------------------------------------------------
    #  LOOP DE TELEMETRIA WEBSOCKET
    # ----------------------------------------------------------

    def _telemetry_loop(self):
        """
        Envia telemetria periodicamente para todos os clientes WebSocket.

        Frequência: TELEM_INTERVAL_S (padrão: 100ms = 10 Hz).
        """
        while self._running:
            if self._ws_clients:
                payload = json.dumps(self._build_telemetry_payload())

                with self._ws_lock:
                    dead = []
                    for ws in self._ws_clients:
                        try:
                            ws.send(payload)
                            self._stats['telem_sent'] += 1
                        except Exception:
                            dead.append(ws)

                    for ws in dead:
                        self._ws_clients.remove(ws)

            time.sleep(TELEM_INTERVAL_S)

    def _build_telemetry_payload(self) -> dict:
        """
        Constrói o payload de telemetria a partir do estado atual do robô.

        Returns:
            Dicionário serializável em JSON com todos os dados do sistema.
        """
        robot = self._robot

        line   = robot._line.data
        victim = robot._husky.data
        pid    = robot._pid
        motors = robot._motors

        return {
            'timestamp': time.time(),
            'state'    : robot._state.name,
            'line': {
                'position'        : round(line.position, 4),
                'detected'        : line.line_detected,
                'lost'            : line.line_lost,
                'crossing'        : line.crossing_detected,
                'raw_binary'      : line.raw_binary,
            },
            'pid': {
                'output'          : round(pid.output, 2),
                'p'               : round(pid.term_p, 2),
                'i'               : round(pid.term_i, 2),
                'd'               : round(pid.term_d, 2),
                'last_error'      : round(pid.last_error, 4),
            },
            'victim': {
                'detected'        : victim.detected,
                'x'               : round(victim.normalized_x, 3),
                'y'               : round(victim.normalized_y, 3),
                'area'            : round(victim.area, 4),
                'id'              : victim.object_id,
            },
            'motors': {
                'left_duty_pct'   : round(motors.left_speed, 1),
                'right_duty_pct'  : round(motors.right_speed, 1),
            },
        }

    # ----------------------------------------------------------
    #  RUNNER FLASK
    # ----------------------------------------------------------

    def _run_flask(self):
        """Executa o servidor Flask em thread daemon."""
        try:
            # Desativa reloader e debug para uso em produção embarcada
            self._app.run(
                host=self._host,
                port=self._http_port,
                debug=False,
                use_reloader=False,
                threaded=True
            )
        except Exception as e:
            logger.error(f"[WifiComm] Erro no servidor Flask: {e}")
