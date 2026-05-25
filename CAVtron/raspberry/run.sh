#!/usr/bin/env bash
# =============================================================
#  run.sh — Script de Inicialização do Sistema Raspberry Pi
#  Robô Seguidor de Linha com Resgate e Mapeamento
# =============================================================
#
#  Uso:
#      ./run.sh [OPÇÃO]
#
#  Opções:
#      (sem argumento)   Inicia o sistema normalmente
#      --install         Instala/atualiza dependências Python
#      --check           Verifica pré-requisitos sem iniciar
#      --debug           Inicia com logging em nível DEBUG
#      --dry-run         Valida configuração sem conectar hardware
#      --monitor         Abre monitor serial (ttyS/ttyUSB) após start
#      --help            Exibe esta ajuda
#
#  O script realiza, em ordem:
#    1. Validação de pré-requisitos (Python, pip, portas seriais)
#    2. Ativação do ambiente virtual (cria se não existir)
#    3. Instalação de dependências (com --install ou se ausentes)
#    4. Configuração de permissões de hardware (dialout, tty)
#    5. Verificação de disponibilidade das portas (UART, USB)
#    6. Inicialização do processo Python com reinicialização automática
#    7. Captura de logs e rotação
#
#  Shutdown limpo:
#    Ctrl+C ou SIGTERM disparam SIGINT no processo filho,
#    que aciona o shutdown handler do Python (motores parados,
#    LIDAR desligado, serial fechada).
# =============================================================

set -euo pipefail

# -------------------------------------------------------
#  CONSTANTES E CAMINHOS
# -------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

SRC_DIR="${SCRIPT_DIR}/src"
VENV_DIR="${SCRIPT_DIR}/.venv"
REQUIREMENTS="${SCRIPT_DIR}/requirements.txt"
LOG_DIR="${PROJECT_ROOT}/logs"
LOG_FILE="${LOG_DIR}/system.log"
PID_FILE="/tmp/robot_system.pid"

PYTHON_MIN_MAJOR=3
PYTHON_MIN_MINOR=9

# Portas de hardware (ajustar conforme hardware)
SERIAL_PORT_ARDUINO="${SERIAL_PORT:-/dev/serial0}"  # GPIO UART (RPi 4/5)
SERIAL_PORT_LIDAR="${LIDAR_PORT:-/dev/ttyUSB0}"      # USB LIDAR

# Cores para output no terminal
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'  # No Color

# -------------------------------------------------------
#  FUNÇÕES AUXILIARES
# -------------------------------------------------------

log_info()    { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $*" >&2; }
log_step()    { echo -e "${BLUE}[STEP]${NC}  ${BOLD}$*${NC}"; }
log_ok()      { echo -e "${GREEN}[  OK]${NC}  $*"; }
log_skip()    { echo -e "${CYAN}[SKIP]${NC}  $*"; }

print_banner() {
    echo ""
    echo -e "${BOLD}${CYAN}╔══════════════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}${CYAN}║        Robot Rescue System — Raspberry Pi Layer      ║${NC}"
    echo -e "${BOLD}${CYAN}║          Seguidor de Linha · LIDAR · Resgate         ║${NC}"
    echo -e "${BOLD}${CYAN}╚══════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_help() {
    echo -e "${BOLD}Uso:${NC} ./run.sh [OPÇÃO]"
    echo ""
    echo -e "${BOLD}Opções:${NC}"
    echo "  (sem argumento)   Inicia o sistema normalmente"
    echo "  --install         Instala/atualiza dependências Python"
    echo "  --check           Verifica pré-requisitos sem iniciar"
    echo "  --debug           Inicia com logging em nível DEBUG"
    echo "  --dry-run         Valida configuração sem conectar hardware"
    echo "  --monitor         Abre monitor serial após iniciar"
    echo "  --help            Exibe esta mensagem"
    echo ""
    echo -e "${BOLD}Variáveis de ambiente:${NC}"
    echo "  SERIAL_PORT       Porta do Arduino (padrão: /dev/serial0)"
    echo "  LIDAR_PORT        Porta do LIDAR   (padrão: /dev/ttyUSB0)"
    echo "  LOG_LEVEL         Nível de log     (padrão: INFO)"
    echo ""
}

# -------------------------------------------------------
#  VERIFICAÇÃO DE PRÉ-REQUISITOS
# -------------------------------------------------------

check_python() {
    log_step "Verificando Python..."

    if ! command -v python3 &>/dev/null; then
        log_error "Python 3 não encontrado. Instale: sudo apt install python3"
        exit 1
    fi

    local version
    version=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
    local major minor
    major=$(echo "$version" | cut -d. -f1)
    minor=$(echo "$version" | cut -d. -f2)

    if (( major < PYTHON_MIN_MAJOR || (major == PYTHON_MIN_MAJOR && minor < PYTHON_MIN_MINOR) )); then
        log_error "Python ${PYTHON_MIN_MAJOR}.${PYTHON_MIN_MINOR}+ necessário. Encontrado: ${version}"
        exit 1
    fi

    log_ok "Python ${version}"
}

check_pip() {
    log_step "Verificando pip..."

    if ! python3 -m pip --version &>/dev/null; then
        log_warn "pip não encontrado. Instalando..."
        sudo apt-get install -y python3-pip
    fi

    log_ok "pip $(python3 -m pip --version | awk '{print $2}')"
}

check_hardware_ports() {
    log_step "Verificando portas de hardware..."

    local all_ok=true

    # Porta Arduino (UART GPIO)
    if [ -e "${SERIAL_PORT_ARDUINO}" ]; then
        log_ok "Arduino serial: ${SERIAL_PORT_ARDUINO}"
    else
        log_warn "Arduino serial não encontrada: ${SERIAL_PORT_ARDUINO}"
        log_warn "  → Verifique se UART está habilitada em /boot/config.txt"
        log_warn "  → Use: sudo raspi-config → Interface Options → Serial Port"
        all_ok=false
    fi

    # Porta LIDAR (USB)
    if [ -e "${SERIAL_PORT_LIDAR}" ]; then
        log_ok "LIDAR USB: ${SERIAL_PORT_LIDAR}"
    else
        log_warn "LIDAR USB não encontrado: ${SERIAL_PORT_LIDAR}"
        log_warn "  → Conecte o sensor LIDAR via USB e verifique com: ls /dev/ttyUSB*"
        all_ok=false
    fi

    if [ "${all_ok}" = false ]; then
        log_warn "Hardware parcialmente ausente — sistema iniciará em modo degradado."
    fi
}

check_permissions() {
    log_step "Verificando permissões de hardware..."

    local user_groups
    user_groups=$(groups)

    # Grupo dialout: necessário para acesso às portas seriais
    if echo "${user_groups}" | grep -q "dialout"; then
        log_ok "Usuário no grupo 'dialout'"
    else
        log_warn "Usuário não está no grupo 'dialout'. Adicionando..."
        sudo usermod -aG dialout "${USER}"
        log_warn "ATENÇÃO: Reinicie a sessão para aplicar as permissões de grupo."
        log_warn "         Execute: newgrp dialout (para sessão atual)"
        newgrp dialout || true
    fi

    # Habilitar UART no Raspberry Pi (se não estiver)
    if [ -f /boot/config.txt ]; then
        if ! grep -q "enable_uart=1" /boot/config.txt; then
            log_warn "UART pode não estar habilitada. Verifique /boot/config.txt"
            log_warn "  → Adicione: enable_uart=1"
        fi
    fi
}

check_i2c() {
    log_step "Verificando I2C (HuskyLens via Arduino)..."
    # HuskyLens conecta ao Arduino via I2C. O Arduino gerencia a interface.
    # I2C direto no RPi não é necessário nesta arquitetura.
    log_skip "I2C gerenciado pelo Arduino — sem verificação necessária"
}

# -------------------------------------------------------
#  AMBIENTE VIRTUAL PYTHON
# -------------------------------------------------------

setup_venv() {
    log_step "Configurando ambiente virtual Python..."

    if [ ! -d "${VENV_DIR}" ]; then
        log_info "Criando ambiente virtual em ${VENV_DIR}..."
        python3 -m venv "${VENV_DIR}" --system-site-packages
        # --system-site-packages permite acesso a pacotes do sistema
        # (ex: python3-opencv instalado via apt)
        log_ok "Ambiente virtual criado."
    else
        log_ok "Ambiente virtual existente: ${VENV_DIR}"
    fi

    # Ativa o ambiente virtual
    # shellcheck source=/dev/null
    source "${VENV_DIR}/bin/activate"
    log_ok "Ambiente virtual ativado."
}

# -------------------------------------------------------
#  INSTALAÇÃO DE DEPENDÊNCIAS
# -------------------------------------------------------

install_dependencies() {
    log_step "Instalando dependências Python..."

    if [ ! -f "${REQUIREMENTS}" ]; then
        log_error "requirements.txt não encontrado: ${REQUIREMENTS}"
        exit 1
    fi

    # Atualiza pip silenciosamente
    python3 -m pip install --upgrade pip --quiet

    # Instala dependências
    python3 -m pip install -r "${REQUIREMENTS}" \
        --quiet \
        --no-warn-script-location

    log_ok "Dependências instaladas com sucesso."
}

check_dependencies() {
    log_step "Verificando dependências instaladas..."

    local missing=()

    # Verifica cada pacote crítico
    local packages=("serial" "numpy" "yaml" "rplidar")
    local names=("pyserial" "numpy" "PyYAML" "rplidar-roboticia")

    for i in "${!packages[@]}"; do
        if python3 -c "import ${packages[$i]}" &>/dev/null; then
            log_ok "  ${names[$i]}"
        else
            log_warn "  AUSENTE: ${names[$i]}"
            missing+=("${names[$i]}")
        fi
    done

    if [ ${#missing[@]} -gt 0 ]; then
        log_warn "Dependências ausentes. Execute: ./run.sh --install"
        return 1
    fi

    return 0
}

# -------------------------------------------------------
#  CONFIGURAÇÃO DO SISTEMA
# -------------------------------------------------------

setup_directories() {
    log_step "Criando diretórios necessários..."
    mkdir -p "${LOG_DIR}"
    log_ok "Diretórios: ${LOG_DIR}"
}

write_robot_config() {
    local config_dir="${PROJECT_ROOT}/config"
    local config_file="${config_dir}/robot_config.yaml"

    mkdir -p "${config_dir}"

    if [ ! -f "${config_file}" ]; then
        log_info "Gerando robot_config.yaml padrão em ${config_file}..."

        cat > "${config_file}" << YAML
# robot_config.yaml
# Gerado automaticamente por run.sh
# Ajuste os parâmetros conforme o hardware utilizado.

serial:
  port: "${SERIAL_PORT_ARDUINO}"
  baudrate: 115200
  watchdog_s: 2.0

lidar:
  port: "${SERIAL_PORT_LIDAR}"
  obstacle_dist_mm: 350.0
  warning_dist_mm: 600.0
  min_quality: 5
  min_dist_mm: 100.0
  max_dist_mm: 6000.0

arena:
  width_m: 3.0
  height_m: 3.0
  resolution_m: 0.05

mission:
  victims_target: 1
  loop_period_ms: 20
  ping_interval_s: 1.0
YAML
        log_ok "robot_config.yaml criado."
    else
        log_skip "robot_config.yaml já existe."
    fi
}

# -------------------------------------------------------
#  INICIALIZAÇÃO DO PROCESSO PRINCIPAL
# -------------------------------------------------------

start_system() {
    local log_level="${LOG_LEVEL:-INFO}"
    local extra_args=""

    if [ "${MODE:-}" = "debug" ]; then
        log_level="DEBUG"
    fi

    if [ "${MODE:-}" = "dry-run" ]; then
        extra_args="--dry-run"
        log_info "Modo DRY-RUN ativado — hardware não será acessado."
    fi

    # Verifica se já está rodando
    if [ -f "${PID_FILE}" ]; then
        local old_pid
        old_pid=$(cat "${PID_FILE}")
        if kill -0 "${old_pid}" 2>/dev/null; then
            log_error "Sistema já em execução (PID ${old_pid})."
            log_error "  Para parar: kill ${old_pid}  ou  kill -SIGTERM ${old_pid}"
            exit 1
        else
            rm -f "${PID_FILE}"
        fi
    fi

    log_step "Iniciando sistema principal..."
    log_info "Log:        ${LOG_FILE}"
    log_info "Nível log:  ${log_level}"
    log_info "Arduino:    ${SERIAL_PORT_ARDUINO}"
    log_info "LIDAR:      ${SERIAL_PORT_LIDAR}"
    echo ""

    # Configura variáveis de ambiente para o processo Python
    export SERIAL_PORT="${SERIAL_PORT_ARDUINO}"
    export LIDAR_PORT="${SERIAL_PORT_LIDAR}"
    export LOG_LEVEL="${log_level}"
    export PYTHONPATH="${SRC_DIR}:${PYTHONPATH:-}"
    export PYTHONUNBUFFERED=1   # Garante flush imediato dos logs

    # Inicia o processo Python
    # tee: duplica saída para terminal E arquivo de log simultaneamente
    python3 "${SRC_DIR}/main.py" ${extra_args} 2>&1 | \
        tee -a "${LOG_FILE}" &

    local pid=$!
    echo "${pid}" > "${PID_FILE}"
    log_ok "Sistema iniciado (PID ${pid})"

    # Trap para shutdown limpo
    trap 'shutdown_system ${pid}' INT TERM EXIT

    # Aguarda o processo terminar
    wait "${pid}" || true
    rm -f "${PID_FILE}"
}

shutdown_system() {
    local pid="${1:-}"
    echo ""
    log_step "Iniciando shutdown limpo..."

    if [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null; then
        log_info "Enviando SIGINT ao processo principal (PID ${pid})..."
        kill -SIGINT "${pid}" 2>/dev/null || true

        # Aguarda finalização graciosa (máx 5s)
        local timeout=50
        while kill -0 "${pid}" 2>/dev/null && (( timeout > 0 )); do
            sleep 0.1
            (( timeout-- ))
        done

        # Força encerramento se necessário
        if kill -0 "${pid}" 2>/dev/null; then
            log_warn "Processo não respondeu — forçando encerramento (SIGKILL)."
            kill -SIGKILL "${pid}" 2>/dev/null || true
        fi
    fi

    rm -f "${PID_FILE}"
    log_ok "Sistema encerrado."

    # Remove trap para evitar loop
    trap - INT TERM EXIT
}

# -------------------------------------------------------
#  MONITOR SERIAL (opcional)
# -------------------------------------------------------

open_serial_monitor() {
    log_step "Abrindo monitor serial..."

    if command -v minicom &>/dev/null; then
        log_info "Minicom: minicom -b 115200 -D ${SERIAL_PORT_ARDUINO}"
        minicom -b 115200 -D "${SERIAL_PORT_ARDUINO}"
    elif command -v screen &>/dev/null; then
        log_info "Screen: screen ${SERIAL_PORT_ARDUINO} 115200"
        screen "${SERIAL_PORT_ARDUINO}" 115200
    elif python3 -c "import serial" &>/dev/null; then
        log_info "Usando monitor Python (Ctrl+C para sair)..."
        python3 "${SCRIPT_DIR}/../../scripts/monitor_serial.py" \
            --port "${SERIAL_PORT_ARDUINO}" \
            --baudrate 115200
    else
        log_warn "Nenhuma ferramenta de monitor serial encontrada."
        log_warn "Instale: sudo apt install minicom  ou  sudo apt install screen"
    fi
}

# -------------------------------------------------------
#  FLUXO PRINCIPAL
# -------------------------------------------------------

main() {
    local mode="${1:-}"

    # Interpretação de argumentos
    case "${mode}" in
        --help|-h)
            print_banner
            print_help
            exit 0
            ;;
        --install)
            print_banner
            check_python
            check_pip
            setup_venv
            install_dependencies
            log_ok "Instalação concluída."
            exit 0
            ;;
        --check)
            print_banner
            check_python
            check_pip
            setup_venv
            check_dependencies || true
            check_hardware_ports
            check_permissions
            log_ok "Verificação concluída."
            exit 0
            ;;
        --debug)
            MODE="debug"
            ;;
        --dry-run)
            MODE="dry-run"
            ;;
        --monitor)
            print_banner
            setup_venv
            open_serial_monitor
            exit 0
            ;;
        "")
            MODE="normal"
            ;;
        *)
            log_error "Opção desconhecida: ${mode}"
            print_help
            exit 1
            ;;
    esac

    # Fluxo normal de inicialização
    print_banner

    check_python
    check_pip
    setup_directories
    setup_venv

    # Instala dependências se ausentes
    if ! check_dependencies 2>/dev/null; then
        log_warn "Dependências ausentes. Instalando automaticamente..."
        install_dependencies
    fi

    check_hardware_ports
    check_permissions
    check_i2c

    write_robot_config

    echo ""
    log_info "Todos os pré-requisitos verificados. Iniciando em 2 segundos..."
    log_info "  Pressione Ctrl+C para cancelar."
    sleep 2

    start_system
}

# Executa
main "$@"
