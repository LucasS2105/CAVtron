# Robô Seguidor de Linha com Resgate e Mapeamento

## Visão Geral

Este projeto consiste no desenvolvimento de um robô móvel autônomo capaz de:

* Seguir linha com controle PID
* Identificar vítimas utilizando câmera HuskyLens
* Mapear a arena e detectar obstáculos com LIDAR
* Manipular objetos com garra acionada por servos
* Tomar decisões com base em múltiplos sensores

A arquitetura do sistema é distribuída entre dois níveis de processamento:

* **Arduino (tempo real e controle de hardware)**
* **Raspberry Pi (processamento avançado e tomada de decisão)**

---
## Arquitetura do Sistema
### Arduino (Controle em Tempo Real)

Responsabilidades:

* Controle de motores
* Execução de PID (com base nos dados da HuskyLens)
* Controle de servos (garra)
* Leitura dos dados da HuskyLens via UART/I2C
* Comunicação com Raspberry Pi

Linguagem:

C/C++

###Raspberry Pi (Processamento Avançado)

Responsabilidades:

* Leitura do LIDAR
* Mapeamento da arena
* Detecção de obstáculos
* Sistema de decisão (alto nível)
* Coordenação geral do robô

Linguagem:

* Python
---

## Módulos do Sistema

### 1. Seguimento de Linha

* Sensores infravermelhos
* Controle baseado em PID
* Implementado no Arduino

### 2. Identificação de Vítimas

* Câmera HuskyLens
* Processamento interno da câmera
* Arduino realiza leitura dos resultados via comunicação serial/I2C

### 3. Mapeamento e Navegação

* Sensor LIDAR
* Processamento realizado no Raspberry Pi
* Construção de mapa e detecção de obstáculos

### 4. Manipulação (Garra)

* Servomotores controlados pelo Arduino
* Atuação baseada em comandos do Raspberry Pi

### 5. Comunicação

* Interface serial entre Arduino e Raspberry Pi
* Protocolo simples de troca de mensagens

Exemplo de mensagens:

* Raspberry → Arduino: comandos de movimento
* Arduino → Raspberry: status dos sensores

---

## Fluxo de Operação

1. Arduino executa controle de linha continuamente
2. Raspberry Pi processa dados do LIDAR
3. HuskyLens detecta possíveis vítimas
4. Arduino recebe dados da HuskyLens
5. Raspberry Pi decide a ação:
   * continuar na linha
   * desviar de obstáculo
   * capturar vítima
6. Raspberry envia comando ao Arduino
7. Arduino executa ação (movimento ou garra)

---

## Considerações Técnicas

* O Arduino não possui capacidade para lidar com mapeamento ou processamento complexo de sensores como LIDAR
* O Raspberry Pi não é adequado para controle em tempo real de motores com precisão
* A separação de responsabilidades é essencial para desempenho e estabilidade do sistema
* O uso combinado das duas plataformas permite escalabilidade e robustez

---

## Possíveis Extensões

* Integração com ROS (Robot Operating System)
* Implementação de SLAM completo
* Uso de OpenCV para visão computacional avançada
* Aprimoramento do sistema de decisão com aprendizado de máquina

---

## Requisitos

Hardware:

* Arduino (Uno, Mega ou similar)
* Raspberry Pi
* Sensor LIDAR
* HuskyLens
* Motores DC + driver
* Servomotores

Software:

* Arduino IDE
* Python 3
* Bibliotecas de controle de hardware e sensores
* (Opcional) ROS para aplicações avançadas

---

## Conclusão

A combinação de C/C++ no Arduino com Python no Raspberry Pi oferece uma solução eficiente, modular e escalável para o desenvolvimento de um robô autônomo com múltiplas capacidades, atendendo às demandas de controle em tempo real e processamento avançado simultaneamente.

# Estrutura de Projeto e Organização de Pastas

A organização abaixo foi projetada para um sistema híbrido (Arduino + Raspberry Pi), com separação clara de responsabilidades, modularidade e facilidade de manutenção.

---

## Estrutura Geral

```
robot-project/
│
├── docs/
├── hardware/
├── arduino/
├── raspberry/
├── communication/
├── tests/
├── scripts/
├── config/
├── logs/
├── README.md
└── .gitignore
```

---

## Descrição dos Diretórios

### `/docs`

Documentação do projeto.

```
docs/
├── architecture.md
├── communication_protocol.md
├── wiring_diagram.png
└── datasheets/
```

Conteúdo:

* Arquitetura do sistema
* Protocolo de comunicação Arduino ↔ Raspberry
* Diagramas elétricos
* Datasheets dos componentes

---

### `/hardware`

Informações físicas e montagem do robô.

```
hardware/
├── schematics/
├── pcb/
└── assembly/
```

Conteúdo:

* Esquemáticos elétricos
* Layout de placas (se houver)
* Instruções de montagem mecânica

---

## Arduino (Controle em Tempo Real)

### `/arduino`

```
arduino/
├── src/
│   ├── main.ino
│   ├── config.h
│   │
│   ├── control/
│   │   ├── pid_controller.cpp
│   │   └── pid_controller.h
│   │
│   ├── drivers/
│   │   ├── motor_driver.cpp
│   │   ├── motor_driver.h
│   │   ├── servo_driver.cpp
│   │   └── servo_driver.h
│   │
│   ├── sensors/
│   │   ├── huskylens.cpp
│   │   └── huskylens.h
│   │
│   ├── communication/
│   │   ├── serial_comm.cpp
│   │   └── serial_comm.h
│   │
│   └── utils/
│       ├── filters.cpp
│       └── filters.h
│
└── lib/
```

Responsabilidades:

* Controle de motores
* PID
* Leitura de sensores
* Controle de servos
* Interface com HuskyLens
* Comunicação serial

---

## Raspberry Pi (Processamento e Inteligência)

### `/raspberry`

```
raspberry/
├── src/
│   ├── main.py
│   │
│   ├── control/
│   │   └── state_machine.py
│   │
│   ├── mapping/
│   │   ├── lidar_interface.py
│   │   ├── mapping.py
│   │   └── obstacle_detection.py
│   │
│   ├── vision/
│   │   └── vision_processing.py
│   │
│   ├── communication/
│   │   └── serial_comm.py
│   │
│   ├── decision/
│   │   └── decision_system.py
│   │
│   └── utils/
│       └── helpers.py
│
├── requirements.txt
└── run.sh
```

Responsabilidades:

* Leitura do LIDAR
* Mapeamento da arena
* Detecção de obstáculos
* Sistema de decisão
* Controle de estados
* Comunicação com Arduino

---

## Comunicação

### `/communication`

```
communication/
├── protocol.md
├── message_types.json
└── examples/
```

Conteúdo:

* Definição do protocolo serial
* Tipos de mensagens
* Exemplos de troca de dados

Exemplo de mensagem:

```
<MOVE,FORWARD,120>
<GRIP,OPEN>
<STATUS,LINE_DETECTED>
```

---

## Testes

### `/tests`

```
tests/
├── arduino/
├── raspberry/
└── integration/
```

Conteúdo:

* Testes unitários (quando possível)
* Testes de comunicação
* Testes integrados (hardware + software)

---

## Scripts

### `/scripts`

```
scripts/
├── deploy.sh
├── start_robot.sh
└── monitor_serial.py
```

Funções:

* Inicialização do sistema
* Automação de execução
* Monitoramento de logs e comunicação

---

## Configurações

### `/config`

```
config/
├── robot_config.yaml
├── pid_config.yaml
└── hardware_config.yaml
```

Conteúdo:

* Parâmetros do robô
* Constantes de PID
* Configurações de sensores e hardware

---

## Logs

### `/logs`

```
logs/
├── system.log
├── errors.log
└── lidar.log
```

Uso:

* Registro de execução
* Debug
* Análise de comportamento
