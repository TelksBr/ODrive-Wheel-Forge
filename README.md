# ODrive Wheel Forge — Firmware Direct Drive Force Feedback

Firmware de alta performance para controle de motores Direct Drive e Force Feedback (FFB) em bases **ODrive v3.6**, **MKS XDrive Mini** e **ODESC V4.2** (STM32F405), combinando o controle vetorial de motor (FOC) do **ODrive v0.5.6** com a stack de Force Feedback do **OpenFFBoard** e compatibilidade total com DirectInput / USB HID PID.

<p align="center">
  <a href="https://github.com/TelksBr/OdriveAPP">
    <img src="https://img.shields.io/badge/Companion%20App-WheelForge%20PWA-0ea5e9?style=for-the-badge&logo=pwa&logoColor=white" alt="WheelForge PWA">
  </a>
  <a href="https://github.com/TelksBr/ODrive-Wheel-Forge/releases/latest">
    <img src="https://img.shields.io/github/v/release/TelksBr/ODrive-Wheel-Forge?style=for-the-badge&color=22c55e" alt="Latest Release">
  </a>
  <a href="https://github.com/TelksBr/ODrive-Wheel-Forge/actions/workflows/build.yml">
    <img src="https://img.shields.io/github/actions/workflow/status/TelksBr/ODrive-Wheel-Forge/build.yml?style=for-the-badge&label=Build%20Firmware" alt="Build Status">
  </a>
</p>

---

## 🏎️ Aplicativo Oficial: WheelForge PWA

O firmware é 100% integrado ao **[WheelForge PWA](https://github.com/TelksBr/OdriveAPP)** — um Progressive Web App moderno, rápido e intuitivo para configuração completa da sua base sem necessidade de instalar drivers complexos:

- **Conexão Direta**: Comunicação via **Web Serial API** e **WebHID API** direto do navegador (Chrome, Edge, Opera).
- **DFU Flashing no Navegador**: Atualização de firmware via **WebUSB DFU** com um clique, sem usar jumpers ou utilitários externos.
- **Telemetria em Tempo Real (1 kHz)**: Monitoramento de corrente $I_q$, torque em Nm, tensão $V_{bus}$, velocidade e saturação sem interferir no jogo.
- **Assistente de Calibração Guiado**: Wizard passo a passo para configuração de motor, encoder (MT6835, AS5047, ABZ), proteções de fonte e anti-cogging.
- **Ajuste Fino de FFB**: Ganhos por efeito, filtros passa-baixa Biquad customizados, batentes eletrônicos dinâmicos e testes de força em tempo real.

👉 Acesse o repositório do app: **[TelksBr/OdriveAPP](https://github.com/TelksBr/OdriveAPP)**

---

## ✨ Principais Recursos do Firmware

### 1. Suporte a Encoders de Alta Resolução
- **MagnTek MT6835 (21 bits - 2.097.152 CPR)**: Novo modo SPI nativo (`mode = 261` / `0x105`) com validação por CRC-8 em tempo real, leitura contínua (*burst read*) e thread FreeRTOS dedicada sincronizada a 8 kHz.
- **AMS AS5047D / AS5047P**: Leitura SPI absoluta de 14 bits.
- **Encoders Incrementais ABZ**: Suporte com ou sem pino Z de indexação.
- **Zero Mecânico Persistente**: Ajuste de centro mecânico com persistência na memória flash (`axis.zeroenc`).

### 2. Inversão Independente de Eixo e Torque FFB
- `axis.invert (0 ou 1)`: Inverte o sentido de leitura da posição física do volante.
- `axis.ffbinvert (0 ou 1)`: Inverte o sentido do torque do Force Feedback gerado pelos jogos sem alterar o sentido do eixo.
- Armazenamento persistente na EEPROM emulada (`ADR_AXIS1_CONFIG`).

### 3. Métricas Dinâmicas Filtradas com ODrive PLL
- Eliminação de ruído diferencial ($10^6$ noise amplification) em efeitos de amortecimento (Damper), atrito (Friction) e inércia (Inertia) através da velocidade estimada do estimador de estado PLL do ODrive (`vel_estimate * 360.0f`).
- Filtro passa-baixa de 1ª ordem a 100 Hz na aceleração angular.

### 4. Arbiter SPI Atômico
- Proteção atômica de reentrância no barramento SPI (`in_transfer_`) e escrita direta de baixo atraso (< 1 µs) nos registradores de controle do STM32, eliminando perda de pacotes USB HID OUT durante a gameplay.

---

## 🛠️ Suíte Completa de Correções FFB (DirectInput / USB HID PID)

Este fork traz correções essenciais para problemas históricos em simuladores de corrida:

| Efeito / Área | Comportamento Corrigido | Simuladores Beneficiados |
|---|---|---|
| **Direção Polar 0° / 180°** | Força não é mais perdida quando o jogo envia ângulo Norte/Sul em volantes de 1 eixo. | *Assetto Corsa*, *rFactor 2*, *KartKraft* |
| **PID State Report (`GET_REPORT ID 2`)** | Resposta síncrona com flags de atuadores e energia ativos (`DIGFFS_ACTUATORSON` / `POWERON`). | *Forza Horizon 4/5*, *Forza Motorsport*, ferramentas DirectInput |
| **Saturação Zero (`dwSat = 0`)** | Saturação zero agora assume 100% (`0x7FFF`), evitando efeitos de mola, atrito e damper mudos. | *Assetto Corsa*, *Automobilista 2*, *Project CARS* |
| **Proteção de Filtros (Null Pointer)** | Guardas seguras contra ponteiro nulo em filtros Biquad, eliminando HardFault ao trocar de carro/pista. | Todos os simuladores |
| **Pool de Memória (`used_effects`)** | Decremento correto em `PID Block Free Report`, eliminando underflow de memória em sessões longas. | *iRacing*, *ACC*, *BeamNG.drive* |
| **DC Stop All Effects (`0x04`)** | Interrupção imediata de todos os efeitos ativos, evitando trancos violentos ao bater ou renascer o carro. | *BeamNG.drive*, *DiRT Rally 2.0* |
| **Ramp Force** | Proteção contra divisão por zero (`duration = 0`) e clamp de interpolação sem extrapolação de força. | Jogos com efeitos de colisão em rampa |
| **Ondas Periódicas (Sine, Saw, Tri)** | Aritmética modular inteira com normalização de fase $[0.0, 1.0)$, sem jitter após horas contínuas de uso. | Corridas de longa duração (Endurance) |
| **Parameter Block Offset** | Máscara de bits `& 0x03` e clamp de eixo, garantindo mola e damper em volantes de 1 eixo. | Jogos Codemasters (*F1*, *DiRT Rally*, *GRID*) |

---

## 📥 Instalação e Atualização

### Opção 1: Via WheelForge PWA (Recomendado)
1. Conecte sua base via cabo USB ao computador.
2. Abra o **[WheelForge PWA](https://github.com/TelksBr/OdriveAPP)** em um navegador compatível (Chrome/Edge).
3. Vá na aba **DFU Flash**, selecione o arquivo `odrive-wheel.bin` do release e clique em **Gravar Firmware**.

### Opção 2: Via `dfu-util` (Primeira Gravação)
Coloque a placa em modo DFU (ligando com o jumper BOOT0 ativado ou acionando o botão DFU):
```bash
dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D build/odrive-wheel.bin
```

### Opção 3: Via ST-Link / STM32CubeProgrammer
Grave o arquivo `odrive-wheel.hex` iniciando no endereço `0x08000000`.

---

## 🔨 Compilação do Código-Fonte

### Compilação Local no Windows (PowerShell)

Requisitos:
- **MSYS2** instalado em `C:\msys64` (com pacote `mingw-w64-x86_64-make`).
- **Arm GNU Toolchain** (`arm-none-eabi-gcc`).
- **Python 3** com os módulos `pyyaml`, `jinja2`, `jsonschema`.

Execute o script de build local:
```powershell
powershell -ExecutionPolicy Bypass -File .\build-local.ps1 -SkipSubmoduleInit
```

O binário final será gerado em `Odrive-Wheel/build/odrive-wheel.bin`.

### Compilação Automatizada via GitHub Actions

O repositório conta com CI/CD completo no GitHub Actions ([build.yml](.github/workflows/build.yml)). A cada push na branch `main` ou criação de uma tag de versão (`v*`), o firmware é automaticamente compilado e os artefatos (`.bin`, `.hex`, `.elf`) são publicados diretamente na aba **Releases**.

---

## 📜 Licenças e Créditos

Este projeto integra tecnologias de ponta em controle de motores e simulação:
- **[ODrive Firmware v0.5.6](https://github.com/odriverobotics/ODrive)** — MIT License
- **[OpenFFBoard](https://github.com/Ultrawipf/OpenFFBoard)** — GPLv3 License
- **ODrive-Wheel Core & ODrive Wheel Forge** — GPLv3 License

Distribuído sob os termos da licença **GPLv3**.
