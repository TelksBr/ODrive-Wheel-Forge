# Guia de Integração para App Controlador (Front-End)

Este documento descreve as novas variáveis, comandos ASCII e endpoints adicionados ao firmware **ODrive-Wheel (v1.0.0+)** para facilitar a integração com a nova interface PWA / WebApp.

---

## 1. Identificação e Versão do Firmware

- **Comando:** `sys.swver?`
- **Retorno:** `1.0.0-dev` (ou versão SemVer correspondente)
- **Uso no Front-End:** Usar para checar se o firmware conectado suporta as novas features (>= 1.0.0).

---

## 2. Inversão Independente de Volante e FFB

Anteriormente, havia apenas uma inversão unificada (`axis.invert`) que podia causar comportamentos indesejados dependendo do jogo/simulador. Agora a inversão de direção visual/HID e a inversão de força FFB são totalmente independentes.

### 2.1 Inversão de Eixo (Posição do Volante / HID IN)
Inverte apenas a posição reportada ao Windows/jogo e a telemetria do app.
- **Leitura:** `axis.invert?` $\rightarrow$ `0` ou `1`
- **Escrita:** `axis.invert=1` (invertido) ou `axis.invert=0` (normal)

### 2.2 Inversão de Força FFB (Torque do Jogo / HID OUT)
Inverte apenas o sentido do torque que o jogo solicita ao motor.
- **Leitura:** `axis.ffbinvert?` $\rightarrow$ `0` ou `1`
- **Escrita:** `axis.ffbinvert=1` (FFB invertido) ou `axis.ffbinvert=0` (FFB normal)

### 2.3 Persistência na EEPROM
- **Salvar alterações:** `sys.save!`
- Salva ambos os estados de forma permanente nos bits de configuração do eixo (`ADR_AXIS1_CONFIG`).

---

## 3. Telemetria e Leituras em Tempo Real

Todas as leituras de velocidade e aceleração agora são filtradas internamente via PLL e LPF de 100 Hz, proporcionando dados limpos para gráficos e medidores (gauges) no front-end:

| Comando | Tipo | Retorno / Exemplo | Descrição |
|---|---|---|---|
| `axis.curpos?` | GET | `180.45` | Posição atual do volante em graus (já respeita `axis.invert`). |
| `axis.curspd?` | GET | `12.50` | Velocidade angular atual em graus/s (PLL filtrada). |
| `axis.curaccel?` | GET | `-4.20` | Aceleração angular em graus/s² (100 Hz LPF). |
| `axis.curtorque?` | GET | `1250` | Torque atual aplicado (-32767 a +32767). |
| `sys.fxtest?` | GET | `ffb=1 pos=12.5 spd=0.0 trq=0.000 fx=2` | Resumo de telemetria e contagem de efeitos ativos. |

---

## 4. Suporte ao Encoder Magnético MT6835 (21-bit)

Se o usuário estiver utilizando o encoder MT6835 (`encoder.config.mode = 261`):

### 4.1 Status do Encoder e Alertas Magnéticos
- **Comando:** `sys.mtstatus?`
- **Retorno:** `boot=1 hyst0=1 overspeed=0 weakfield=0 undervolt=0 cal=ok`
- **Campos:**
  - `boot`: `1` se a comunicação inicial passou no CRC8.
  - `hyst0`: `1` se a histerese interna foi zerada (modo ideal para Direct Drive).
  - `weakfield`: `1` se o campo magnético está fraco (ímã muito distante do sensor).
  - `overspeed`: `1` se ultrapassou a rotação limite do sensor.
  - `undervolt`: `1` se houver sub-tensão no sensor.
  - `cal`: Estado da auto-calibração (`none`, `running`, `failed`, `ok`).

### 4.2 Calibração de Zero e Gravação na EEPROM do Chip
- **Definir Zero Atual:** `sys.mtzero!`
  - *Retorno:* `OK (volatil - sys.mteeprom! para persistir)` ou `FAIL` se o motor estiver armado.
- **Gravar EEPROM do MT6835:** `sys.mteeprom!`
  - *Retorno:* `OK - NAO desligar por 6s (leituras pausadas 6.5s)`
  - *Importante:* Exibir aviso na UI para não desconectar a alimentação por 6 segundos.

### 4.3 Leitura e Escrita Direta de Registradores
- **Ler registrador:** `sys.mtread=0x011` $\rightarrow$ `reg[0x011]=0x02`
- **Escrever registrador:** `sys.mtwrite=0x011 0x03` $\rightarrow$ `reg[0x011]=0x03`

---

## 5. Diagnóstico de Integridade de Comunicação SPI

- **Comando:** `sys.encraw!`
- **Retorno:** `ok=125430 pty=0 ef=0 xfr=0 last=0x0A2B pos=524288`
- **Interpretação para UI:**
  - `ok`: Quantidade de leituras válidas.
  - `pty`: Erros de paridade / CRC8 (deve ser 0 ou próximo de 0).
  - `xfr`: Erros de barramento SPI ocupado / timeout.
