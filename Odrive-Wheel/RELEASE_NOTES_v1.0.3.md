# ODrive Wheel Forge v1.0.3

## Português

### Sincronização de telemetria e encoder

- Publica a escala do divisor VBUS como atômica entre a configuração em runtime e a interrupção do ADC.
- Atualiza os picos de corrente do barramento, corrente do motor e tensão VBUS com CAS lock-free; leituras e resets do diagnóstico também são atômicos.
- Converte os contadores de pulsos Z aceitos e glitches do encoder para incrementos atômicos no ISR e acesso atômico pelo protocolo serial.

### Validação

- 60 testes de contrato/regressão passaram.
- Build ARM limpo para STM32F405 concluído sem warnings de compilador.
- ELF verificado sem dependências `__atomic`/`__sync` fora de linha.

## English

### Telemetry and encoder synchronization

- Publishes the VBUS divider scale atomically between runtime configuration and the ADC interrupt.
- Updates bus current, motor current, and VBUS peaks with lock-free CAS; diagnostic reads and resets are atomic as well.
- Makes accepted encoder Z pulses and glitches atomic counters, with ISR increments and serial protocol reads/resets.

### Validation

- All 60 contract/regression tests passed.
- Clean STM32F405 ARM build completed without compiler warnings.
- ELF inspection found no out-of-line `__atomic`/`__sync` dependencies.
