# ODrive Wheel Forge v1.0.2

## Português

### Force Feedback e segurança de concorrência

- Sincroniza a amostragem do FFB a 1 kHz com o ciclo de controle do motor e publica com segurança o semáforo usado pelo ISR.
- Usa snapshots atômicos para telemetria de corrente, tensão, velocidade e torque entre o controle do motor, a tarefa FFB e os relatórios HID.
- Converte contadores e métricas compartilhados do FFB para operações atômicas lock-free, evitando leituras inconsistentes e incrementos perdidos.
- Limita a região crítica ao montar o relatório HID e atualiza membros `packed` por meio de variáveis alinhadas.
- Evita reentrada no mutex ao processar o botão de zeragem do volante.
- Protege a configuração e o estado dos filtros GPIO entre tarefas; salva uma cópia consistente antes das gravações na flash.
- Converte comandos de torque não finitos em torque zero.

### Validação

- 57 testes de contrato/regressão passaram.
- Build ARM para STM32F405 concluído e `odrive-wheel.bin` gerado.
- Verificação do ELF confirmou ausência de dependências `__atomic`/`__sync` fora de linha.

## English

### Force feedback and concurrency safety

- Synchronizes FFB sampling at 1 kHz with the motor control loop and safely publishes the semaphore used by the ISR.
- Uses atomic snapshots for current, voltage, velocity, and torque telemetry shared across motor control, the FFB task, and HID reports.
- Makes shared FFB diagnostic counters and timing metrics lock-free atomics, preventing inconsistent reads and lost increments.
- Narrows the HID report critical section and writes `packed` members through aligned local variables.
- Avoids mutex re-entry while processing the wheel-zero button.
- Protects GPIO configuration and filter state across tasks; saves a consistent snapshot before flash writes.
- Converts non-finite torque commands to zero torque.

### Validation

- All 57 contract/regression tests passed.
- STM32F405 ARM build completed and generated `odrive-wheel.bin`.
- ELF inspection confirmed no out-of-line `__atomic`/`__sync` dependencies.
