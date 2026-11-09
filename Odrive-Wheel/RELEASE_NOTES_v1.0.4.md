# ODrive Wheel Forge v1.0.4

## Português

### Force Feedback e amostragem do encoder

- Lê posição e velocidade do encoder como um único snapshot coerente para os cálculos de FFB, evitando combinar valores de amostras diferentes quando a interrupção de controle ocorre entre as leituras.
- Sincroniza os escritores do snapshot entre a interrupção de controle e as rotinas de configuração/calibração.
- Acrescenta regressões para verificar a coerência do snapshot e a serialização dos escritores.

### Validação

- Os 61 testes de contrato/regressão passaram.
- O CI compilou o firmware ARM para STM32F405 com sucesso.
- O firmware foi testado no volante.

## English

### Force Feedback and encoder sampling

- Reads encoder position and velocity as one coherent snapshot for FFB calculations, preventing values from different samples being combined when the control interrupt occurs between reads.
- Synchronizes snapshot writers across the control interrupt and configuration/calibration routines.
- Adds regressions that verify snapshot coherence and writer serialization.

### Validation

- All 61 contract/regression tests passed.
- CI successfully built the STM32F405 ARM firmware.
- The firmware was tested on the wheel.
