# ODrive Wheel Forge v1.0.5

## Português

### Force Feedback e desempenho

- Reduz o custo de publicação do snapshot de movimento do encoder na interrupção de controle, removendo operações atômicas exclusivas desnecessárias no caminho do ISR.
- Calcula o efeito periódico senoidal com interpolação da tabela CMSIS, evitando a redução de argumento da biblioteca matemática na rotina de FFB de 1 kHz. O erro de interpolação validado fica abaixo de 0,7 contagem de torque em amplitude máxima.
- Mantém regressões para coerência e serialização do snapshot do encoder e acrescenta cobertura para a precisão do seno interpolado.

### Validação

- Os 62 testes de contrato/regressão passaram.
- O CI compilou o firmware ARM para STM32F405 com sucesso.
- O firmware foi testado no volante.

## English

### Force Feedback and performance

- Reduces encoder motion snapshot publication cost in the control interrupt by removing unnecessary exclusive atomic operations from the ISR path.
- Calculates periodic sine effects with CMSIS table interpolation, avoiding math-library argument reduction in the 1 kHz FFB routine. Validated interpolation error stays below 0.7 torque counts at full amplitude.
- Keeps regressions for encoder snapshot coherence and writer serialization, and adds coverage for interpolated sine accuracy.

### Validation

- All 62 contract/regression tests passed.
- CI successfully built the STM32F405 ARM firmware.
- The firmware was tested on the wheel.
