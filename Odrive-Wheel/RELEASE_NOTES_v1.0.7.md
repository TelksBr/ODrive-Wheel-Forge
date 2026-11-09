# ODrive Wheel Forge v1.0.7

## Português

### Force Feedback e desempenho

- Reduz o custo de cálculo dos efeitos periódicos com conversão de fase inteira equivalente, dobra de ciclo por subtração condicional e escala recíproca pré-calculada.
- Usa a senoide otimizada do CMSIS-DSP no ramp-up do atrito.
- Pré-calcula a curva de expo e o inverso da meia-faixa quando os parâmetros mudam, removendo divisões da amostra de controle de 1 kHz.
- Atualiza métricas de contenção e ciclos fora do mutex de estado para evitar que a telemetria aumente a seção crítica.
- Acrescenta `sys.ffbcycles?` para consultar máximos de ciclos do cálculo e do loop; `sys.ffbperf!` também zera esses máximos.

### Validação

- Os 73 testes de contrato/regressão passaram.
- O build ARM para STM32F405 gerou `odrive-wheel.bin` sem warnings nem erros.
- O firmware foi testado pelo usuário no volante e o funcionamento geral do FFB foi confirmado.
- O roteiro para comparar efeitos e tempos de execução está em `docs/FFB_BENCH_VALIDATION_PT.md`.

## English

### Force Feedback and performance

- Reduces periodic-effect calculation cost with equivalent integer phase conversion, a conditional cycle fold, and a precomputed reciprocal scale.
- Uses the CMSIS-DSP optimized sine for friction ramp-up.
- Precomputes the expo curve and inverse half-range when parameters change, removing divisions from the 1 kHz control sample.
- Updates contention and cycle metrics after releasing the effect-state mutex so instrumentation does not lengthen the critical section.
- Adds `sys.ffbcycles?` to read calculation and loop cycle maxima; `sys.ffbperf!` also clears those maxima.

### Validation

- All 73 contract/regression tests passed.
- The STM32F405 ARM build generated `odrive-wheel.bin` without warnings or errors.
- The user tested the firmware on the wheel and confirmed that FFB works in general.
- The comparison procedure for effects and execution timing is in `docs/FFB_BENCH_VALIDATION_PT.md`.
