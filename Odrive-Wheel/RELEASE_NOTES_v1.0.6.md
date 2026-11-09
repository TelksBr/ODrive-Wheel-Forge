# ODrive Wheel Forge v1.0.6

## Português

### Force Feedback

- Preserva valores fracionários dos efeitos até a soma final, evitando que efeitos simultâneos de baixa intensidade sejam descartados antes da aplicação do ganho global.
- Mantém a interpolação fracionária de rampas e envelopes de ataque e fade, reduzindo degraus em efeitos de baixa amplitude.
- Aplica o ganho global uma vez por amostra e limita o torque depois da soma dos efeitos.

### Validação

- Os 66 testes de contrato/regressão passaram.
- O build ARM para STM32F405 foi concluído sem warnings nem erros.
- O firmware foi testado no volante e o Force Feedback funcionou corretamente.

## English

### Force Feedback

- Preserves fractional effect values through the final sum, preventing simultaneous low-intensity effects from being discarded before global gain is applied.
- Keeps ramp and attack/fade envelope interpolation fractional, reducing steps in low-amplitude effects.
- Applies global gain once per sample and limits torque after summing the effects.

### Validation

- All 66 contract/regression tests passed.
- The STM32F405 ARM build completed without warnings or errors.
- The firmware was tested on the wheel and Force Feedback worked correctly.
