# ODrive Wheel Forge v1.0.8

## Português

### Correções de Force Feedback

- Aplica o HID Set Effect Gain também a spring, damper, friction e inertia.
- `Pause` congela os efeitos no tempo atual: duração finita, atraso inicial e fase periódica retomam corretamente com `Continue`.
- O relatório de estado PID agora distingue atuadores habilitados, dispositivo pausado e efeito em execução; consultas GET atualizam o estado antes de responder.
- O `Reset` PID limpa a pausa, remove os efeitos e reativa os atuadores.
- O atraso de início é consumido uma vez por operação Start e não reinicia ao mudar o tipo do efeito.
- A fase de efeitos periódicos infinitos permanece contínua na volta do tick HAL; mudanças de frequência preservam a fração da fase.
- A primeira amostra de velocidade não gera um impulso artificial de inércia.

### Validação

- 82 testes de contrato/regressão passaram.
- Build ARM para STM32F405 concluído com sucesso; artefatos `.bin`, `.elf` e `.hex` anexados.
- A nova semântica de Pause/Continue ainda precisa de validação no volante.

## English

### Force Feedback fixes

- Applies HID Set Effect Gain to spring, damper, friction, and inertia effects.
- `Pause` freezes playback time: finite duration, start delay, and periodic phase resume correctly with `Continue`.
- The PID state report now distinguishes enabled actuators, paused device, and playing effect; GET queries refresh the state before responding.
- PID `Reset` clears pause, removes effects, and re-enables actuators.
- Start delay is consumed once per Start operation and is not reset when an effect type changes.
- Infinite periodic phase remains continuous across HAL tick rollover; frequency changes preserve the phase fraction.
- The first velocity sample no longer creates an artificial inertia impulse.

### Validation

- All 82 contract/regression tests passed.
- STM32F405 ARM build completed successfully; `.bin`, `.elf`, and `.hex` artifacts are attached.
- The new Pause/Continue behavior still needs validation on the wheel.
