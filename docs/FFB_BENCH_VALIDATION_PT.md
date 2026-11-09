# Validação de bancada do Force Feedback

Este roteiro valida alterações no cálculo do FFB e compara a carga da tarefa de 1 kHz. Use torque conservador, volante mecanicamente seguro e interrompa o teste se houver força inesperada, ruído novo ou aquecimento anormal.

## Preparação

1. Anote a versão/hash do firmware, o jogo ou laboratório usado, o ganho FFB e os limites de torque configurados. Use v1.0.6 como referência publicada e compare com o binário candidato atual.
2. Conecte a CDC do volante e consulte `sys.fxtest?`. Confirme `ffb=1` e que `slots`/contadores HID correspondem ao efeito enviado.
3. Consulte `sys.ffbperf!` para zerar os máximos e contadores em microssegundos e ciclos antes de cada cenário.

## Cenários

Execute cada cenário por pelo menos 60 segundos e consulte `sys.ffbperf?` ao final.
Consulte também `sys.ffbcycles?` para obter os máximos do cálculo (`C`) e do loop (`L`) em ciclos do processador; essa resolução permite comparar pequenas reduções que não aparecem em microssegundos.

1. **Periódico senoidal:** período de 500 ms, amplitude baixa, fases 0°, 90°, 180° e 270°. Confirme a mudança de fase e a repetição sem saltos no contorno do ciclo.
2. **Outras formas periódicas:** triângulo, serra ascendente, serra descendente e quadrada, com o mesmo período. Confirme forma, sinal e repetição.
3. **Atrito perto do centro:** aplique velocidade lentamente nos dois sentidos, atravessando zero e a faixa de ramp-up. Confirme que a força entra e sai suavemente, sem inversão ou degrau audível/tátil.
4. **Curva de posição:** compare expo desligado com expo negativo e positivo usando a mesma posição física. Confirme o sentido da força e uma transição contínua pelo centro.
5. **Desligamento:** pare o efeito e depois desconecte o USB. Confirme que o torque residual retorna a zero.

## Registro de timing

`sys.ffbperf?` retorna:

- `L`: último e máximo tempo total do loop, em microssegundos.
- `C`: máximo do cálculo e commit de torque.
- `W` / `H`: espera e tempo segurando o mutex de estado.
- `O`: loops que excederam 1 ms.
- `S`: último e máximo intervalo entre amostras.
- `P`: intervalos de amostra que excederam 1 ms.

Para cada cenário, registre a resposta completa, incluindo `O` e `P`. O alvo é `O=0`, `P=0` e `L` abaixo de 1000 µs. Compare o firmware candidato e o firmware de referência com a mesma placa, configuração, laboratório e sequência de efeitos. Anote também qualquer ruído, atraso percebido, oscilação ou diferença na força.

## Resultado a devolver

Envie a versão testada, os parâmetros usados, a resposta de `sys.fxtest?` e de `sys.ffbperf?` para cada cenário, e descreva qualquer diferença tátil. Isso separa falhas de entrega HID de alterações no cálculo e permite comparar latência medida na placa.
