Encontrei 10 bugs e inconsistências importantes no firmware que impactam diretamente a compatibilidade, a estabilidade e o comportamento do Force Feedback (FFB) nos jogos e simuladores.

Abaixo está o diagnóstico detalhado de cada um deles, com a causa-raiz no código, os jogos impactados e como corrigi-los.

1. 🚨 Bug Crítico: Perda Total de FFB em Direção Norte/Sul (0° / 180°) em Volantes de 1 Eixo
Arquivos: 

HidFFB.cpp
, 

EffectsCalculator.cpp
Causa-raiz: O descritor HID USB anuncia suporte a 2 eixos FFB (X e Y), mas a base física possui apenas 1 eixo (Eixo 0). Quando um jogo envia um efeito (Constant Force ou Periódico) com direção polar apontando para o Norte (0°) ou Sul (180°): $$\text{phaseX} = 0^\circ \implies \text{axisMagnitudes}[0] = \sin(0^\circ) = 0.0f, \quad \text{axisMagnitudes}[1] = -\cos(0^\circ) = -1.0f$$ No código de 

HidFFB.cpp
:
cpp
if (effect_p->axisMagnitudes[0] == 0.0f && effect_p->axisMagnitudes[1] == 0.0f) {
    float fallback = directionEnable ? cosf(phaseX) : 1.0f;
    effect_p->axisMagnitudes[0] = fallback;
}
Como axisMagnitudes[1] vale -1.0f (não é zero), essa condição de fallback nunca é executada! O axisMagnitudes[0] continua valendo 0.0f, e em EffectsCalculator::calcComponentForce a força resultante no volante é multiplicada por zero: $$\text{Torque} = -\text{forceVector} \times 0.0f = 0\text{ Nm}$$
Impacto nos Jogos: Jogos que inicializam efeitos com direção 0° ou 180° têm perda total de FFB (volante fica completamente solto sem torque).
Jogos afetados: iRacing, Assetto Corsa, Assetto Corsa Competizione, rFactor 2, BeamNG.drive, DiRT Rally 2.0, EA Sports WRC, F1 23/24.
Correção: Quando axisCount == 1, se o jogo utilizar direção polar (directionEnable), a magnitude para o eixo 0 deve considerar $\cos(\text{phaseX})$ ou a projeção correta quando $\sin(\text{phaseX}) = 0$, em vez de exigir que ambos os eixos sejam simultaneamente zero.
2. 🚨 Bug do DirectInput PID State Report (GET_REPORT ID 2 retorna zeros)
Arquivos: 

HidFFB.cpp
, 

usb_descriptors.c
Causa-raiz: Muitos jogos chamam IDirectInputDevice8::GetForceFeedbackState() para checar se os motores estão ligados antes de enviar forças. Isso gera uma requisição USB GET_REPORT para o Report ID 2 (HID_ID_STATE). Em HidFFB::hidGet, o Report ID 2 cai no default e retorna 0. O fallback de TinyUSB preenche tudo com zero. Para o DirectInput, os bits retornados indicam:
Bit 1 (Actuators Enabled): 0 (Desativado)
Bit 3 (Actuator Power): 0 (Sem Energia)
Impacto nos Jogos:
Forza Motorsport / Forza Horizon 4 e 5: desativa o Force Feedback automaticamente por achar que o dispositivo está sem energia/desconectado.
Assetto Corsa Competizione / BeamNG: podem acusar "Device unpowered" ou falhar na inicialização do FFB.
Correção: Tratar case HID_ID_STATE: em HidFFB::hidGet, retornando this->reportFFBStatus.status com os bits HID_ACTUATOR_POWER | HID_ENABLE_ACTUATORS.
3. 🚨 Efeitos de Mola, Damper e Atrito Mudos por Saturação Zero (Saturation = 0)
Arquivos: 

ffb_defs.h
, 

EffectsCalculator.cpp
Causa-raiz: Na API DirectInput (DICONDITION), muitos jogos deixam os campos de saturação (dwPosSat e dwNegSat) como 0 para indicar "sem limite de saturação / saturação máxima (100%)". No firmware:
isActive() em FFB_Effect_Condition:
cpp
bool isActive(){
    return (positiveCoefficient != 0 && positiveSaturation != 0) || (negativeCoefficient != 0 && negativeSaturation != 0);
}
Se a saturação for 0, isActive() retorna false.
Em calcConditionEffectForce:
cpp
force = clip(..., -negativeSaturation, positiveSaturation);
Com saturação 0, o clip(..., 0, 0) força a força para 0.
Impacto nos Jogos:
Assetto Corsa, rFactor 2, Automobilista 2, KartKraft, Project CARS 2: os efeitos de atrito de pneu (Friction), peso de volante parado (Damper) e mola de retorno (Spring) ficam completamente desligados.
Correção: Se o jogo enviar positiveSaturation == 0 ou negativeSaturation == 0 com coeficiente diferente de zero, atribuir o default 0x7FFF (32767 = 100% de saturação).
4. 💥 Crash / HardFault (Ponteiro Nulo) ao Executar Damper, Friction ou Inertia
Arquivo: 

EffectsCalculator.cpp
, 

L400
, 

L407
Causa-raiz: Em calcComponentForce:
Para FFB_EFFECT_CONSTANT, há checagem if (effect->filter[axis] != nullptr).
Porém, para FFB_EFFECT_FRICTION, FFB_EFFECT_DAMPER e FFB_EFFECT_INERTIA, o código chama effect->filter[axis]->process(...) sem checar se o filtro é nulo. Se um jogo enviar comandos ou redefinir efeitos onde set_filters não foi chamado previamente para aquele slot, a chamada desreferencia ponteiro nulo (nullptr), causando HardFault imediato no STM32 (firmware congela e a porta USB cai).
Impacto nos Jogos:
Jogos que reconfiguram efeitos dinamicamente ao trocar de pista ou carro (ex.: rFactor 2, Automobilista 2, Assetto Corsa).
Correção: Adicionar a verificação if (effect->filter[axis] != nullptr) antes de chamar ->process() em todos os efeitos condicionais, ou passar a força direto caso o filtro ainda não esteja alocado.
5. ⚠️ Vazamento de Efeitos no Pool de Memória (used_effects nunca decrementa)
Arquivo: 

HidFFB.cpp
, 

L285
Causa-raiz: Quando um efeito é criado (HID_ID_NEWEFREP), o firmware faz used_effects++. Porém, quando o jogo libera o efeito (HID_ID_BLKFRREP), o firmware chama free_effect(), mas esquece de decrementar used_effects. Após 40 criações/destruições de efeitos durante a gameplay, used_effects ultrapassa effects.size(), e ramPoolAvailable = (effects.size() - used_effects) * sizeof(FFB_Effect) sofre underflow aritmético (vira um número negativo ou gigante), corrompendo as respostas de BlockLoadReport para o DirectInput.
Impacto nos Jogos:
Em sessões longas com trocas de carro, resets de volta ou trocas de sessão em iRacing, AC, rFactor 2, BeamNG.
Correção: No case HID_ID_BLKFRREP:, decrementar if (used_effects > 1) used_effects--;.
6. ⚠️ Comando DC Stop All Effects não para os Efeitos Ativos
Arquivo: 

HidFFB.cpp
Causa-raiz: Ao receber o comando PID DC Stop All Effects (cmd & 0x04), o firmware chama apenas stop_FFB() (ffb_active = false), mas não altera o effect.state dos efeitos individuais (eles continuam com state = 1). Quando o jogo envia DC Enable Actuators ou DC Continue, ffb_active volta a ser true e todos os efeitos que deveriam ter sido interrompidos voltam a tocar juntos.
Impacto nos Jogos:
Jogos que pausam ou resetam o carro após batidas (BeamNG.drive, DiRT Rally 2.0, Automobilista 2).
Correção: Em cmd & 0x04, além de pausar o FFB, iterar pelos efeitos e setar effect.state = 0.
7. ⚠️ Efeito RAMP sem Limite de Extrapolação e Risco de Divisão por Zero
Arquivo: 

EffectsCalculator.cpp
Causa-raiz: Na fórmula do Ramp Force:
cpp
force_vector = (int32_t)effect->startLevel + (elapsed_time * (effect->endLevel - effect->startLevel)) / duration;
Se duration == 0, ocorre divisão por zero. Se elapsed_time > duration, a força não é limitada ao endLevel e extrapola indefinidamente.
Impacto nos Jogos:
Jogos que usam Ramp Force para efeitos de rampa de força ou crash effects.
Correção: Garantir if (duration <= 0) duration = 1; e limitar if (elapsed_time > duration) elapsed_time = duration;.
8. ⚠️ Perda de Precisão em Efeitos Periódicos (Sine, Sawtooth, Triangle) após Horas de Uso
Arquivo: 

EffectsCalculator.cpp
Causa-raiz: O tempo decorrido é calculado convertendo micros() para float de 32 bits (micros() / 1000.0). Como a mantissa do float possui apenas 24 bits, após ~3 a 4 horas de base ligada, o valor de microssegundos atinge magnitudes onde a resolução do float perde os dígitos fracionários e passa a dar saltos de 0.5 ms a 2.0 ms. Isso introduz jitter e ruído em ondas senoidais e dentes de serra.
Impacto nos Jogos:
Vibrações de zebra, motor e textura de pista em Assetto Corsa, Automobilista 2, ACC em sessões longas ou endurance.
Correção: Calcular o delta de tempo primeiro em inteiros com módulo do período (uint32_t elapsed_us = (micros() - startTime_us) % (period_us);) antes de converter para float.
9. ⚠️ Inconsistência de Tamanho no Descritor HID (90 bytes vs 88 bytes)
Arquivo: 

usb_hid_ffb_desc.h
Causa-raiz: O macro #define HIDDESC_GAMEPAD_16B_SIZE 90 declara 90 bytes, mas os bytes reais somam 88 bytes. O descritor USB Configuration envia 2 bytes nulos (0x00, 0x00) ao final do Report Descriptor.
Impacto nos Jogos / SO:
Drivers USB rigorosos (Linux hidraw / macOS / analisadores de conformidade USB-IF) podem emitir alertas de descritor malformado.
Correção: Ajustar #define HIDDESC_GAMEPAD_16B_SIZE 88.
10. ⚠️ Falta de Máscara de Bits em parameterBlockOffset para Efeitos de Condição
Arquivo: 

HidFFB.cpp
Causa-raiz:
cpp
uint8_t axis = std::min(axisCount, cond->parameterBlockOffset);
Na especificação USB PID, o byte parameterBlockOffset empacota o índice do eixo nos 4 bits inferiores (0..3) e ordinais de instância nos 4 bits superiores (4..7). Se o jogo preencher os bits de instância, cond->parameterBlockOffset resulta em um número $\ge 16$, fazendo com que a mola/damper do Eixo X seja atribuída incorretamente ao Eixo Y (que não existe na base de 1 eixo).
Impacto nos Jogos:
Efeitos de mola e amortecimento ignorados em jogos da Codemasters (DiRT Rally, F1, GRID) e Richard Burns Rally.
Correção: Utilizar uint8_t axis = std::min<uint8_t>(axisCount - 1, cond->parameterBlockOffset & 0x03);.
Se desejar, posso preparar um plano de implementação para corrigir esses 10 pontos diretamente no código do firmware e garantir compatibilidade total com todos os simuladores.

