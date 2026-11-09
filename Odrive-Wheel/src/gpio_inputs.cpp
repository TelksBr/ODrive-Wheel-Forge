// gpio_inputs.cpp — implementa GPIOs 1-4 como botões/eixos do joystick HID.
// Ver gpio_inputs.h pro contrato. Usa ADC1 do ODrive (já configurado em
// round-robin DMA) via get_adc_relative_voltage_ch().

#include "gpio_inputs.h"
#include "eeprom_addresses.h"
#include "flash_helpers.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

// Não dá pra incluir low_level.h direto: ele declara funções com Stm32Gpio
// (classe C++) e arrasta dependências de drivers. Só preciso de uma função
// que usa channel (uint16_t), declaro aqui. Símbolo no .o é C-mangled.
extern "C" float get_adc_relative_voltage_ch(uint16_t channel);

// Pra modo ZEROWHEEL — chama o mesmo handler do botão Zero do tool.
// Captura zeroOffset_ em RAM (NÃO persiste; user salva manualmente).
extern "C" void ffb_axis_zeroenc(void);

// Axis processor (Biquad filter + autorange + min/max manual). Roteia
// reads de GPIOs em modo AXIS pelo processor quando filter ou autorange
// esta habilitado (config global, valida pra todos os 4 canais).
#include "gpio_axis_proc.h"

extern "C" {
#include "eeprom.h"
}

#include <atomic>
#include <string.h>

// -------------------- Pinout (hardcoded pra MKS XDrive Mini / ODrive v3.6) --------------------
// idx 0..4 → GPIO 1, 2, 3, 4, 6 → PA0..PA3, PB2
// adc_channel = 0xFFFF marca pino sem ADC (digital only, axis mode bloqueado).
struct gpio_pin_t {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint16_t adc_channel;   // pra get_adc_relative_voltage_ch (0xFFFF = sem ADC)
};

#define GPIO_NO_ADC  0xFFFF

static const gpio_pin_t s_pins[GPIO_INPUTS_COUNT] = {
    { GPIOA, GPIO_PIN_0,      0 },           // GPIO 1 → PA0 → ADC1_IN0
    { GPIOA, GPIO_PIN_1,      1 },           // GPIO 2 → PA1 → ADC1_IN1
    { GPIOA, GPIO_PIN_2,      2 },           // GPIO 3 → PA2 → ADC1_IN2
    { GPIOA, GPIO_PIN_3,      3 },           // GPIO 4 → PA3 → ADC1_IN3
    { GPIOB, GPIO_PIN_2, GPIO_NO_ADC },      // GPIO 6 → PB2 → digital only
};

// Mapeamento ASCII inst (1, 2, 3, 4, 6) → idx0 interno (0..4).
// Inst 5 retorna -1 (não suportado — PC4 não exposto no header MKS).
static inline int inst_to_idx0(uint8_t inst) {
    switch (inst) {
        case 1: return 0;
        case 2: return 1;
        case 3: return 2;
        case 4: return 3;
        case 6: return 4;
        default: return -1;
    }
}

// -------------------- Config em RAM --------------------
struct gpio_cfg_t {
    uint8_t  mode;     // GPIO_INPUT_DISABLED/BUTTON/AXIS
    uint8_t  idx;      // 0..63 (button) ou 0..3 (axis: 0=RX, 1=RY, 2=RZ, 3=Slider)
    uint8_t  invert;   // 0/1
    uint16_t amin;     // ADC raw 0..4095 (axis only)
    uint16_t amax;     // ADC raw 0..4095 (axis only)
};

static gpio_cfg_t s_cfg[GPIO_INPUTS_COUNT];
static_assert(std::atomic<osMutexId>::is_always_lock_free,
              "GPIO config mutex handle must be published lock-free");
static std::atomic<osMutexId> s_gpio_cfg_mutex{nullptr};

class GpioConfigGuard {
public:
    explicit GpioConfigGuard(const std::atomic<osMutexId>& mutex)
        : mutex_(mutex.load(std::memory_order_acquire)),
          locked_(mutex_ != nullptr && osMutexWait(mutex_, osWaitForever) == osOK) {}
    ~GpioConfigGuard() {
        if (locked_) osMutexRelease(mutex_);
    }
    explicit operator bool() const { return locked_; }

private:
    osMutexId mutex_;
    bool locked_;
};

// Cache do último valor filtrado por GPIO (modo AXIS). Atualizado no update
// loop quando o axis processor processa. Usado por gpio.N.filt? pra UI.
// 0xFFFF = nunca foi atualizado (GPIO não está em modo AXIS).
static uint16_t s_last_filtered[GPIO_INPUTS_COUNT] = { 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF };

// -------------------- Endereços EE por GPIO (helpers) --------------------
// Mesma ordem do s_pins / inst_to_idx0: GPIO 1, 2, 3, 4, 6.
static const uint16_t s_addr_cfg[GPIO_INPUTS_COUNT]  = { ADR_GPIO1_CFG,  ADR_GPIO2_CFG,  ADR_GPIO3_CFG,  ADR_GPIO4_CFG,  ADR_GPIO6_CFG  };
static const uint16_t s_addr_amin[GPIO_INPUTS_COUNT] = { ADR_GPIO1_AMIN, ADR_GPIO2_AMIN, ADR_GPIO3_AMIN, ADR_GPIO4_AMIN, ADR_GPIO6_AMIN };
static const uint16_t s_addr_amax[GPIO_INPUTS_COUNT] = { ADR_GPIO1_AMAX, ADR_GPIO2_AMAX, ADR_GPIO3_AMAX, ADR_GPIO4_AMAX, ADR_GPIO6_AMAX };

// Empacota mode/idx/invert em uint16
static inline uint16_t pack_cfg(uint8_t mode, uint8_t idx, uint8_t invert) {
    return (uint16_t)((mode & 0x03) | ((idx & 0x3F) << 2) | ((invert & 1) << 8));
}
static inline void unpack_cfg(uint16_t p, uint8_t *mode, uint8_t *idx, uint8_t *invert) {
    *mode   = (uint8_t)(p & 0x03);
    *idx    = (uint8_t)((p >> 2) & 0x3F);
    *invert = (uint8_t)((p >> 8) & 1);
}

// -------------------- Pin configuration (HAL) --------------------
// Reconfigura o pino físico baseado no s_cfg[idx0]. Chamado de init e dos
// setters de mode.
static void apply_pin_mode(int idx0) {
    if (idx0 < 0 || idx0 >= GPIO_INPUTS_COUNT) return;
    const gpio_pin_t *p = &s_pins[idx0];
    GPIO_InitTypeDef init = {};
    init.Pin = p->pin;
    init.Speed = GPIO_SPEED_FREQ_LOW;

    switch (s_cfg[idx0].mode) {
        case GPIO_INPUT_BUTTON:
        case GPIO_INPUT_ZEROWHEEL:    // mesma config de pin: digital pull-up
            init.Mode = GPIO_MODE_INPUT;
            init.Pull = GPIO_PULLUP;   // botão pra GND quando pressionado
            HAL_GPIO_Init(p->port, &init);
            break;
        case GPIO_INPUT_AXIS:
            init.Mode = GPIO_MODE_ANALOG;
            init.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(p->port, &init);
            break;
        case GPIO_INPUT_DISABLED:
        default:
            // Phase 4.x fix: quando DISABLED, NÃO reconfigurar o pino. Preserva
            // a configuração que o ODrive setou via axis0.config.gpioN_mode
            // (importante pra modos ANALOG_IN usados pelo motor_thermistor).
            // O antigo "forçar GPIO_MODE_INPUT" quebrava o termistor: pino ia
            // pra digital input, ADC lia garbage, polinômio retornava sempre c0.
            // Se nada antes configurou o pino, ele permanece no reset state
            // (analog hi-Z no STM32F405) — também seguro.
            break;
    }
}

// -------------------- Init --------------------
extern "C" void gpio_inputs_init(void) {
    osMutexDef(gpioCfgMutex);
    osMutexId mutex = s_gpio_cfg_mutex.load(std::memory_order_acquire);
    if (mutex == nullptr) {
        mutex = osMutexCreate(osMutex(gpioCfgMutex));
        if (mutex != nullptr) {
            s_gpio_cfg_mutex.store(mutex, std::memory_order_release);
        }
    }
    configASSERT(mutex != nullptr);
    if (mutex == nullptr) return;
    GpioConfigGuard guard(s_gpio_cfg_mutex);
    if (!guard) return;

    // Defaults: tudo disabled, calibração razoável (0.16V .. 3.07V)
    for (int i = 0; i < GPIO_INPUTS_COUNT; i++) {
        s_cfg[i].mode = GPIO_INPUT_DISABLED;
        s_cfg[i].idx = (uint8_t)i;       // botão 0/1/2/3 default
        s_cfg[i].invert = 0;
        s_cfg[i].amin = 200;
        s_cfg[i].amax = 3800;
    }

    // Tenta carregar da EE. Cada slot que vier 0xFFFF (não existe) ignoramos.
    uint16_t v;
    for (int i = 0; i < GPIO_INPUTS_COUNT; i++) {
        if (Flash_Read(s_addr_cfg[i], &v, false) && v != 0xFFFF) {
            uint8_t m, idx, inv;
            unpack_cfg(v, &m, &idx, &inv);
            // Sanity: mode válido (0..3 = DISABLED, BUTTON, AXIS, ZEROWHEEL)
            if (m <= GPIO_INPUT_ZEROWHEEL) {
                s_cfg[i].mode = m;
                s_cfg[i].idx = idx;
                s_cfg[i].invert = inv;
            }
        }
        if (Flash_Read(s_addr_amin[i], &v, false) && v != 0xFFFF) {
            s_cfg[i].amin = v;
        }
        if (Flash_Read(s_addr_amax[i], &v, false) && v != 0xFFFF) {
            s_cfg[i].amax = v;
        }
    }

    // Configura todos os pinos baseado no cfg final
    for (int i = 0; i < GPIO_INPUTS_COUNT; i++) {
        apply_pin_mode(i);
    }
}

// -------------------- Save --------------------
extern "C" int gpio_inputs_save(int *writes_out, int *errors_out) {
    gpio_cfg_t config_snapshot[GPIO_INPUTS_COUNT];
    {
        GpioConfigGuard guard(s_gpio_cfg_mutex);
        if (!guard) return 0;
        memcpy(config_snapshot, s_cfg, sizeof(config_snapshot));
    }

    int writes = 0, errors = 0;
    for (int i = 0; i < GPIO_INPUTS_COUNT; i++) {
        if (!Flash_Write(s_addr_cfg[i],
                          pack_cfg(config_snapshot[i].mode, config_snapshot[i].idx, config_snapshot[i].invert))) errors++;
        writes++;
        if (!Flash_Write(s_addr_amin[i], config_snapshot[i].amin)) errors++;
        writes++;
        if (!Flash_Write(s_addr_amax[i], config_snapshot[i].amax)) errors++;
        writes++;
    }
    if (writes_out) *writes_out = writes;
    if (errors_out) *errors_out = errors;
    return errors == 0 ? 1 : 0;
}

// -------------------- Update report (chamado a 1 kHz) --------------------
// Dado raw ADC 0..4095, escala pra -32767..+32767 baseado em [amin, amax]
static inline int16_t scale_axis(uint16_t raw, uint16_t amin, uint16_t amax, uint8_t invert) {
    // Sanity: amin < amax. Se invertido ou igual, retorna 0 (deadcenter).
    if (amin >= amax) return 0;
    int32_t r = (int32_t)raw;
    if (r <= (int32_t)amin) r = amin;
    if (r >= (int32_t)amax) r = amax;
    // Mapeia [amin..amax] → [-32767..+32767]
    int32_t span = (int32_t)amax - (int32_t)amin;     // > 0
    int32_t pos = r - (int32_t)amin;                  // 0..span
    int32_t scaled = (pos * 65534) / span - 32767;    // -32767..+32767
    if (invert) scaled = -scaled;
    if (scaled > 32767) scaled = 32767;
    if (scaled < -32767) scaled = -32767;
    return (int16_t)scaled;
}

extern "C" void gpio_inputs_update_report(uint64_t *buttons,
                                           int16_t *RX, int16_t *RY,
                                           int16_t *RZ, int16_t *Slider) {
    if (!buttons) return;
    GpioConfigGuard guard(s_gpio_cfg_mutex);
    if (!guard) return;

    // Edge detection per-GPIO pra modo ZEROWHEEL.
    // Inicializa true = "estava solto" → primeira leitura como pressed dispara.
    // Reset implícito: ao mudar pra outro modo, próximo set_mode chama
    // apply_pin_mode que não toca aqui, mas a transição de mode no s_cfg
    // já desliga a lógica do botão. State stale = ok (não dispara duplicado).
    static bool s_zerowheel_was_high[GPIO_INPUTS_COUNT] = { true, true, true, true, true };

    for (int i = 0; i < GPIO_INPUTS_COUNT; i++) {
        // c eh REFERENCIA mutavel pra permitir autocal escrever em amin/amax
        // diretamente. Antes era const&; o autocal precisa atualizar o estado.
        gpio_cfg_t &c = s_cfg[i];
        if (c.mode == GPIO_INPUT_BUTTON) {
            // Botão pra GND com pull-up: pressionado = nível 0.
            // Se invert=1, lógica inversa (active high).
            GPIO_PinState st = HAL_GPIO_ReadPin(s_pins[i].port, s_pins[i].pin);
            bool pressed = (st == GPIO_PIN_RESET);    // active low default
            if (c.invert) pressed = !pressed;
            if (pressed && c.idx < 64) {
                *buttons |= ((uint64_t)1 << c.idx);
            }
        } else if (c.mode == GPIO_INPUT_ZEROWHEEL) {
            // Edge detection high→low (botão pressionado pra GND).
            // Dispara ffb_axis_zeroenc() uma vez na borda, não enquanto segura.
            // invert=1 inverte (trigger no release em vez do press).
            GPIO_PinState st = HAL_GPIO_ReadPin(s_pins[i].port, s_pins[i].pin);
            bool isHigh = (st == GPIO_PIN_SET);
            bool isHigh_logical = c.invert ? !isHigh : isHigh;
            // Trigger no edge: estava high (idle, pull-up) → agora low (pressed)
            if (s_zerowheel_was_high[i] && !isHigh_logical) {
                ffb_axis_zeroenc();
            }
            s_zerowheel_was_high[i] = isHigh_logical;
        } else if (c.mode == GPIO_INPUT_AXIS) {
            // Defesa: se pino sem ADC chegou aqui (não deveria, set_mode bloqueia),
            // ignora pra não chamar get_adc_relative_voltage_ch com canal inválido.
            if (s_pins[i].adc_channel == GPIO_NO_ADC) continue;
            // ADC1 round-robin do ODrive: get_adc_relative_voltage_ch retorna
            // 0.0 .. 1.0 (= 0 .. 4095 / 4095). Volta pra raw.
            float rel = get_adc_relative_voltage_ch(s_pins[i].adc_channel);
            if (rel < 0) rel = 0;
            if (rel > 1) rel = 1;
            uint16_t raw = (uint16_t)(rel * 4095.0f + 0.5f);

            // Pipeline:
            //   1. (opcional) Biquad filter — usa c.idx (canal HID 0..3) pra
            //      indexar o filtro do processor
            //   2. (opcional) autocal — aprende AMIN/AMAX em runtime
            //   3. scale_axis pro range HID (-32767..+32767) usando c.amin/c.amax
            uint16_t filt = axis_proc_filter_raw((uint8_t)c.idx, raw);
            s_last_filtered[i] = filt;  // cache pra gpio.N.filt?

            // Autocal: atualiza AMIN/AMAX direto em RAM. Quando user
            // clicar Save, persiste em flash via slots ADR_GPIO*_AMIN/AMAX
            // ja existentes. Sem duplicacao de estado.
            if (axis_proc_get_autorange_enabled()) {
                if (filt < c.amin) c.amin = filt;
                if (filt > c.amax) c.amax = filt;
            }

            int16_t v = scale_axis(filt, c.amin, c.amax, c.invert);

            switch (c.idx) {
                case 0: if (RX)     *RX     = v; break;
                case 1: if (RY)     *RY     = v; break;
                case 2: if (RZ)     *RZ     = v; break;
                case 3: if (Slider) *Slider = v; break;
                default: break;
            }
        }
        // GPIO_INPUT_DISABLED: nothing
    }
}

// -------------------- Setters/getters pra ASCII --------------------
// idx0_from_inst() agora delega pro mapeamento descontínuo definido no
// topo do arquivo (inst_to_idx0). inst=5 retorna -1 (inválido).
static inline int idx0_from_inst(uint8_t inst) {
    return inst_to_idx0(inst);
}

extern "C" uint8_t gpio_inputs_get_mode(uint8_t inst) {
    int i = idx0_from_inst(inst);
    if (i < 0) return 0;
    GpioConfigGuard guard(s_gpio_cfg_mutex);
    return guard ? s_cfg[i].mode : 0;
}
extern "C" int gpio_inputs_set_mode(uint8_t inst, uint8_t mode) {
    int i = idx0_from_inst(inst);
    if (i < 0 || mode > GPIO_INPUT_ZEROWHEEL) return -1;
    // Bloqueia mode=AXIS em pinos sem ADC (ex: GPIO 6 = PB2).
    // ZEROWHEEL é digital → funciona em qualquer GPIO (com ou sem ADC).
    if (mode == GPIO_INPUT_AXIS && s_pins[i].adc_channel == GPIO_NO_ADC) return -1;
    GpioConfigGuard guard(s_gpio_cfg_mutex);
    if (!guard) return -1;
    if (s_cfg[i].mode != mode) {
        s_cfg[i].mode = mode;
        apply_pin_mode(i);
    }
    return 0;
}

extern "C" uint8_t gpio_inputs_get_idx(uint8_t inst) {
    int i = idx0_from_inst(inst);
    if (i < 0) return 0;
    GpioConfigGuard guard(s_gpio_cfg_mutex);
    return guard ? s_cfg[i].idx : 0;
}
extern "C" int gpio_inputs_set_idx(uint8_t inst, uint8_t idx) {
    int i = idx0_from_inst(inst);
    if (i < 0 || idx > 63) return -1;
    GpioConfigGuard guard(s_gpio_cfg_mutex);
    if (!guard) return -1;
    s_cfg[i].idx = idx;
    return 0;
}

extern "C" uint8_t gpio_inputs_get_invert(uint8_t inst) {
    int i = idx0_from_inst(inst);
    if (i < 0) return 0;
    GpioConfigGuard guard(s_gpio_cfg_mutex);
    return guard ? s_cfg[i].invert : 0;
}
extern "C" int gpio_inputs_set_invert(uint8_t inst, uint8_t inv) {
    int i = idx0_from_inst(inst);
    if (i < 0) return -1;
    GpioConfigGuard guard(s_gpio_cfg_mutex);
    if (!guard) return -1;
    s_cfg[i].invert = inv ? 1 : 0;
    return 0;
}

extern "C" uint16_t gpio_inputs_get_amin(uint8_t inst) {
    int i = idx0_from_inst(inst);
    if (i < 0) return 0;
    GpioConfigGuard guard(s_gpio_cfg_mutex);
    return guard ? s_cfg[i].amin : 0;
}
extern "C" int gpio_inputs_set_amin(uint8_t inst, uint16_t v) {
    int i = idx0_from_inst(inst);
    if (i < 0 || v > 4095) return -1;
    GpioConfigGuard guard(s_gpio_cfg_mutex);
    if (!guard) return -1;
    s_cfg[i].amin = v;
    return 0;
}

extern "C" uint16_t gpio_inputs_get_amax(uint8_t inst) {
    int i = idx0_from_inst(inst);
    if (i < 0) return 0;
    GpioConfigGuard guard(s_gpio_cfg_mutex);
    return guard ? s_cfg[i].amax : 0;
}
extern "C" int gpio_inputs_set_amax(uint8_t inst, uint16_t v) {
    int i = idx0_from_inst(inst);
    if (i < 0 || v > 4095) return -1;
    GpioConfigGuard guard(s_gpio_cfg_mutex);
    if (!guard) return -1;
    s_cfg[i].amax = v;
    return 0;
}

// Retorna o último valor filtrado (após Biquad do axis processor) pra o
// GPIO em modo AXIS. Em counts ADC 0..4095, mesmo formato do raw.
// 0xFFFF se GPIO não está em modo AXIS ou nunca foi processado.
extern "C" uint16_t gpio_inputs_get_filt(uint8_t inst) {
    int i = idx0_from_inst(inst);
    if (i < 0) return 0xFFFF;
    GpioConfigGuard guard(s_gpio_cfg_mutex);
    return guard ? s_last_filtered[i] : 0xFFFF;
}

extern "C" uint16_t gpio_inputs_read_raw(uint8_t inst) {
    int i = idx0_from_inst(inst);
    if (i < 0) return 0xFFFF;
    uint8_t mode;
    {
        GpioConfigGuard guard(s_gpio_cfg_mutex);
        if (!guard) return 0xFFFF;
        mode = s_cfg[i].mode;
    }
    if (mode == GPIO_INPUT_BUTTON || mode == GPIO_INPUT_ZEROWHEEL) {
        // Mesma leitura digital: 1 = pressionado (pino em GND), 0 = solto (pull-up high)
        return HAL_GPIO_ReadPin(s_pins[i].port, s_pins[i].pin) == GPIO_PIN_RESET ? 1 : 0;
    } else if (mode == GPIO_INPUT_AXIS) {
        if (s_pins[i].adc_channel == GPIO_NO_ADC) return 0xFFFF;
        float rel = get_adc_relative_voltage_ch(s_pins[i].adc_channel);
        if (rel < 0) rel = 0;
        if (rel > 1) rel = 1;
        return (uint16_t)(rel * 4095.0f + 0.5f);
    }
    return 0xFFFF;
}
