/*
 * gpio_axis_proc.h — port simplificado da classe AnalogAxisProcessing do
 * OpenFFBoard. Diferente do original, esta versao foca SO no Biquad filter
 * e no flag autocal (estado global).
 *
 * Decisao de design: o autocal real (atualizar min/max) acontece DENTRO
 * de gpio_inputs.cpp, escrevendo direto nos campos AMIN/AMAX existentes
 * (ja persistidos em EE por GPIO via ADR_GPIO*_AMIN/AMAX). Assim:
 *   - Sem duplicacao de armazenamento (RAM duplicada)
 *   - Persistencia automatica via sys.save (ja cobre AMIN/AMAX)
 *   - UI dos GPIO cards mostra os valores live (sem widget extra)
 *   - Coerencia conceitual: AMIN/AMAX e o min/max, ponto.
 *
 * Esta classe fornece:
 *   - 4 Biquads (1 por canal HID — RX/RY/RZ/Slider)
 *   - Filter on/off global
 *   - Cutoff Hz configuravel
 *   - Autocal on/off global (so o flag — logica de update vive no gpio_inputs)
 */

#ifndef INC_GPIO_AXIS_PROC_H_
#define INC_GPIO_AXIS_PROC_H_

#include <stdint.h>

#ifdef __cplusplus

#include "Filters.h"
#include "mutex.hpp"

#define AXIS_PROC_CHANNELS 4

class GpioAxisProcessor {
public:
    static GpioAxisProcessor& instance();

    void init();

    // Filter pipeline: recebe raw (0..4095), retorna raw filtrado.
    // Se filter desabilitado, retorna raw inalterado. ch eh o canal HID
    // (0..3 = RX/RY/RZ/Slider) — cada canal tem seu Biquad independente.
    uint16_t filterRaw(uint8_t ch, uint16_t raw);

    // Estado
    bool isFilterEnabled() const;
    void setFilterEnabled(bool en);

    bool isAutorangeEnabled() const;
    void setAutorangeEnabled(bool en);

    float getFilterFreq() const;
    void  setFilterFreq(float hz);

private:
    GpioAxisProcessor();
    void setupFilters();

    bool  filter_enabled_;
    bool  autorange_enabled_;
    float filter_freq_hz_;
    mutable cpp_freertos::MutexStandard state_mutex_;
    static constexpr float SAMPLE_RATE_HZ = 1000.0f;
    static constexpr float DEFAULT_FREQ_HZ = 60.0f;
    static constexpr float FILTER_Q = 0.5f;

    Biquad filters_[AXIS_PROC_CHANNELS];
};

#endif  // __cplusplus

// ===== C-callable API pra cmd_table.cpp e gpio_inputs.cpp =====
#ifdef __cplusplus
extern "C" {
#endif

void     axis_proc_init(void);
uint16_t axis_proc_filter_raw(uint8_t ch, uint16_t raw);

int   axis_proc_get_filter_enabled(void);
void  axis_proc_set_filter_enabled(int en);
int   axis_proc_get_autorange_enabled(void);
void  axis_proc_set_autorange_enabled(int en);
float axis_proc_get_filter_freq(void);
void  axis_proc_set_filter_freq(float hz);

#ifdef __cplusplus
}
#endif

#endif /* INC_GPIO_AXIS_PROC_H_ */
