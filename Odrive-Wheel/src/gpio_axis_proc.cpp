/*
 * gpio_axis_proc.cpp — implementacao simplificada
 *
 * Apenas Biquad filter por canal HID + flags de estado.
 * Autocal de min/max acontece em gpio_inputs.cpp, escrevendo direto
 * nos AMIN/AMAX existentes por GPIO (persistidos em EE).
 */

#include "gpio_axis_proc.h"

namespace {
class AxisProcGuard {
public:
    explicit AxisProcGuard(cpp_freertos::MutexStandard& mutex)
        : mutex_(mutex), locked_(mutex_.Lock()) {}
    ~AxisProcGuard() {
        if (locked_) mutex_.Unlock();
    }
    explicit operator bool() const { return locked_; }

private:
    cpp_freertos::MutexStandard& mutex_;
    bool locked_;
};
}

// ===== Singleton =====
GpioAxisProcessor& GpioAxisProcessor::instance() {
    static GpioAxisProcessor inst;
    return inst;
}

GpioAxisProcessor::GpioAxisProcessor()
    : filter_enabled_(false)
    , autorange_enabled_(false)
    , filter_freq_hz_(DEFAULT_FREQ_HZ)
{
}

void GpioAxisProcessor::init() {
    AxisProcGuard state_lock(state_mutex_);
    if (!state_lock) return;
    setupFilters();
}

void GpioAxisProcessor::setupFilters() {
    float Fc = filter_freq_hz_ / SAMPLE_RATE_HZ;
    if (Fc <= 0.0f)  Fc = 0.001f;
    if (Fc >= 0.5f)  Fc = 0.49f;
    for (int i = 0; i < AXIS_PROC_CHANNELS; i++) {
        filters_[i].setBiquad(BiquadType::lowpass, Fc, FILTER_Q, 0.0f);
    }
}

// Recebe raw ADC (0..4095), retorna raw filtrado (0..4095). Se filter
// desabilitado, retorna raw inalterado. Clamp pra range do ADC.
uint16_t GpioAxisProcessor::filterRaw(uint8_t ch, uint16_t raw) {
    AxisProcGuard state_lock(state_mutex_);
    if (!state_lock) return raw;
    if (ch >= AXIS_PROC_CHANNELS) return raw;
    if (!filter_enabled_) return raw;

    float v = filters_[ch].process((float)raw);
    if (v < 0.0f)    v = 0.0f;
    if (v > 4095.0f) v = 4095.0f;
    return (uint16_t)v;
}

bool GpioAxisProcessor::isFilterEnabled() const {
    AxisProcGuard state_lock(state_mutex_);
    return state_lock ? filter_enabled_ : false;
}

void GpioAxisProcessor::setFilterEnabled(bool en) {
    AxisProcGuard state_lock(state_mutex_);
    if (!state_lock) return;
    filter_enabled_ = en;
    if (en) {
        // Re-setup limpa o estado interno (z1/z2) dos Biquads
        setupFilters();
    }
}

bool GpioAxisProcessor::isAutorangeEnabled() const {
    AxisProcGuard state_lock(state_mutex_);
    return state_lock ? autorange_enabled_ : false;
}

void GpioAxisProcessor::setAutorangeEnabled(bool en) {
    AxisProcGuard state_lock(state_mutex_);
    if (!state_lock) return;
    autorange_enabled_ = en;
}

float GpioAxisProcessor::getFilterFreq() const {
    AxisProcGuard state_lock(state_mutex_);
    return state_lock ? filter_freq_hz_ : DEFAULT_FREQ_HZ;
}

void GpioAxisProcessor::setFilterFreq(float hz) {
    AxisProcGuard state_lock(state_mutex_);
    if (!state_lock) return;
    if (hz < 0.5f)   hz = 0.5f;
    if (hz > 500.0f) hz = 500.0f;
    filter_freq_hz_ = hz;
    setupFilters();
}

// ===== C API =====
extern "C" {

void axis_proc_init(void) {
    GpioAxisProcessor::instance().init();
}
uint16_t axis_proc_filter_raw(uint8_t ch, uint16_t raw) {
    return GpioAxisProcessor::instance().filterRaw(ch, raw);
}
int axis_proc_get_filter_enabled(void) {
    return GpioAxisProcessor::instance().isFilterEnabled() ? 1 : 0;
}
void axis_proc_set_filter_enabled(int en) {
    GpioAxisProcessor::instance().setFilterEnabled(en != 0);
}
int axis_proc_get_autorange_enabled(void) {
    return GpioAxisProcessor::instance().isAutorangeEnabled() ? 1 : 0;
}
void axis_proc_set_autorange_enabled(int en) {
    GpioAxisProcessor::instance().setAutorangeEnabled(en != 0);
}
float axis_proc_get_filter_freq(void) {
    return GpioAxisProcessor::instance().getFilterFreq();
}
void axis_proc_set_filter_freq(float hz) {
    GpioAxisProcessor::instance().setFilterFreq(hz);
}

}  // extern "C"
