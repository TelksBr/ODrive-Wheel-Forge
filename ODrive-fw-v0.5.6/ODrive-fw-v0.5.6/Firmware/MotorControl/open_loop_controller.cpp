
#include "open_loop_controller.hpp"
#include <board.h>

void OpenLoopController::update(uint32_t timestamp) {
    float2D previous_idq{0.0f, 0.0f};
    const auto previous_idq_value = Idq_setpoint_.previous();
    if (previous_idq_value.has_value()) {
        previous_idq = *previous_idq_value;
    }
    auto [prev_Id, prev_Iq] = previous_idq;

    float2D previous_vdq{0.0f, 0.0f};
    const auto previous_vdq_value = Vdq_setpoint_.previous();
    if (previous_vdq_value.has_value()) {
        previous_vdq = *previous_vdq_value;
    }
    auto [prev_Vd, prev_Vq] = previous_vdq;
    float phase = phase_.previous().value_or(initial_phase_);
    float phase_vel = phase_vel_.previous().value_or(0.0f);

    (void)prev_Iq; // unused
    (void)prev_Vq; // unused

    float dt = (float)(timestamp - timestamp_) / (float)TIM_1_8_CLOCK_HZ;
    
    Idq_setpoint_ = {
        std::clamp(target_current_, prev_Id - max_current_ramp_ * dt, prev_Id + max_current_ramp_ * dt),
        0.0f
    };
    Vdq_setpoint_ = {
        std::clamp(target_voltage_, prev_Vd - max_voltage_ramp_ * dt, prev_Vd + max_voltage_ramp_ * dt),
        0.0f
    };
    
    phase_vel = std::clamp(target_vel_, phase_vel - max_phase_vel_ramp_ * dt, phase_vel + max_phase_vel_ramp_ * dt);
    phase_vel_ = phase_vel;
    phase_ = wrap_pm_pi(phase + phase_vel * dt);
    total_distance_ = total_distance_.previous().value_or(0.0f) + phase_vel * dt;
    timestamp_ = timestamp;
}
