#ifndef __FOC_HPP
#define __FOC_HPP

#include <atomic>

#include "phase_control_law.hpp"
#include "component.hpp"

/**
 * @brief Field oriented controller.
 * 
 * This controller can run in either current control mode or voltage control
 * mode.
 */
class FieldOrientedController : public AlphaBetaFrameController, public ComponentBase {
public:
    static_assert(std::atomic<float>::is_always_lock_free,
                  "FFB current telemetry snapshot must be lock-free in the control loop");

    void update(uint32_t timestamp) final;

    void reset() final;
    
    ODriveIntf::MotorIntf::Error on_measurement(
            std::optional<float> vbus_voltage,
            std::optional<float2D> Ialpha_beta,
            uint32_t input_timestamp) final;

    ODriveIntf::MotorIntf::Error get_alpha_beta_output(
            uint32_t output_timestamp,
            std::optional<float2D>* mod_alpha_beta,
            std::optional<float>* ibus) final;

    // Scalar accessors keep interface getters from passing float2D by value.
    float p_gain_for_interface() const {
        return pi_gains_.has_value() ? pi_gains_->first : 0.0f;
    }
    float i_gain_for_interface() const {
        return pi_gains_.has_value() ? pi_gains_->second : 0.0f;
    }
    float id_setpoint_for_interface() const {
        return Idq_setpoint_.has_value() ? Idq_setpoint_->first : 0.0f;
    }
    float iq_setpoint_for_interface() const {
        return Idq_setpoint_.has_value() ? Idq_setpoint_->second : 0.0f;
    }
    float vd_setpoint_for_interface() const {
        return Vdq_setpoint_.has_value() ? Vdq_setpoint_->first : 0.0f;
    }
    float vq_setpoint_for_interface() const {
        return Vdq_setpoint_.has_value() ? Vdq_setpoint_->second : 0.0f;
    }
    float ialpha_measured_for_interface() const {
        return Ialpha_beta_measured_.has_value() ? Ialpha_beta_measured_->first : 0.0f;
    }
    float ibeta_measured_for_interface() const {
        return Ialpha_beta_measured_.has_value() ? Ialpha_beta_measured_->second : 0.0f;
    }
    float get_ffb_iq_measured_snapshot() const {
        return ffb_iq_measured_snapshot_.load(std::memory_order_relaxed);
    }

    // Config - these values are set while this controller is inactive
    std::optional<float2D> pi_gains_; // [V/A, V/As] should be auto set after resistance and inductance measurement
    // Dead-band on Id/Iq error: when |Ierr| < this threshold, P term is forced
    // to 0 and the integrator freezes. Kills idle vibration from the PI chasing
    // ADC/encoder quantization noise when Iq_setpoint ≈ 0. Trade-off: introduces
    // a static torque uncertainty of approximately deadband * torque_constant.
    // 0 = disabled (stock behaviour). Set via Motor::config_.current_control_deadband.
    float current_control_deadband_ = 0.0f; // [A]
    float I_measured_report_filter_k_ = 1.0f;

    // Inputs
    bool enable_current_control_src_ = false;
    InputPort<float2D> Idq_setpoint_src_;
    InputPort<float2D> Vdq_setpoint_src_;
    InputPort<float> phase_src_;
    InputPort<float> phase_vel_src_;

    // These values are set atomically by the update() function and read by the
    // calculate() function in an interrupt context.
    uint32_t ctrl_timestamp_; // [HCLK ticks]
    bool enable_current_control_ = false; // true: FOC runs in current control mode using I{dq}_setpoint, false: FOC runs in voltage control mode using V{dq}_setpoint
    std::optional<float2D> Idq_setpoint_; // [A] only used if enable_current_control_ == true
    std::optional<float2D> Vdq_setpoint_; // [V] feed-forward voltage term (or standalone setpoint if enable_current_control_ == false)
    std::optional<float> phase_; // [rad]
    std::optional<float> phase_vel_; // [rad/s]

    // These values (or some of them) are updated inside on_measurement() and get_alpha_beta_output()
    uint32_t i_timestamp_;
    std::optional<float> vbus_voltage_measured_; // [V]
    std::optional<float2D> Ialpha_beta_measured_; // [A, A]
    float Id_measured_; // [A]
    float Iq_measured_; // [A]
    std::atomic<float> ffb_iq_measured_snapshot_{0.0f}; // [A], read by FFB/HID telemetry
    float v_current_control_integral_d_ = 0.0f; // [V]
    float v_current_control_integral_q_ = 0.0f; // [V]
    //float mod_to_V_ = 0.0f;
    //float mod_d_ = 0.0f;
    //float mod_q_ = 0.0f;
    //float ibus_ = 0.0f;
    float final_v_alpha_ = 0.0f; // [V]
    float final_v_beta_ = 0.0f; // [V]
    float power_ = 0.0f; // [W] dot product of Vdq and Idq
};

#endif // __FOC_HPP
