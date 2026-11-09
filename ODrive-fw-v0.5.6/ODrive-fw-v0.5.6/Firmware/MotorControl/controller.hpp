#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#include <atomic>

class Controller : public ODriveIntf::ControllerIntf {
public:
    static_assert(std::atomic<float>::is_always_lock_free,
                  "FFB torque input must be lock-free in the motor control loop");
    struct Anticogging_t {
        uint32_t index = 0;
        float cogging_map[3600];
        bool pre_calibrated = false;
        bool calib_anticogging = false;
        float calib_pos_threshold = 1.0f;
        float calib_vel_threshold = 1.0f;
        float cogging_ratio = 1.0f;
        bool anticogging_enabled = true;
    };

    struct Autotuning_t {
        float frequency = 0.0f;
        float pos_amplitude = 0.0f;
        float vel_amplitude = 0.0f;
        float torque_amplitude = 0.0f;
    };

    struct Config_t {
        // MKS Mini + FFB-target defaults — TORQUE_CONTROL mode needed for FFB torque injection.
        ControlMode control_mode = CONTROL_MODE_TORQUE_CONTROL;   // was POSITION_CONTROL stock; FFB requires torque mode
        InputMode input_mode = INPUT_MODE_PASSTHROUGH;            // input_torque_ → torque_setpoint_ 1:1
        float pos_gain = 100.0f;                 // [(turn/s) / turn] — era 20.0 stock; subido pra dar resposta de posição mais rígida (POSITION_CONTROL e anticogging calibration)
        float vel_gain = 0.566f;                 // [Nm/(turn/s)]    — era 1/6 (≈0.1667) stock
        // float vel_gain = 0.2f / 200.0f,       // [Nm/(rad/s)] <sensorless example>
        float vel_integrator_gain = 1.33f;       // [Nm/(turn/s * s)] — era 2/6 (≈0.333) stock
        float vel_limit = 5.0f;                  // [turn/s] (era 2.0f stock; subimos pra wheel sim racing — passar de 5 turn/s = 300 rpm é raro)
        float vel_limit_tolerance = 1.2f;        // ratio to vel_lim. Infinity to disable.
        float vel_integrator_limit = INFINITY;   // Vel. integrator clamping value. Infinity to disable.
        float vel_ramp_rate = 1.0f;              // [(turn/s) / s]
        float torque_ramp_rate = 0.01f;          // Nm / sec
        bool circular_setpoints = false;
        float circular_setpoint_range = 1.0f;    // Circular range when circular_setpoints is true. [turn]
        uint32_t steps_per_circular_range = 1024;
        float inertia = 0.0f;                    // [Nm/(turn/s^2)]
        float input_filter_bandwidth = 2.0f;     // [1/s]
        float homing_speed = 0.25f;              // [turn/s]
        Anticogging_t anticogging;
        float gain_scheduling_width = 10.0f;
        bool enable_gain_scheduling = false;
        bool enable_vel_limit = true;
        bool enable_overspeed_error = true;
        bool enable_torque_mode_vel_limit = false; // (era true stock) DESLIGADO pra wheel: o clamp `Tmax = (vel_limit - vel_estimate) * vel_gain` ficava cortando torque em modo TORQUE mesmo com motor parado (default ~0.83 Nm). Pra wheel sim racing, queremos torque livre e usamos vel_limit só como sensor de OVERSPEED.
        uint8_t axis_to_mirror = -1;
        float mirror_ratio = 1.0f;
        float torque_mirror_ratio = 0.0f;
        uint8_t load_encoder_axis = -1;  // default depends on Axis number and is set in load_configuration(). Set to -1 to select sensorless estimator.
        // FFB wheel tuning: spinout detection mais tolerante a picos legítimos
        // de potência (counter-torque com MAIRA, kicks fortes do FFB, etc).
        // Bandwidth mantido no stock (20 rad/s) — só os thresholds foram subidos.
        float mechanical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for mechanical power (stock)
        float electrical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for electrical power (stock)
        float spinout_electrical_power_threshold =  50.0f; // [W] (era 10.0 stock) — só dispara em sustained high power
        float spinout_mechanical_power_threshold = -50.0f; // [W] (era -10.0 stock) — só dispara em sustained high regen

        // custom setters
        Controller* parent;
        void set_input_filter_bandwidth(float value) { input_filter_bandwidth = value; parent->update_filter_gains(); }
        void set_steps_per_circular_range(uint32_t value) { steps_per_circular_range = value > 0 ? value : steps_per_circular_range; }
        void set_control_mode(ControlMode value) { control_mode = value; parent->control_mode_updated(); }
    };

    
    bool apply_config();

    void reset();
    void set_error(Error error);

    constexpr void input_pos_updated() {
        input_pos_updated_ = true;
    }
    void set_input_torque(float value) {
        input_torque_.store(value, std::memory_order_relaxed);
    }
    float get_input_torque() const {
        return input_torque_.load(std::memory_order_relaxed);
    }
    void set_ffb_torque_output_snapshot(float value) {
        ffb_torque_output_snapshot_.store(value, std::memory_order_relaxed);
    }
    float get_ffb_torque_output_snapshot() const {
        return ffb_torque_output_snapshot_.load(std::memory_order_relaxed);
    }
    bool control_mode_updated();
    void set_input_pos_and_steps(float pos);

    bool select_encoder(size_t encoder_num);

    // Trajectory-Planned control
    void move_to_pos(float goal_point);
    void move_incremental(float displacement, bool from_goal_point);
    
    // TODO: make this more similar to other calibration loops
    void start_anticogging_calibration();
    float remove_anticogging_bias();
    bool anticogging_calibration(float pos_estimate, float vel_estimate);
    
    float get_anticogging_value(uint32_t index) {
        return (index < 3600) ? config_.anticogging.cogging_map[index] : 0.0f;
    }

    void update_filter_gains();
    bool update();

    Config_t config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    Error error_ = ERROR_NONE;
    float last_error_time_ = 0.0f;

    // Inputs
    InputPort<float> pos_estimate_linear_src_;
    InputPort<float> pos_estimate_circular_src_;
    InputPort<float> vel_estimate_src_;
    InputPort<float> pos_wrap_src_; 

    float pos_setpoint_ = 0.0f; // [turns]
    float vel_setpoint_ = 0.0f; // [turn/s]
    // float vel_setpoint = 800.0f; <sensorless example>
    float vel_integrator_torque_ = 0.0f;    // [Nm]
    float torque_setpoint_ = 0.0f;  // [Nm]

    float input_pos_ = 0.0f;     // [turns]
    float input_vel_ = 0.0f;     // [turn/s]
    float input_filter_kp_ = 0.0f;
    float input_filter_ki_ = 0.0f;

    Autotuning_t autotuning_;
    float autotuning_phase_ = 0.0f;
    
    bool input_pos_updated_ = false;
    
    bool trajectory_done_ = true;

    bool anticogging_valid_ = false;
    float mechanical_power_ = 0.0f; // [W]
    float electrical_power_ = 0.0f; // [W]

    // Outputs
    OutputPort<float> torque_output_ = 0.0f;

    // custom setters
    void set_input_pos(float value) { set_input_pos_and_steps(value); input_pos_updated(); }

private:
    std::atomic<float> input_torque_{0.0f}; // [Nm], published across tasks/interrupts
    std::atomic<float> ffb_torque_output_snapshot_{0.0f}; // [Nm], read by HID telemetry
};

#endif // __CONTROLLER_HPP
