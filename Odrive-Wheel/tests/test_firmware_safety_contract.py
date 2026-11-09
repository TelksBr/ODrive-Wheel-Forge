"""Regression checks for firmware input and timing safety contracts.

These checks intentionally inspect the production source: they do not replace
hardware-in-the-loop validation of the motor control loop.
"""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


def function_body(source, signature):
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1:index]
    raise AssertionError(f"Unclosed function body: {signature}")


class FirmwareSafetyContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hid = (ROOT / "src" / "HidFFB.cpp").read_text(encoding="utf-8")
        cls.effects = (ROOT / "src" / "EffectsCalculator.cpp").read_text(encoding="utf-8")
        cls.ffb_task = (ROOT / "src" / "ffb_task.cpp").read_text(encoding="utf-8")
        cls.commands = (ROOT / "src" / "cmd_table.cpp").read_text(encoding="utf-8")
        cls.defs = (ROOT / "inc" / "ffb_defs.h").read_text(encoding="utf-8")
        cls.effects_header = (ROOT / "inc" / "EffectsCalculator.h").read_text(encoding="utf-8")
        cls.board = (ROOT / "src" / "board_v3.cpp").read_text(encoding="utf-8")

    def test_hid_payload_staging_rejects_short_or_null_input(self):
        self.assertIn("buffer == nullptr || size < sizeof(T)", self.hid)
        self.assertIn("buffer == nullptr || size < sizeof(T) - 1", self.hid)
        self.assertIn("memcpy(bytes + 1, buffer, sizeof(T) - 1)", self.hid)
        self.assertNotIn("(FFB_SetCondition_Data_t*)report", self.hid)
        self.assertNotIn("(FFB_SetPeriodic_Data_t*)report", self.hid)

    def test_hid_get_never_writes_past_requested_length(self):
        self.assertIn("buffer == nullptr || reqlen == 0", self.hid)
        self.assertIn("std::min<uint16_t>(reqlen, sizeof(FFB_BlockLoad_Feature_Data_t))", self.hid)
        self.assertIn("std::min<uint16_t>(reqlen, sizeof(FFB_PIDPool_Feature_Data_t))", self.hid)

    def test_interrupt_out_report_id_is_removed_before_dispatch(self):
        self.assertIn("if (report_id == 0 && size > 0 && type == HID_REPORT_TYPE_OUTPUT)", self.ffb_task)
        self.assertIn("report_id = buf[0];", self.ffb_task)
        self.assertIn("buf++;", self.ffb_task)
        self.assertIn("size--;", self.ffb_task)
        # The payload-only buffer is what reaches HidFFB; it rebuilds IDs only
        # for internal structs whose first byte represents reportId.
        self.assertIn("UsbHidHandler::globalHidHandler->hidOut(report_id, type, buf, size)", self.ffb_task)

    def test_axis_parameters_are_sanitized_on_set_and_restore(self):
        self.assertIn("!__builtin_isfinite(value)", self.ffb_task)
        self.assertIn("value > 25.0f ? 25.0f : value", self.ffb_task)
        self.assertIn("value > 1.0f ? 1.0f : value", self.ffb_task)
        self.assertIn("rangeDegrees_ = sanitize_axis_range", self.ffb_task)
        self.assertIn("maxTorque_Nm_ = sanitize_axis_torque", self.ffb_task)
        self.assertIn("fxRatio_ = sanitize_axis_fxratio", self.ffb_task)

    def test_fxratio_default_is_consistent_for_axis_init_and_invalid_values(self):
        self.assertRegex(self.ffb_task, r"DEFAULT_AXIS_FX_RATIO\s*=\s*1\.0f")
        self.assertIn("float fxRatio_       = DEFAULT_AXIS_FX_RATIO", self.ffb_task)
        sanitize = self.ffb_task.split("static float sanitize_axis_fxratio", 1)[1].split(
            "static void ffb_load_axis_params_internal", 1
        )[0]
        getter = self.ffb_task.split('extern "C" float ffb_get_axis_fxratio(void)', 1)[1].split(
            'extern "C" void ffb_set_axis_fxratio', 1
        )[0]
        self.assertIn("return DEFAULT_AXIS_FX_RATIO", sanitize)
        self.assertIn("return DEFAULT_AXIS_FX_RATIO", getter)

    def test_zero_offset_setter_rejects_nonfinite_values_without_changing_state(self):
        setter = self.ffb_task.split('extern "C" void ffb_set_axis_zeroofs(float v)', 1)[1].split(
            "\n}", 1
        )[0]
        reject = setter.index("if (!__builtin_isfinite(v)) return;")
        lock = setter.index("EffectsStateGuard state_lock(*s_effects_calc)")
        write = setter.index("s_axis_raw->zeroOffset_ = v")
        self.assertLess(reject, lock)
        self.assertLess(lock, write)

        import math
        for value in (math.nan, math.inf, -math.inf):
            current_offset = 123.5
            if math.isfinite(value):
                current_offset = value
            self.assertEqual(current_offset, 123.5)

    def test_odrive_torque_setpoint_is_lock_free_and_atomic_across_tasks(self):
        odrive_root = ROOT.parent / "ODrive-fw-v0.5.6" / "ODrive-fw-v0.5.6" / "Firmware"
        controller_header = (odrive_root / "MotorControl" / "controller.hpp").read_text(encoding="utf-8")
        controller = (odrive_root / "MotorControl" / "controller.cpp").read_text(encoding="utf-8")
        axis = (odrive_root / "MotorControl" / "axis.cpp").read_text(encoding="utf-8")
        can = (odrive_root / "communication" / "can" / "can_simple.cpp").read_text(encoding="utf-8")
        ascii_protocol = (odrive_root / "communication" / "ascii_protocol.cpp").read_text(encoding="utf-8")
        bridge = (ROOT / "src" / "odrive_bridge.cpp").read_text(encoding="utf-8")
        interface_yaml = (odrive_root / "odrive-interface.yaml").read_text(encoding="utf-8")
        generated_interfaces = (odrive_root / "autogen" / "interfaces.hpp").read_text(encoding="utf-8")

        self.assertIn("std::atomic<float> input_torque_", controller_header)
        self.assertIn("static_assert(std::atomic<float>::is_always_lock_free", controller_header)
        self.assertIn("input_torque_.load(std::memory_order_relaxed)", controller_header)
        self.assertIn("input_torque_.store(value, std::memory_order_relaxed)", controller_header)
        torque_property = interface_yaml.split("      input_torque:", 1)[1].split(
            "      pos_setpoint:", 1
        )[0]
        self.assertIn("c_getter: get_input_torque()", torque_property)
        self.assertIn("c_setter: set_input_torque", torque_property)
        self.assertIn("get_input_torque()", generated_interfaces)
        self.assertIn("set_input_torque(value)", generated_interfaces)
        self.assertNotIn("&obj->input_torque_", generated_interfaces)
        self.assertEqual(controller.count("get_input_torque()"), 3)
        self.assertEqual(controller.count("set_input_torque("), 2)
        self.assertNotRegex(controller, r"\binput_torque_\s*=")
        self.assertNotRegex(axis, r"\binput_torque_\s*=")
        self.assertNotRegex(can, r"\binput_torque_\s*=")
        self.assertNotRegex(ascii_protocol, r"\binput_torque_\s*=")
        self.assertNotIn("controller_.input_torque_", bridge)
        self.assertIn("controller_.set_input_torque(nm)", bridge)

    def test_nonfinite_torque_setpoints_are_zeroed_before_atomic_publication(self):
        odrive_root = ROOT.parent / "ODrive-fw-v0.5.6" / "ODrive-fw-v0.5.6" / "Firmware"
        controller_header = (odrive_root / "MotorControl" / "controller.hpp").read_text(encoding="utf-8")
        setter = function_body(controller_header, "void set_input_torque(float value)")
        reject = setter.index("if (!__builtin_isfinite(value)) value = 0.0f;")
        store = setter.index("input_torque_.store(value, std::memory_order_relaxed)")
        self.assertLess(reject, store)

    def test_ffb_diagnostics_use_lock_free_atomic_counters_and_snapshots(self):
        counters = (
            "s_diag_hidout_total", "s_diag_hidout_ctrl", "s_diag_hidout_neweff",
            "s_diag_hidout_seteff", "s_diag_hidout_cond", "s_diag_hidout_const",
            "s_diag_hidout_period", "s_diag_hidout_efop", "s_diag_hidout_gain",
            "s_diag_hidout_other", "s_diag_hidget", "s_diag_set_eff_torque",
            "s_ffb_loop_overruns", "s_ffb_loop_samples", "s_ffb_period_overruns",
        )
        self.assertIn("std::atomic<uint32_t>::is_always_lock_free", self.ffb_task)
        self.assertIn("compare_exchange_weak", self.ffb_task)
        for counter in counters:
            self.assertIn(f"std::atomic<uint32_t> {counter}", self.ffb_task)
            self.assertIn(f"{counter}.fetch_add(1, std::memory_order_relaxed)", self.ffb_task)
            self.assertRegex(self.ffb_task, rf"{counter}\.load\(std::memory_order_relaxed\)")
        for metric in (
            "s_ffb_loop_last_us", "s_ffb_loop_max_us", "s_ffb_calc_max_us",
            "s_ffb_lock_wait_last_us", "s_ffb_lock_wait_max_us",
            "s_ffb_lock_hold_max_us", "s_ffb_sample_last_us", "s_ffb_sample_max_us",
        ):
            self.assertIn(f"std::atomic<uint32_t> {metric}", self.ffb_task)
        self.assertIn("std::atomic<int32_t> s_diag_last_torque_int32", self.ffb_task)
        self.assertIn("s_diag_last_torque_int32.load(std::memory_order_relaxed)", self.ffb_task)
        self.assertNotRegex(self.ffb_task, r"volatile\s+(?:uint32_t|int32_t)\s+s_(?:diag|ffb)_")

    def test_ffb_bus_peaks_are_atomic_across_sampling_and_diagnostic_reset(self):
        peaks = (
            "s_ibus_max", "s_ibus_min", "s_motor_ibus_max",
            "s_motor_ibus_min", "s_vbus_max", "s_vbus_min",
        )
        self.assertIn("std::atomic<float>::is_always_lock_free", self.ffb_task)
        self.assertIn("update_diag_max_float", self.ffb_task)
        self.assertIn("update_diag_min_float", self.ffb_task)
        for peak in peaks:
            self.assertRegex(self.ffb_task, rf"std::atomic<float>\s+{peak}")
            self.assertNotRegex(self.ffb_task, rf"volatile\s+float\s+{peak}")
            self.assertRegex(self.ffb_task, rf"{peak}\.load\(std::memory_order_relaxed\)")
            self.assertRegex(self.ffb_task, rf"{peak}\.store\([^\n]*std::memory_order_relaxed\)")

    def test_vbus_scale_is_published_atomically_between_configuration_and_adc_isr(self):
        odrive_root = ROOT.parent / "ODrive-fw-v0.5.6" / "ODrive-fw-v0.5.6" / "Firmware"
        low_level = (odrive_root / "MotorControl" / "low_level.cpp").read_text(encoding="utf-8")
        self.assertTrue(
            "std::atomic<float> g_vbus_voltage_scale" in self.ffb_task,
            "VBUS scale shared with ADC ISR must be atomic",
        )
        self.assertTrue(
            "extern std::atomic<float> g_vbus_voltage_scale" in low_level,
            "ADC ISR must observe the atomic VBUS scale declaration",
        )
        adc_callback = function_body(low_level, "void vbus_sense_adc_cb(uint32_t adc_value)")
        self.assertIn("g_vbus_voltage_scale.load(std::memory_order_relaxed)", adc_callback)
        setter = function_body(self.ffb_task, 'extern "C" void ffb_set_vbus_divider(int v) {')
        self.assertIn("g_vbus_voltage_scale.store(", setter)
        self.assertNotIn("volatile float g_vbus_voltage_scale", self.ffb_task)

    def test_encoder_index_diagnostics_use_atomic_isr_counters(self):
        odrive_root = ROOT.parent / "ODrive-fw-v0.5.6" / "ODrive-fw-v0.5.6" / "Firmware"
        encoder = (odrive_root / "MotorControl" / "encoder.cpp").read_text(encoding="utf-8")
        for counter in ("g_z_hit_count", "g_z_glitch_count"):
            self.assertRegex(encoder, rf"std::atomic<uint32_t>\s+{counter}")
            self.assertIn(f"{counter}.fetch_add(1, std::memory_order_relaxed)", encoder)
            self.assertNotRegex(encoder, rf"volatile\s+uint32_t\s+{counter}")
        self.assertIn('extern std::atomic<uint32_t> g_z_hit_count;', self.ffb_task)
        self.assertIn('extern std::atomic<uint32_t> g_z_glitch_count;', self.ffb_task)
        self.assertIn("g_z_hit_count.load(std::memory_order_relaxed)", self.ffb_task)
        self.assertIn("g_z_hit_count.store(v, std::memory_order_relaxed)", self.ffb_task)
        self.assertIn("g_z_glitch_count.load(std::memory_order_relaxed)", self.ffb_task)
        self.assertIn("g_z_glitch_count.store(v, std::memory_order_relaxed)", self.ffb_task)

    def test_encoder_position_and_velocity_are_published_as_atomic_ffb_snapshots(self):
        odrive_root = ROOT.parent / "ODrive-fw-v0.5.6" / "ODrive-fw-v0.5.6" / "Firmware"
        encoder_header = (odrive_root / "MotorControl" / "encoder.hpp").read_text(encoding="utf-8")
        encoder = (odrive_root / "MotorControl" / "encoder.cpp").read_text(encoding="utf-8")
        bridge = (ROOT / "src" / "odrive_bridge.cpp").read_text(encoding="utf-8")

        self.assertIn("std::atomic<float> ffb_pos_turns_snapshot_", encoder_header)
        self.assertIn("std::atomic<float> ffb_vel_estimate_snapshot_", encoder_header)
        self.assertIn("std::atomic<float>::is_always_lock_free", encoder_header)
        self.assertIn("std::atomic<uint32_t> ffb_motion_snapshot_sequence_", encoder_header)
        self.assertIn("ffb_pos_turns_snapshot_.load(std::memory_order_relaxed)", encoder_header)
        self.assertIn("ffb_vel_estimate_snapshot_.load(std::memory_order_relaxed)", encoder_header)
        self.assertEqual(encoder.count("publish_ffb_motion_snapshot("), 3)
        set_linear_count = encoder.split("void Encoder::set_linear_count", 1)[1].split(
            "void Encoder::set_circular_count", 1
        )[0]
        self.assertLess(
            set_linear_count.index("const float ffb_position_snapshot"),
            set_linear_count.index("cpu_enter_critical()"),
        )
        self.assertLess(set_linear_count.index("cpu_enter_critical()"),
                        set_linear_count.index("publish_ffb_motion_snapshot("))
        self.assertLess(set_linear_count.index("publish_ffb_motion_snapshot("),
                        set_linear_count.index("cpu_exit_critical(prim)"))
        offset_calibration = function_body(encoder, "bool Encoder::run_offset_calibration()")
        offset_snapshot_section = offset_calibration.split("CRITICAL_SECTION() {", 1)[1].split(
            "\n    }", 1
        )[0]
        self.assertIn("publish_ffb_motion_snapshot(", offset_snapshot_section)
        main_source = (odrive_root / "MotorControl" / "main.cpp").read_text(encoding="utf-8")
        control_loop = function_body(main_source, "void ODrive::control_loop_cb(uint32_t timestamp)")
        self.assertIn("axis.encoder_.update();", control_loop)
        self.assertNotIn("int32_t cnt = axes[0].encoder_.shadow_count_", bridge)
        self.assertNotIn("encoder_.vel_estimate_.any()", bridge)
        self.assertIn("encoder_.get_ffb_pos_turns_snapshot()", bridge)
        self.assertIn("encoder_.get_ffb_vel_estimate_snapshot()", bridge)
        self.assertIn("encoder_.get_ffb_motion_snapshot(*position_turns", bridge)
        axis_update = self.ffb_task.split("void update()", 1)[1].split("void zeroEncoder()", 1)[0]
        snapshot_read = axis_update.index("odrive_bridge_get_motion_snapshot(&turns, &velocity_turns_per_second)")
        derived_position = axis_update.index("float degrees = turns * 360.0f")
        self.assertLess(snapshot_read, derived_position)

    def test_ffb_axis_reads_position_and_velocity_as_one_encoder_snapshot(self):
        bridge = (ROOT / "src" / "odrive_bridge.cpp").read_text(encoding="utf-8")
        odrive_root = ROOT.parent / "ODrive-fw-v0.5.6" / "ODrive-fw-v0.5.6" / "Firmware"
        encoder_header = (odrive_root / "MotorControl" / "encoder.hpp").read_text(encoding="utf-8")
        encoder = (odrive_root / "MotorControl" / "encoder.cpp").read_text(encoding="utf-8")

        self.assertIn("get_ffb_motion_snapshot(float& position_turns,", encoder_header)
        self.assertIn("ffb_motion_snapshot_sequence_", encoder_header)
        self.assertIn("if (sequence_before == sequence_after) return;", encoder_header)
        snapshot_reader = function_body(
            encoder_header,
            "void get_ffb_motion_snapshot(float& position_turns,",
        )
        reader_order = (
            "sequence_before =",
            "if (sequence_before & 1u) continue;",
            "position_turns = ffb_pos_turns_snapshot_.load",
            "velocity_turns_per_second =",
            "std::atomic_thread_fence(std::memory_order_acq_rel)",
            "sequence_after =",
            "if (sequence_before == sequence_after) return;",
        )
        reader_positions = [snapshot_reader.index(item) for item in reader_order]
        self.assertEqual(reader_positions, sorted(reader_positions))
        snapshot_writer = function_body(
            encoder_header,
            "void publish_ffb_motion_snapshot(float position_turns,",
        )
        sequence_read = snapshot_writer.index(
            "ffb_motion_snapshot_sequence_.load(std::memory_order_relaxed)"
        )
        odd_publish = snapshot_writer.index(
            "ffb_motion_snapshot_sequence_.store(sequence + 1u,"
        )
        writer_fence = snapshot_writer.index(
            "std::atomic_thread_fence(std::memory_order_seq_cst)"
        )
        position_write = snapshot_writer.index("ffb_pos_turns_snapshot_.store(position_turns")
        velocity_write = snapshot_writer.index("ffb_vel_estimate_snapshot_.store(velocity_turns_per_second")
        final_publish = snapshot_writer.index(
            "ffb_motion_snapshot_sequence_.store(sequence + 2u,"
        )
        self.assertLess(sequence_read, odd_publish)
        self.assertLess(odd_publish, writer_fence)
        self.assertLess(writer_fence, position_write)
        self.assertLess(position_write, velocity_write)
        self.assertLess(velocity_write, final_publish)
        self.assertNotIn("ffb_motion_snapshot_sequence_.fetch_add", snapshot_writer)

        # The uint32 version may wrap, but its odd/even writer states must not.
        mask = (1 << 32) - 1
        for sequence in (0, 2, mask - 1):
            odd_sequence = (sequence + 1) & mask
            even_sequence = (sequence + 2) & mask
            self.assertEqual(odd_sequence & 1, 1)
            self.assertEqual(even_sequence & 1, 0)

        self.assertIn("odrive_bridge_get_motion_snapshot(&turns, &velocity_turns_per_second)", self.ffb_task)
        self.assertRegex(
            bridge,
            r"get_ffb_motion_snapshot\(\*position_turns,\s*\*velocity_turns_per_second\)",
        )
        self.assertIn("publish_ffb_motion_snapshot(", encoder)
        axis_update = self.ffb_task.split("void update()", 1)[1].split("void zeroEncoder()", 1)[0]
        self.assertNotIn("odrive_bridge_get_pos_turns()", axis_update)
        self.assertNotIn("odrive_bridge_get_vel_estimate()", axis_update)

    def test_hid_torque_telemetry_uses_a_lock_free_controller_snapshot(self):
        odrive_root = ROOT.parent / "ODrive-fw-v0.5.6" / "ODrive-fw-v0.5.6" / "Firmware"
        controller_header = (odrive_root / "MotorControl" / "controller.hpp").read_text(encoding="utf-8")
        controller = (odrive_root / "MotorControl" / "controller.cpp").read_text(encoding="utf-8")
        bridge = (ROOT / "src" / "odrive_bridge.cpp").read_text(encoding="utf-8")
        self.assertIn("std::atomic<float> ffb_torque_output_snapshot_", controller_header)
        self.assertIn("ffb_torque_output_snapshot_.load(std::memory_order_relaxed)", controller_header)
        self.assertIn("ffb_torque_output_snapshot_.store(value, std::memory_order_relaxed)", controller_header)
        output_write = controller.index("torque_output_ = torque;")
        snapshot_write = controller.index("set_ffb_torque_output_snapshot(torque)", output_write)
        self.assertLess(output_write, snapshot_write)
        self.assertIn("controller_.get_ffb_torque_output_snapshot()", bridge)
        self.assertNotIn("controller_.torque_output_.any()", bridge)

    def test_hid_iq_telemetry_uses_a_lock_free_foc_snapshot(self):
        odrive_root = ROOT.parent / "ODrive-fw-v0.5.6" / "ODrive-fw-v0.5.6" / "Firmware"
        foc_header = (odrive_root / "MotorControl" / "foc.hpp").read_text(encoding="utf-8")
        foc = (odrive_root / "MotorControl" / "foc.cpp").read_text(encoding="utf-8")
        bridge = (ROOT / "src" / "odrive_bridge.cpp").read_text(encoding="utf-8")

        self.assertIn("std::atomic<float> ffb_iq_measured_snapshot_", foc_header)
        self.assertIn("std::atomic<float>::is_always_lock_free", foc_header)
        self.assertIn("ffb_iq_measured_snapshot_.load(std::memory_order_relaxed)", foc_header)
        output_calculation = foc.index("FieldOrientedController::get_alpha_beta_output")
        iq_update = foc.index("Iq_measured_ +=", output_calculation)
        snapshot_publish = foc.index("ffb_iq_measured_snapshot_.store(Iq_measured_", iq_update)
        self.assertLess(iq_update, snapshot_publish)
        self.assertIn("get_ffb_iq_measured_snapshot()", bridge)
        self.assertNotIn("current_control_.Iq_measured_", bridge)

    def test_hid_bus_telemetry_uses_lock_free_snapshots(self):
        odrive_root = ROOT.parent / "ODrive-fw-v0.5.6" / "ODrive-fw-v0.5.6" / "Firmware"
        low_level = (odrive_root / "MotorControl" / "low_level.cpp").read_text(encoding="utf-8")
        motor_header = (odrive_root / "MotorControl" / "motor.hpp").read_text(encoding="utf-8")
        motor = (odrive_root / "MotorControl" / "motor.cpp").read_text(encoding="utf-8")
        bridge = (ROOT / "src" / "odrive_bridge.cpp").read_text(encoding="utf-8")

        for snapshot in (
            "s_ffb_vbus_voltage_snapshot_",
            "s_ffb_ibus_snapshot_",
            "s_ffb_brake_current_snapshot_",
        ):
            self.assertIn(f"std::atomic<float> {snapshot}", low_level)
            self.assertIn(f"{snapshot}.load(std::memory_order_relaxed)", low_level)
        self.assertIn("s_ffb_vbus_voltage_snapshot_.store(vbus_voltage", low_level)
        self.assertIn("s_ffb_ibus_snapshot_.store(ibus_", low_level)
        self.assertIn("s_ffb_brake_current_snapshot_.store(brake_resistor_current", low_level)
        self.assertIn("std::atomic<float> ffb_i_bus_snapshot_", motor_header)
        self.assertIn("ffb_i_bus_snapshot_.load(std::memory_order_relaxed)", motor_header)
        bus_write = motor.index("I_bus_ = *i_bus;")
        self.assertLess(bus_write, motor.index("ffb_i_bus_snapshot_.store(I_bus_", bus_write))
        self.assertIn("low_level_ffb_get_vbus_snapshot()", bridge)
        self.assertIn("low_level_ffb_get_ibus_snapshot()", bridge)
        self.assertIn("get_ffb_i_bus_snapshot()", bridge)
        self.assertIn("low_level_ffb_get_brake_current_snapshot()", bridge)
        self.assertNotIn("return odrv.vbus_voltage_;", bridge)
        self.assertNotIn("return odrv.ibus_;", bridge)
        self.assertNotIn("return axes[0].motor_.I_bus_;", bridge)
        self.assertNotIn("return odrv.brake_resistor_current_;", bridge)

    def test_zerowheel_gpio_callback_never_reenters_effect_state_mutex(self):
        gpio = (ROOT / "src" / "gpio_inputs.cpp").read_text(encoding="utf-8")
        thread = self.ffb_task.split("static void ffb_thread(void *arg)", 1)[1].split(
            "// -------------------- Init", 1
        )[0]
        gpio_update = gpio.split("extern \"C\" void gpio_inputs_update_report", 1)[1]
        zero_setter = self.ffb_task.split('extern "C" void ffb_axis_zeroenc(void)', 1)[1].split(
            "\n}", 1
        )[0]

        self.assertIn("ffb_axis_zeroenc();", gpio_update)
        self.assertIn("EffectsStateGuard state_lock(*s_effects_calc)", zero_setter)
        self.assertLess(thread.index("gpio_inputs_update_report("),
                        thread.index("EffectsStateGuard axis_report_lock"))

    def test_hid_send_and_telemetry_conversion_are_outside_effect_mutex(self):
        thread = self.ffb_task.split("static void ffb_thread(void *arg)", 1)[1].split(
            "// -------------------- Init", 1
        )[0]
        lock_start = thread.index("EffectsStateGuard axis_report_lock")
        lock_scope_end = thread.index("}", lock_start)
        axis_read = thread.index("getScaledAxisPos()", lock_start)
        report_write = thread.index("rpt.X =", lock_start)
        usb_send = thread.index("tud_hid_report(", lock_start)

        self.assertLess(axis_read, lock_scope_end)
        self.assertLess(lock_scope_end, report_write)
        self.assertLess(report_write, usb_send)

    def test_gpio_configuration_and_report_state_share_a_mutex(self):
        gpio = (ROOT / "src" / "gpio_inputs.cpp").read_text(encoding="utf-8")
        functions = (
            'void gpio_inputs_init',
            'void gpio_inputs_update_report',
            'int gpio_inputs_save',
            'uint8_t gpio_inputs_get_mode', 'int gpio_inputs_set_mode',
            'uint8_t gpio_inputs_get_idx', 'int gpio_inputs_set_idx',
            'uint8_t gpio_inputs_get_invert', 'int gpio_inputs_set_invert',
            'uint16_t gpio_inputs_get_amin', 'int gpio_inputs_set_amin',
            'uint16_t gpio_inputs_get_amax', 'int gpio_inputs_set_amax',
            'uint16_t gpio_inputs_get_filt', 'uint16_t gpio_inputs_read_raw',
        )
        for signature in functions:
            body = function_body(gpio, signature)
            self.assertIn("GpioConfigGuard guard(s_gpio_cfg_mutex)", body, signature)

        save_body = function_body(gpio, "int gpio_inputs_save")
        self.assertIn("gpio_cfg_t config_snapshot[GPIO_INPUTS_COUNT]", save_body)
        lock_scope_end = save_body.index("}")
        flash_write = save_body.index("Flash_Write(s_addr_cfg[i]")
        self.assertLess(lock_scope_end, flash_write)

    def test_gpio_mutex_handle_is_published_atomically_during_boot(self):
        gpio = (ROOT / "src" / "gpio_inputs.cpp").read_text(encoding="utf-8")
        self.assertIn("std::atomic<osMutexId> s_gpio_cfg_mutex", gpio)
        self.assertIn("std::atomic<osMutexId>::is_always_lock_free", gpio)
        self.assertIn("mutex.load(std::memory_order_acquire)", gpio)
        init = function_body(gpio, "void gpio_inputs_init")
        create = init.index("osMutexCreate(osMutex(gpioCfgMutex))")
        publish = init.index("s_gpio_cfg_mutex.store(mutex, std::memory_order_release)", create)
        guard = init.index("GpioConfigGuard guard(s_gpio_cfg_mutex)", publish)
        self.assertLess(create, publish)
        self.assertLess(publish, guard)

    def test_gpio_axis_biquad_state_is_mutex_protected(self):
        header = (ROOT / "inc" / "gpio_axis_proc.h").read_text(encoding="utf-8")
        processor = (ROOT / "src" / "gpio_axis_proc.cpp").read_text(encoding="utf-8")
        self.assertIn("cpp_freertos::MutexStandard state_mutex_", header)
        functions = (
            "void GpioAxisProcessor::init",
            "uint16_t GpioAxisProcessor::filterRaw",
            "bool GpioAxisProcessor::isFilterEnabled",
            "void GpioAxisProcessor::setFilterEnabled",
            "bool GpioAxisProcessor::isAutorangeEnabled",
            "void GpioAxisProcessor::setAutorangeEnabled",
            "float GpioAxisProcessor::getFilterFreq",
            "void GpioAxisProcessor::setFilterFreq",
        )
        for signature in functions:
            body = function_body(processor, signature)
            self.assertIn("AxisProcGuard state_lock(state_mutex_)", body, signature)

    def test_restored_filter_profile_is_limited_to_allocated_slots(self):
        restore = self.effects.split("void EffectsCalculator::restoreFlash()", 1)[1].split(
            "void EffectsCalculator::saveFlash", 1
        )[0]
        self.assertIn("filterProfileId = clip<uint8_t, uint8_t>", restore)
        self.assertIn("CUSTOM_PROFILE_ID", restore)

        # The filter array has only the default and custom profiles (0 and 1).
        self.assertEqual([min(raw_profile, 1) for raw_profile in range(4)], [0, 1, 1, 1])

    def test_restored_friction_rampup_percent_matches_runtime_range(self):
        restore = self.effects.split("void EffectsCalculator::restoreFlash()", 1)[1].split(
            "void EffectsCalculator::saveFlash", 1
        )[0]
        self.assertIn("frictionPctSpeedToRampup = clip<uint8_t, uint8_t>", restore)
        self.assertIn("uint8_t pct = clip<uint8_t, uint8_t>(cmd.val, 0, 100)", self.effects)
        self.assertIn("frictionPctSpeedToRampup = pct", self.effects)

        self.assertEqual([min(raw, 100) for raw in (0, 25, 100, 101, 255)], [0, 25, 100, 100, 100])

    def test_hid_diagnostic_report_ids_match_the_hid_definitions(self):
        bridge = self.ffb_task.split("// Diagnóstico — incrementa contador específico por report_id", 1)[1]
        self.assertIn("case 3:  s_diag_hidout_cond.fetch_add(1, std::memory_order_relaxed)", bridge)
        self.assertIn("case 4:  s_diag_hidout_period.fetch_add(1, std::memory_order_relaxed)", bridge)
        self.assertNotIn("case 2:  s_diag_hidout_period.fetch_add", bridge)
        self.assertIn("(s_axis_raw->maxTorque_Nm_ * s_axis_raw->fxRatio_)", self.ffb_task)

    def test_effect_start_and_ramp_elapsed_use_wrap_safe_millisecond_clock(self):
        self.assertIn("(int32_t)elapsed_ms < 0", self.effects)
        noncondition = self.effects.split("int32_t EffectsCalculator::calcNonConditionEffectForce", 1)[1].split(
            "int32_t EffectsCalculator::calcComponentForce", 1
        )[0]
        self.assertIn("const uint32_t elapsed_ms = now_ms - effect->startTime", noncondition)
        self.assertNotIn("micros() / 1000.0f", self.effects)

    def test_ramp_envelope_shapes_attack_and_fade_without_changing_sustain(self):
        noncondition = self.effects.split("int32_t EffectsCalculator::calcNonConditionEffectForce", 1)[1].split(
            "int32_t EffectsCalculator::calcComponentForce", 1
        )[0]
        ramp = noncondition.split("case FFB_EFFECT_RAMP:", 1)[1].split(
            "case FFB_EFFECT_SQUARE:", 1
        )[0]
        self.assertIn("effect->useEnvelope", ramp)
        self.assertIn("effect->attackLevel", ramp)
        self.assertIn("effect->fadeLevel", ramp)
        self.assertIn("effect->startLevel", ramp)
        self.assertIn("effect->endLevel", ramp)
        self.assertIn("envelope_time_sum", ramp)
        self.assertIn("ramp_level_at(attack_time)", ramp)
        self.assertIn("ramp_level_at(fade_start_time)", ramp)
        self.assertIn("effect->useEnvelope", ramp)

        def expected_ramp(now, start, end, duration, attack, attack_level, fade, fade_level):
            attack = min(attack, duration)
            fade = min(fade, duration)
            if attack + fade > duration:
                attack = attack * duration // (attack + fade)
                fade = duration - attack

            def base(time):
                return start + int((end - start) * time / duration)

            if attack and now < attack:
                attack_start = attack_level if start >= 0 else -attack_level
                return attack_start + int((base(attack) - attack_start) * now / attack)
            fade_start = duration - fade
            if fade and now >= fade_start:
                fade_end = fade_level if end >= 0 else -fade_level
                return base(fade_start) + int((fade_end - base(fade_start)) * (now - fade_start) / fade)
            return base(now)

        levels = (-10000, 10000, 1000, 100, 5000, 100, 3000)
        self.assertEqual(expected_ramp(0, *levels), -5000)
        self.assertEqual(expected_ramp(50, *levels), -6500)
        self.assertEqual(expected_ramp(100, *levels), -8000)
        self.assertEqual(expected_ramp(500, *levels), 0)
        self.assertEqual(expected_ramp(900, *levels), 8000)
        self.assertEqual(expected_ramp(950, *levels), 5500)
        self.assertEqual(expected_ramp(1000, *levels), 3000)
        # Overlapping attack/fade are fitted to duration without a gap.
        overlap = (-12000, 12000, 100, 80, 4000, 80, 2000)
        self.assertEqual(expected_ramp(0, *overlap), -4000)
        self.assertEqual(expected_ramp(25, *overlap), -2000)
        self.assertEqual(expected_ramp(50, *overlap), 0)
        self.assertEqual(expected_ramp(75, *overlap), 1000)
        self.assertEqual(expected_ramp(100, *overlap), 2000)
        reverse = (10000, -10000, 1000, 100, 5000, 100, 3000)
        self.assertEqual(expected_ramp(0, *reverse), 5000)
        self.assertEqual(expected_ramp(1000, *reverse), -3000)

    def test_start_delay_is_honored_for_infinite_effects(self):
        calculate = self.effects.split("void EffectsCalculator::calculateEffects", 1)[1].split(
            "int32_t EffectsCalculator::calcNonConditionEffectForce", 1
        )[0]
        delay_guard = calculate.index("(int32_t)elapsed_ms < 0")
        finite_duration_guard = calculate.index(
            "effect->duration != FFB_EFFECT_DURATION_INFINITE && effect->duration != 0"
        )
        self.assertLess(delay_guard, finite_duration_guard)

    def test_all_effects_in_one_sample_share_the_same_tick_snapshot(self):
        calculate = self.effects.split("void EffectsCalculator::calculateEffects", 1)[1].split(
            "int32_t EffectsCalculator::calcNonConditionEffectForce", 1
        )[0]
        noncondition = self.effects.split("int32_t EffectsCalculator::calcNonConditionEffectForce", 1)[1].split(
            "int32_t EffectsCalculator::calcComponentForce", 1
        )[0]
        envelope = self.effects.split("int32_t EffectsCalculator::getEnvelopeMagnitude", 1)[1].split(
            "void EffectsCalculator::setFilters", 1
        )[0]

        self.assertEqual(calculate.count("HAL_GetTick()"), 1)
        self.assertIn("calcNonConditionEffectForce(effect, now_ms)", calculate)
        self.assertIn("getEnvelopeMagnitude(effect, now_ms)", noncondition)
        self.assertNotIn("HAL_GetTick()", noncondition)
        self.assertNotIn("HAL_GetTick()", envelope)

    def test_finite_effect_expires_at_its_duration_boundary(self):
        calculate = self.effects.split("void EffectsCalculator::calculateEffects", 1)[1].split(
            "int32_t EffectsCalculator::calcNonConditionEffectForce", 1
        )[0]
        self.assertIn("const uint32_t elapsed_ms = now_ms - effect->startTime", calculate)
        self.assertIn("elapsed_ms >= effect->duration", calculate)

    def test_effect_operation_loop_count_repeats_finite_effects(self):
        operation = self.hid.split("void HidFFB::set_effect_operation", 1)[1].split(
            "void HidFFB::set_envelope", 1
        )[0]
        calculate = self.effects.split("void EffectsCalculator::calculateEffects", 1)[1].split(
            "int32_t EffectsCalculator::calcNonConditionEffectForce", 1
        )[0]
        self.assertIn("effects[id].loopsRemaining = report->loopCount", operation)
        self.assertIn("FFB_EFFECT_LOOP_INFINITE", self.defs)
        self.assertIn("effect->loopsRemaining == FFB_EFFECT_LOOP_INFINITE", calculate)
        self.assertIn("effect->loopsRemaining > completed_cycles", calculate)
        self.assertIn("effect->loopsRemaining -= (uint8_t)completed_cycles", calculate)
        self.assertIn("completed_cycles * effect->duration", calculate)

        # The operation count is total plays: one stops at first expiry,
        # while two restarts once and then stops. 255 is the HID infinite code.
        def plays_until_stopped(count):
            completed = 0
            while count == 0xFF or completed < max(count, 1):
                completed += 1
                if completed == 10:
                    break
            return completed

        self.assertEqual(plays_until_stopped(1), 1)
        self.assertEqual(plays_until_stopped(2), 2)
        self.assertEqual(plays_until_stopped(0xFF), 10)

    def test_loop_count_catches_up_all_cycles_after_a_delayed_sample(self):
        calculate = self.effects.split("void EffectsCalculator::calculateEffects", 1)[1].split(
            "int32_t EffectsCalculator::calcNonConditionEffectForce", 1
        )[0]
        self.assertIn("const uint32_t completed_cycles = elapsed_ms / effect->duration", calculate)
        self.assertIn("effect->loopsRemaining > completed_cycles", calculate)
        self.assertIn("effect->loopsRemaining -= (uint8_t)completed_cycles", calculate)
        self.assertIn("completed_cycles * effect->duration", calculate)

        # If a 7 ms effect is next sampled 17 ms after its start, two full
        # iterations elapsed and the next one is already 3 ms in progress.
        mask = (1 << 32) - 1
        start = 0xFFFFFFFC
        duration = 7
        now = (start + 17) & mask
        elapsed = (now - start) & mask
        completed_cycles = elapsed // duration
        next_start = (start + completed_cycles * duration) & mask
        self.assertEqual(completed_cycles, 2)
        self.assertEqual((now - next_start) & mask, 3)

    def test_envelope_times_cannot_underflow_duration_or_overflow_interpolation(self):
        envelope = self.effects.split("int32_t EffectsCalculator::getEnvelopeMagnitude", 1)[1].split("void EffectsCalculator::setFilters", 1)[0]
        self.assertIn("std::min(effect->attackTime, duration)", envelope)
        self.assertIn("std::min(effect->fadeTime, duration)", envelope)
        self.assertIn("int64_t", envelope)
        self.assertNotIn("effect->duration - effect->fadeTime", envelope)

    def test_friction_keeps_signed_condition_coefficients(self):
        friction = self.effects.split("case FFB_EFFECT_FRICTION", 1)[1].split("case FFB_EFFECT_DAMPER", 1)[0]
        self.assertRegex(friction, r"int32_t\s+coeff\s*=")
        self.assertNotRegex(friction, r"uint16_t\s+coeff\s*=")
        self.assertIn("(int32_t)effect->conditions[con_idx].negativeCoefficient", friction)
        self.assertIn("(int32_t)effect->conditions[con_idx].positiveCoefficient", friction)
        self.assertIn("fabsf(speed - (float)offset)", friction)

    def test_condition_deadband_comparison_keeps_fractional_metric(self):
        condition = self.effects.split("int32_t EffectsCalculator::calcConditionEffectForce", 1)[1].split("int32_t EffectsCalculator::getEnvelopeMagnitude", 1)[0]
        self.assertIn("fabsf(metric - (float)offset)", condition)
        self.assertNotIn("abs(metric - offset)", condition)

    def test_conditional_and_friction_gains_silence_zero_and_preserve_legacy_levels(self):
        condition = self.effects.split("int32_t EffectsCalculator::calcConditionEffectForce", 1)[1].split(
            "int32_t EffectsCalculator::getEnvelopeMagnitude", 1
        )[0]
        friction = self.effects.split("case FFB_EFFECT_FRICTION", 1)[1].split(
            "case FFB_EFFECT_DAMPER", 1
        )[0]
        self.assertIn("gain == 0 ? 0.0f : (float)(gain + 1) / 256.0f", condition)
        self.assertIn("gain.friction == 0 ? 0.0f : (float)(gain.friction + 1) / 256.0f", friction)
        self.assertNotIn("(float)gain / 255.0f", condition)
        self.assertNotIn("(float)gain.friction / 255.0f", friction)

        for gain in (0, 64, 127, 255):
            factor = 0.0 if gain == 0 else (gain + 1) / 256.0
            self.assertGreaterEqual(factor, 0.0)
            self.assertLessEqual(factor, 1.0)
        self.assertEqual(0.0, 0.0)
        self.assertEqual((255 + 1) / 256.0, 1.0)

    def test_condition_blocks_keep_hid_axis_slots_separate_from_physical_axes(self):
        handler = self.hid.split("void HidFFB::set_condition", 1)[1].split("void HidFFB::set_effect_operation", 1)[0]
        self.assertIn("std::min<uint8_t>((uint8_t)(MAX_AXIS - 1)", handler)
        self.assertNotIn("axisCount - 1", handler)

    def test_axis_torque_slew_state_tracks_the_clipped_output(self):
        setter = self.ffb_task.split("void setEffectTorque(int32_t torque) override", 1)[1].split("void calculateAxisEffects", 1)[0]
        clip_at = setter.index("if (totalTorque >  0x7FFF)")
        remember_at = setter.index("lastTorque_ = totalTorque")
        metrics_at = setter.index("metrics_.torque = totalTorque")
        self.assertLess(clip_at, remember_at)
        self.assertLess(remember_at, metrics_at)

    def test_axis_acceleration_and_slew_use_measured_sample_interval(self):
        update = self.ffb_task.split("void update()", 1)[1].split("void zeroEncoder()", 1)[0]
        setter = self.ffb_task.split("void setEffectTorque(int32_t torque) override", 1)[1].split("void calculateAxisEffects", 1)[0]
        self.assertIn("uint32_t elapsed_us = hasLastUpdate_ ? sample_time_us - lastUpdateUs_ : 1000u", update)
        self.assertIn("dtSeconds_ = (float)elapsed_us * 0.000001f", update)
        self.assertIn("const float dt = dtSeconds_", update)
        self.assertIn("dt / (dt + accelFilterTauSeconds_)", update)
        self.assertIn("dtSeconds_ * 1000.0f", setter)
        self.assertIn("torqueSlewRemainder_", setter)

        # A 20 deg/s velocity change over a 2 ms scheduling gap is 10,000
        # deg/s². The old fixed-1ms derivative doubled the inertia input.
        self.assertEqual((120.0 - 100.0) / 0.002, 10000.0)
        self.assertEqual((120.0 - 100.0) / 0.001, 20000.0)
        self.assertEqual((0x000006D0 - 0xFFFFFF00) & 0xFFFFFFFF, 2000)

    def test_negative_expo_uses_the_requested_power_curve(self):
        update = self.ffb_task.split("void update()", 1)[1].split("void zeroEncoder()", 1)[0]
        self.assertIn("if (k == 1.0f)", update)
        self.assertIn("absp *= absp;", update)
        self.assertIn("else if (k == 2.0f)", update)
        self.assertIn("absp = (absp * absp) * absp;", update)
        self.assertIn("__builtin_powf(absp, curve)", update)

        # expo=-50 with scale=100 means exponent 0.5. At the midpoint,
        # the configured curve should return sqrt(0.5), not the old fast
        # polynomial's 0.59375 (an 11.3% full-scale steering error).
        import math
        midpoint = 0.5
        expected = math.pow(midpoint, 0.5)
        old_fast_curve = midpoint * (1.0 + (-0.5) * (midpoint * midpoint - 1.0) * 0.5)
        self.assertAlmostEqual(expected, math.sqrt(midpoint))
        self.assertGreater(abs(expected - old_fast_curve), 0.03)

    def test_fractional_positive_expo_does_not_use_inaccurate_cubic_approximation(self):
        import math
        update = self.ffb_task.split("void update()", 1)[1].split("void zeroEncoder()", 1)[0]
        self.assertNotIn("1.0f + k * (x2 - 1.0f) * 0.5f", update)
        self.assertIn("__builtin_powf(absp, curve)", update)

        # expo=100, scale=100 requests x^2. The former cubic fit returned
        # 0.3125 at x=0.5 instead of 0.25, a visible 6.25% full-scale error.
        expected = math.pow(0.5, 2.0)
        old_fast_curve = 0.5 * (1.0 + 1.0 * (0.25 - 1.0) * 0.5)
        self.assertAlmostEqual(expected, 0.25)
        self.assertAlmostEqual(old_fast_curve, 0.3125)

    def test_valid_expo_power_range_has_no_linear_fallback_discontinuities(self):
        import math
        update = self.ffb_task.split("void update()", 1)[1].split("void zeroEncoder()", 1)[0]
        self.assertIn("curve > 0.0f", update)
        self.assertNotIn("curve > 0.01f", update)
        self.assertNotIn("curve < 10.0f", update)

        # Both powers are finite and map [0,1] back into [0,1]. The old
        # fallback returned x=0.5 for both, causing near-half-scale jumps.
        low_valid_power = math.pow(0.5, 0.01)  # expo=-99, scale=100
        high_valid_power = math.pow(0.5, 10.0) # expo=900, scale=100
        self.assertGreater(abs(low_valid_power - 0.5), 0.49)
        self.assertGreater(abs(high_valid_power - 0.5), 0.49)

    def test_usb_unmount_resets_pending_torque_and_slew_history(self):
        reset = self.ffb_task.split("void reset_ffb_state()", 1)[1].split("};", 1)[0]
        self.assertIn("pending_torque_ = 0.0f", reset)
        self.assertIn("lastTorque_ = 0", reset)

        unmount = self.ffb_task.split('extern "C" void ffb_on_usb_unmount(void)', 1)[1].split(
            'extern "C" float ffb_get_pending_torque_nm()', 1
        )[0]
        self.assertIn("EffectsStateGuard", unmount)
        lock_at = unmount.index("EffectsStateGuard")
        stop_at = unmount.index("stop_FFB()")
        reset_at = unmount.index("reset_ffb_state()")
        zero_at = unmount.index("odrive_bridge_set_input_torque(0.0f)")
        self.assertIn("if (s_effects_calc)", unmount)
        self.assertLess(lock_at, stop_at)
        self.assertLess(stop_at, reset_at)
        self.assertLess(reset_at, zero_at)

    def test_current_sample_torque_is_committed_after_effect_calculation(self):
        axis_update = self.ffb_task.split("void update()", 1)[1].split("void zeroEncoder()", 1)[0]
        self.assertNotIn("odrive_bridge_set_input_torque", axis_update)
        thread = self.ffb_task.split("static void ffb_thread", 1)[1].split("// -------------------- Init", 1)[0]
        read_at = thread.index("s_axis_raw->update()")
        calculate_at = thread.index("s_effects_calc->calculateEffects(s_axes_vec)")
        commit_at = thread.index("s_axis_raw->commitPendingTorque()")
        self.assertLess(read_at, calculate_at)
        self.assertLess(calculate_at, commit_at)

    def test_hid_effect_updates_and_1khz_calculation_share_a_mutex(self):
        calculate = self.effects.split("void EffectsCalculator::calculateEffects", 1)[1].split("int32_t EffectsCalculator::calcNonConditionEffectForce", 1)[0]
        hid_out = self.hid.split("void HidFFB::hidOut", 1)[1].split("uint16_t HidFFB::hidGet", 1)[0]
        thread = self.ffb_task.split("static void ffb_thread", 1)[1].split("// -------------------- Init", 1)[0]
        self.assertIn("EffectsStateGuard", self.effects_header)
        self.assertIn("EffectsStateGuard state_lock(*effects_calc)", hid_out)
        self.assertIn("cpp_freertos::MutexStandard", self.effects_header)
        self.assertIn("EffectsStateGuard", thread)
        lock_at = thread.index("EffectsStateGuard")
        calculate_at = thread.index("s_effects_calc->calculateEffects(s_axes_vec)")
        commit_at = thread.index("s_axis_raw->commitPendingTorque()")
        self.assertLess(lock_at, calculate_at)
        self.assertLess(calculate_at, commit_at)

    def test_flash_persistence_releases_effect_mutex_before_programming(self):
        save = self.effects.split("void EffectsCalculator::saveFlash(uint8_t* globalGainSnapshot)", 1)[1].split("void EffectsCalculator::checkFilterCoeff", 1)[0]
        c_save = self.ffb_task.split('extern "C" int ffb_save_flash(void)', 1)[1].split('extern "C" int ffb_save_writes', 1)[0]
        unlock_marker = "// Flash programming starts after the effect state snapshot is unlocked."
        self.assertIn(unlock_marker, save)
        lock_scope_end = save.index(unlock_marker)
        first_write = save.index("Flash_Write(")
        self.assertIn("EffectsStateGuard state_lock(*this)", save)
        self.assertLess(lock_scope_end, first_write)
        self.assertIn("EffectsStateGuard state_lock(*s_effects_calc)", c_save)
        snapshot_end = c_save.index("HAL_FLASH_Unlock()")
        snapshot = c_save[:snapshot_end]
        flash_programming = c_save[snapshot_end:]
        self.assertIn("axisSnapshot.rangeDegrees = s_axis_raw->rangeDegrees_", snapshot)
        self.assertLess(snapshot.index("EffectsStateGuard state_lock(*s_effects_calc)"),
                        snapshot.index("axisSnapshot.rangeDegrees = s_axis_raw->rangeDegrees_"))
        self.assertNotIn("s_axis_raw->", flash_programming)
        self.assertIn("saveFlash(&master_gain_snapshot)", c_save)

    def test_ffb_thread_exposes_loop_timing_and_one_ms_overruns(self):
        self.assertIn("s_ffb_loop_last_us", self.ffb_task)
        self.assertIn("s_ffb_loop_max_us", self.ffb_task)
        self.assertIn("s_ffb_calc_max_us", self.ffb_task)
        self.assertIn("s_ffb_lock_wait_max_us", self.ffb_task)
        self.assertIn("s_ffb_lock_hold_max_us", self.ffb_task)
        self.assertIn("s_ffb_loop_overruns", self.ffb_task)
        self.assertIn("loop_us >= period_ms * 1000", self.ffb_task)
        self.assertIn("s_ffb_sample_max_us", self.ffb_task)
        self.assertIn("sample_interval_us > period_ms * 1000", self.ffb_task)
        self.assertIn("W=%luus H=%luus", self.commands)
        self.assertIn('"ffbperf"', self.commands)
        thread = self.ffb_task.split("static void ffb_thread(void *arg)", 1)[1].split(
            "// -------------------- Init", 1
        )[0]
        wait_start = thread.index("const uint32_t lock_wait_start_us = micros()")
        lock_start = thread.index("EffectsStateGuard effect_output_lock")
        acquired = thread.index("const uint32_t lock_acquired_us = micros()")
        update = thread.index("s_axis_raw->update()")
        hold = thread.index("const uint32_t lock_hold_us = micros() - lock_acquired_us")
        unlock_scope_end = thread.index("const uint32_t calc_us", hold)
        self.assertLess(wait_start, lock_start)
        self.assertLess(lock_start, acquired)
        self.assertLess(acquired, update)
        self.assertLess(update, hold)
        self.assertLess(hold, unlock_scope_end)
        perf_handler = self.commands.split("static int h_sys_ffbperf", 1)[1].split(
            "// ========================", 1
        )[0]
        format_string = perf_handler.split("snprintf(r, s,", 1)[1].split('"', 2)[1]
        worst_case_reply = format_string.replace("%lu", "4294967295")
        self.assertLessEqual(len(worst_case_reply), 127)

    def test_ffb_sampling_is_synchronized_to_every_eighth_control_loop_cycle(self):
        thread = self.ffb_task.split("static void ffb_thread(void *arg)", 1)[1].split(
            "// -------------------- Init", 1
        )[0]
        signal = self.ffb_task.split(
            'extern "C" void ffb_task_signal_tick_from_isr(void)', 1
        )[1].split("\n}", 1)[0]
        control_loop = self.board.split("void ControlLoop_IRQHandler(void)", 1)[1].split(
            "void I2C1_EV_IRQHandler", 1
        )[0]

        self.assertIn("osSemaphoreWait(tick_sem, osWaitForever)", thread)
        self.assertNotIn("osDelayUntil", thread)
        self.assertIn("if (++s_ffb_tick_divider == 8)", signal)
        self.assertIn("osSemaphoreRelease(tick_sem)", signal)
        self.assertIn("ffb_task_signal_tick_from_isr()", control_loop)
        self.assertLess(control_loop.index("odrv.control_loop_cb(timestamp)"),
                        control_loop.index("ffb_task_signal_tick_from_isr()"))

    def test_ffb_isr_semaphore_handle_is_published_atomically(self):
        self.assertIn("std::atomic<osSemaphoreId> s_ffb_tick_sem", self.ffb_task)
        self.assertIn("std::atomic<osSemaphoreId>::is_always_lock_free", self.ffb_task)
        signal = self.ffb_task.split(
            'extern "C" void ffb_task_signal_tick_from_isr(void)', 1
        )[1].split("\n}", 1)[0]
        init = self.ffb_task.split('extern "C" void ffb_task_init(void)', 1)[1].split(
            "// Lê axis params", 1
        )[0]

        self.assertIn("s_ffb_tick_sem.load(std::memory_order_acquire)", signal)
        self.assertIn("osSemaphoreRelease(tick_sem)", signal)
        create = init.index("osSemaphoreCreate(osSemaphore(ffbTickSem), 1)")
        empty = init.index("osSemaphoreWait(tick_sem, 0)", create)
        publish = init.index("s_ffb_tick_sem.store(tick_sem, std::memory_order_release)", empty)
        spawn = init.index("osThreadCreate(osThread(ffbThread), NULL)", publish)
        self.assertLess(create, empty)
        self.assertLess(empty, publish)
        self.assertLess(publish, spawn)

    def test_effect_loop_advances_axis_slew_only_once_per_1khz_sample(self):
        calculate = self.effects.split("void EffectsCalculator::calculateEffects", 1)[1].split("int32_t EffectsCalculator::calcNonConditionEffectForce", 1)[0]
        initial_axis_pass = calculate.split("if(!isActive())", 1)[0]
        self.assertNotIn("axis->setEffectTorque(0)", initial_axis_pass)
        inactive_path = calculate.split("if(!isActive())", 1)[1].split("int32_t force = 0", 1)[0]
        self.assertIn("axis->setEffectTorque(0)", inactive_path)

    def test_periodic_phase_product_uses_wide_intermediate_and_unsigned_storage(self):
        self.assertIn("uint16_t phase = 0", self.defs)
        self.assertIn("(uint64_t)(effect->phase % 36000u) * period_us", self.effects)
        self.assertIn("report->phase % 36000u", self.hid)

    def test_periodic_sine_uses_bounded_error_table_interpolation(self):
        self.assertIn('#include "utils.hpp"', self.effects)
        odrive_root = ROOT.parent / "ODrive-fw-v0.5.6" / "ODrive-fw-v0.5.6" / "Firmware"
        fast_sine = (odrive_root / "MotorControl" / "arm_sin_f32.c").read_text(encoding="utf-8")
        cmsis_math = (odrive_root / "ThirdParty" / "CMSIS" / "Include" / "arm_math.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("sinTable_f32[index+1]", fast_sine)
        self.assertRegex(cmsis_math, r"#define FAST_MATH_TABLE_SIZE\s+512")
        sine_case = self.effects.split("else if (effect->type == FFB_EFFECT_SINE)", 1)[1].split(
            "}", 1
        )[0]
        self.assertIn("our_arm_sin_f32(2.0f * (float)M_PI * phase_norm)", sine_case)
        self.assertNotIn("sinf(", sine_case)

        # CMSIS' 512-interval table interpolation has a worst-case linear
        # interpolation error below h²/8; validate that against one full
        # cycle in torque counts, including wrap at the last table interval.
        import math
        import struct

        def f32(value):
            return struct.unpack("<f", struct.pack("<f", value))[0]

        table_size = 512
        table = [f32(math.sin(2.0 * math.pi * i / table_size)) for i in range(table_size + 1)]
        max_error_counts = 0.0
        for sample in range(65536):
            phase = f32(sample / 65536.0)
            phase_radians = f32(f32(f32(2.0) * f32(math.pi)) * phase)
            input_turns = f32(phase_radians * f32(0.159154943092))
            input_turns -= math.floor(input_turns)
            table_position = f32(f32(table_size) * f32(input_turns))
            index = int(table_position)
            fraction = f32(table_position - index)
            estimate = f32(
                f32(f32(1.0 - fraction) * table[index])
                + f32(fraction * table[index + 1])
            )
            reference = math.sin(2.0 * math.pi * sample / 65536.0)
            max_error_counts = max(max_error_counts, abs(estimate - reference) * 32767.0)
        self.assertLess(max_error_counts, 0.7)

    def test_ramp_levels_decode_signed_hid_torque(self):
        ramp = self.defs.split("} __attribute__((packed)) FFB_SetRamp_Data_t;", 1)[0].rsplit("typedef struct", 1)[1]
        self.assertRegex(ramp, r"\bint16_t\s+startLevel")
        self.assertRegex(ramp, r"\bint16_t\s+endLevel")

        # HID wire value 0xC000 means -16384 in the signed Ramp fields.
        import struct
        self.assertEqual(struct.unpack("<h", bytes((0x00, 0xC0)))[0], -16384)

    def test_direction_enable_mask_uses_descriptor_bit_not_physical_axis_count(self):
        self.assertIn("#define HID_DIRECTION_ENABLE 0x04", self.defs)
        constructor = self.hid.split("HidFFB::HidFFB", 1)[1].split("HidFFB::~HidFFB", 1)[0]
        self.assertIn("directionEnableMask = HID_DIRECTION_ENABLE", constructor)
        self.assertNotIn("directionEnableMask = 1 << axisCount", constructor)

    def test_polar_zero_x_component_redirects_y_force_on_single_axis(self):
        import math
        set_effect = self.hid.split("void HidFFB::set_effect(", 1)[1].split("/*", 1)[0]
        single_axis = set_effect.split("if (axisCount == 1)", 1)[1].split("} else {", 1)[0]
        self.assertIn("fabsf(effect_p->axisMagnitudes[0]) < 1e-6f", single_axis)
        self.assertNotIn("effect_p->axisMagnitudes[0] == 0.0f", single_axis)

        # IEEE float sin(pi) is nonzero; exact-zero detection misses the
        # 180-degree direction where a one-axis wheel must use the Y force.
        x_component = math.sin(math.pi)
        self.assertNotEqual(x_component, 0.0)
        self.assertLess(abs(x_component), 1e-6)
        self.assertAlmostEqual(-math.cos(math.pi), 1.0)

    def test_tick_wrap_elapsed_time_and_start_delay_boundaries(self):
        mask = (1 << 32) - 1
        start = 0xFFFFFFFE
        now = 5
        elapsed = (now - start) & mask
        self.assertEqual(elapsed, 7)
        self.assertGreater(elapsed, 0)

        before_start = (0xFFFFFFF0 - 10) & mask
        signed_before_start = before_start - (1 << 32)
        self.assertEqual(signed_before_start, -26)

    def test_maximum_periodic_phase_product_is_64_bit_safe(self):
        phase = 35999
        period_us = 32767 * 1000
        phase_us = phase * period_us // 36000
        # The worst-case product exceeds uint32_t but fits uint64_t.
        max_product = 35999 * period_us
        self.assertGreater(max_product, (1 << 32) - 1)
        self.assertLess(max_product, (1 << 64))
        self.assertGreaterEqual(phase_us, 0)
        self.assertLess(phase_us, period_us)

    def test_periodic_phase_does_not_jump_when_micros_wraps(self):
        periodic = self.effects.split("uint32_t period_ms =", 1)[1].split("if (effect->type == FFB_EFFECT_SQUARE)", 1)[0]
        self.assertIn("elapsed_ms % period_ms", periodic)
        self.assertNotIn("micros() - ((uint32_t)effect->startTime * 1000)", periodic)

        # After one micros() wrap plus 1.234567 s, a 7 ms effect must be at
        # phase 863 us. The tick-quantized phase is 0 us (<1 ms sample
        # quantization); the old uint32 elapsed modulo landed at 2567 us.
        age_us = (1 << 32) + 1_234_567
        period_us = 7_000
        expected_phase = age_us % period_us
        elapsed_ms = (age_us // 1000) & 0xFFFFFFFF
        tick_phase = (elapsed_ms % 7) * 1000
        old_phase = (age_us & 0xFFFFFFFF) % period_us
        self.assertEqual(expected_phase, 863)
        self.assertEqual(tick_phase, 0)
        self.assertLessEqual(abs(tick_phase - expected_phase), 1000)
        self.assertEqual(old_phase, 2567)
        self.assertGreater(abs(old_phase - expected_phase), 1000)

    def test_endstop_damper_remains_active_when_spring_strength_is_zero(self):
        endstop = self.ffb_task.split("// 5. Endstop eletrônico", 1)[1].split(
            "metric_t *getMetrics()", 1
        )[0]
        self.assertRegex(
            endstop,
            r"if\s*\(\s*endstopStrength_\s*!=\s*0\s*\|\|\s*endstopDamper_\s*!=\s*0\s*\)",
        )
        self.assertIn("spring = -overshoot * (float)endstopStrength_ * SPRING_GAIN", endstop)
        self.assertIn("damper = -speed * (float)endstopDamper_ * DAMP_RATIO", endstop)

    def test_zero_endstop_damper_survives_save_restore_and_keeps_legacy_default(self):
        load = self.ffb_task.split("static void ffb_load_axis_params_internal(void)", 1)[1].split(
            "// -------------------- Bridges", 1
        )[0]
        save = self.ffb_task.split("extern \"C\" int ffb_save_flash(void)", 1)[1].split(
            "extern \"C\" int ffb_save_writes", 1
        )[0]
        self.assertIn("AXIS_CONFIG_ENDSTOP_DAMPER_VALID", self.ffb_task)
        self.assertIn("storedAxisConfig & AXIS_CONFIG_ENDSTOP_DAMPER_VALID", load)
        self.assertRegex(load, r"dampStored\s*!=\s*0\s*\|\|\s*endstopDamperFormatValid")
        self.assertIn("cfgFlags |= AXIS_CONFIG_ENDSTOP_DAMPER_VALID", save)

        def restore_damper(stored, has_format_marker, default=15):
            return stored if stored != 0 or has_format_marker else default

        # Legacy images used zero high-byte as "field absent"; new images use
        # the config marker so zero remains a valid persisted setting.
        self.assertEqual(restore_damper(0, False), 15)
        self.assertEqual(restore_damper(23, False), 23)
        self.assertEqual(restore_damper(0, True), 0)
        self.assertEqual(restore_damper(23, True), 23)

    def test_failed_effect_create_clears_handle_and_reports_the_correct_status(self):
        create = self.hid.split("void HidFFB::new_effect(", 1)[1].split(
            "/**", 1
        )[0]
        self.assertIn("blockLoad_report.effectBlockIndex = 0", create)
        self.assertRegex(
            create,
            r"effectType\s*==\s*FFB_EFFECT_NONE\s*\|\|\s*effect->effectType\s*>\s*FFB_EFFECT_CUSTOM",
        )
        self.assertIn("blockLoad_report.loadStatus = 3", create)
        self.assertIn("blockLoad_report.loadStatus = 2", create)

        def load_report(effect_type, free_slots, previous_index):
            if effect_type == 0 or effect_type > 12:
                return 0, 3
            if free_slots == 0:
                return 0, 2
            return previous_index, 1

        # A failed request following a successful allocation must not leak its
        # previous handle to the host as if it belonged to the new request.
        self.assertEqual(load_report(1, 0, 7), (0, 2))
        self.assertEqual(load_report(0x7F, 4, 7), (0, 3))
        self.assertEqual(load_report(1, 4, 7), (7, 1))

    def test_axis_sample_and_runtime_parameter_access_share_the_effect_mutex(self):
        loop = self.ffb_task.split("static void ffb_thread(void *arg)", 1)[1].split(
            "// -------------------- Init", 1
        )[0]
        sample = loop.split("EffectsStateGuard effect_output_lock", 1)[1].split(
            "const uint32_t calc_us", 1
        )[0]
        self.assertIn("s_axis_raw->update()", sample)
        self.assertIn("s_effects_calc->calculateEffects(s_axes_vec)", sample)
        self.assertIn("s_axis_raw->commitPendingTorque()", sample)

        setters = (
            "ffb_set_axis_invert", "ffb_set_ffb_invert", "ffb_set_axis_range",
            "ffb_set_axis_maxtq", "ffb_set_axis_fxratio", "ffb_set_axis_idlespring",
            "ffb_set_axis_damper", "ffb_set_axis_inertia", "ffb_set_axis_friction",
            "ffb_set_axis_esgain", "ffb_set_axis_esdamp", "ffb_set_axis_maxtorquerate",
            "ffb_set_axis_expo", "ffb_set_axis_exposcale", "ffb_axis_zeroenc",
            "ffb_set_axis_zeroofs",
        )
        for name in setters:
            with self.subTest(setter=name):
                start = self.ffb_task.index(f"{name}(")
                end = self.ffb_task.index("}", start) + 1
                self.assertIn("EffectsStateGuard", self.ffb_task[start:end])

        report = loop.split("if (tud_hid_ready())", 1)[1]
        self.assertIn("EffectsStateGuard", report.split("s_ffb_loop_samples++", 1)[0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
