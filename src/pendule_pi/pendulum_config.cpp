#include <yamlcfg/config_node.hpp>
#include <yamlcfg/constraints.hpp>

#include "pendule_pi/pendulum_config.hpp"
#include "pendule_pi/sockets_config.hpp"

#include <sstream>

namespace {
    using yamlcfg::ConfigNode;
    using yamlcfg::InRange;
    using yamlcfg::not_empty;
    using yamlcfg::positive;

    // Constraints specific to this hardware / protocol
    constexpr InRange<int> gpio_pin{1, 26};
    constexpr InRange<int> tcp_port{1, 65535};

    SwitchConfig parse_switch(const ConfigNode &n) {
        SwitchConfig s{};
        s.pin = n.get_required<int>("pin", gpio_pin);
        s.normally_up = n.get_or<bool>("normally_up", true);
        s.use_internal_pull_resistor = n.get_or<bool>("use_internal_pull_resistor", true);
        s.debounce_us = n.get_or<int>("debounce_us", 500, positive);
        return s;
    }

    EncoderConfig parse_encoder(const ConfigNode &n) {
        EncoderConfig e{};
        e.pin_a = n.get_required<int>("pin_a", gpio_pin);
        e.pin_b = n.get_required<int>("pin_b", gpio_pin);
        n.ensure(e.pin_a != e.pin_b, "pin_b", "must be different from pin_a");
        return e;
    }

    MotorConfig parse_motor(const ConfigNode &n) {
        MotorConfig m{};
        m.direction_pin = n.get_required<int>("direction_pin", gpio_pin);
        m.pwm_pin = n.get_required<int>("pwm_pin", gpio_pin);
        n.ensure(m.direction_pin != m.pwm_pin, "pwm_pin", "must be different from direction_pin");
        return m;
    }

    SafetyThresholds parse_safety_thresholds(const ConfigNode &n) {
        SafetyThresholds t{};
        t.hard = n.get_required<double>("hard", positive);
        t.margin = n.get_required<double>("margin", positive);
        n.ensure(t.margin > t.hard, "margin", "must be greater than 'hard'");
        return t;
    }

    PwmOffsets parse_pwm_offsets(const ConfigNode &n) {
        PwmOffsets o{};
        o.low = n.get_required<int>("low", InRange<int>{0, 255});
        o.high = n.get_required<int>("high");
        n.ensure(o.high >= o.low, "high", "must be greater than or equal to 'low'");
        return o;
    }

    HardwareSpecs parse_hardware_specs(const ConfigNode &n) {
        HardwareSpecs h{};
        h.encoders_step_per_turn = n.get_required<int>("encoders_step_per_turn", positive);
        h.timing_belt_module = n.get_required<double>("timing_belt_module", positive);
        h.pulley_teeth_count = n.get_required<int>("pulley_teeth_count", positive);
        return h;
    }

    PendulumConfig build_config(const ConfigNode &root) {
        PendulumConfig c{};
        c.period_ms = root.get_or<int>("period_ms", 20, positive);
        c.calibration_pwm = root.get_required<int>("calibration_pwm", positive);
        c.cutoff_frequency = root.get_required<double>("cutoff_frequency", positive);
        c.right_switch = parse_switch(root.child("right_switch"));
        c.left_switch = parse_switch(root.child("left_switch"));
        c.motor = parse_motor(root.child("motor"));
        c.hardware_specs = parse_hardware_specs(root.child("hardware_specs"));
        c.position_encoder = parse_encoder(root.child("position_encoder"));
        c.angle_encoder = parse_encoder(root.child("angle_encoder"));
        c.safety_thresholds = parse_safety_thresholds(root.child("safety_thresholds"));
        c.pwm_offsets = parse_pwm_offsets(root.child("pwm_offsets"));
        c.sockets = parse_sockets(root.child("sockets"));
        return c;
    }
} // namespace

PendulumConfig load_pendulum_config(const std::string &path) {
    return yamlcfg::load_file(path, build_config);
}

PendulumConfig parse_pendulum_config(const std::string &yaml_text) {
    return yamlcfg::parse_text(yaml_text, build_config);
}

std::string get_pendulum_config_example() {
    return "# Configuration of the left switch (encoder side)\n"
            "left_switch:\n"
            "  # GPIO pin used to read the state of the switch.\n"
            "  pin: 17\n"
            "  # If true, then the pin should read 'high' when at rest.\n"
            "  normally_up: true\n"
            "  # If true, then the internal pull-up or pull-down resistor is activated.\n"
            "  use_internal_pull_resistor: true\n"
            "  # micro-second window for pigpio glitch filter (default: 5000 if not provided)\n"
            "  debounce_us: 5000\n"
            "\n"
            "# Configuration of the right switch (motor side).\n"
            "right_switch:\n"
            "  # GPIO pin used to read the state of the switch.\n"
            "  pin: 18\n"
            "  # If true, then the pin should read 'high' when at rest.\n"
            "  normally_up: true\n"
            "  # If true, then the internal pull-up or pull-down resistor is activated.\n"
            "  use_internal_pull_resistor: true\n"
            "  # micro-second window for pigpio glitch filter (default: 5000 if not provided)\n"
            "  debounce_us: 5000\n"
            "\n"
            "# Configuration of the position encoder (the one connected to\n"
            "# the transmission belt).\n"
            "position_encoder:\n"
            "  # Pins corresponding to the two phases of the encoder.\n"
            "  pin_a: 20\n"
            "  pin_b: 21\n"
            "\n"
            "# Configuration of the angle encoder (the one that acts as\n"
            "# the pivot for the rod).\n"
            "angle_encoder:\n"
            "  # Pins corresponding to the two phases of the encoder.\n"
            "  pin_a: 26\n"
            "  pin_b: 19\n"
            "\n"
            "# Configuration of the motor.\n"
            "motor:\n"
            "  # Pin used to send PWM signals to the motor controller.\n"
            "  pwm_pin: 24\n"
            "  # Pin used to invert the rotation direction of the motor.\n"
            "  direction_pin: 16\n"
            "\n"
            "hardware_specs:\n"
            "  # Encoders specifications\n"
            "  # IMPORTANT :\n"
            "  #   Provide the physical value, as indicated in the maker's datasheet,\n"
            "  #   NOT the one used in internal computations.\n"
            "  encoders_step_per_turn: 600\n"
            "\n"
            "  # Mechanical specifications of the timing belt and pulley.\n"
            "  timing_belt_module: 2.0         # mm\n"
            "  pulley_teeth_count: 20\n"
            "\n"
            "# Safety distances to stop the pendulum, in meters.\n"
            "safety_thresholds:\n"
            "  # Minimum allowed distance from the switches. If violated, the interface\n"
            "  # shuts-down. It should be positive. Negative values will be inverted.\n"
            "  # Passing a zero is not illegal, but very unsafe.\n"
            "  hard: 0.05\n"
            "  # added margin for base evolution during demo\n"
            "  # The base is stopped when closer than hard + margin from the end position\n"
            "  margin: 0.10\n"
            "\n"
            "# Offsets to be applied to pwm commands. Both should be integers.\n"
            "pwm_offsets:\n"
            "  # When a negative pwm is requested, actually apply pwm-low. If low is\n"
            "  # negative, zero is used instead.\n"
            "  low: 10\n"
            "  # When a positive pwm is requested, actually apply pwm+high. If high is\n"
            "  # negative, zero is used instead.\n"
            "  high: 10\n"
            "\n"
            "# PWM command to be used when performing calibration.\n"
            "calibration_pwm: 60\n"
            "\n"
            "# Control period in milliseconds (integer). Optional parameter, defaults to 20.\n"
            "period_ms: 20\n"
            "\n"
            "# Cutoff frequency, in Hz, used in filtering. It should be less than half the\n"
            "# sampling frequency. You can disable filtering by giving a negative value.\n"
            "cutoff_frequency: 12.5\n"
            "\n"
            "# Configuration of the sockets used to exchange information.\n"
            "sockets:\n"
            "  # Host to bind the connections.\n"
            "  host: \"*\"\n"
            "  # Port to send the state to client applications.\n"
            "  state_port: 10001\n"
            "  # Port to receive commands from clients.\n"
            "  command_port: 10002\n";
}

GPIOMapping get_GPIO_mapping(const PendulumConfig &config) {
    return {
        .left_switch = {"left switch", config.left_switch.pin},
        .right_switch = {"right switch", config.right_switch.pin},
        .position_encoder_A = {"position encoder A", config.position_encoder.pin_a},
        .position_encoder_B = {"position encoder B", config.position_encoder.pin_b},
        .angle_encoder_A = {"angle encoder A", config.angle_encoder.pin_a},
        .angle_encoder_B = {"angle encoder B", config.angle_encoder.pin_b},
        .motor_direction = {"motor DIR", config.motor.direction_pin},
        .motor_pwm = {"motor PWM", config.motor.pwm_pin}
    };
}
