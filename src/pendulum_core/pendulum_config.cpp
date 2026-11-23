#include <yamlcfg/config_node.hpp>
#include <yamlcfg/constraints.hpp>

#include "pendulum_core/pendulum_config.hpp"
#include "pendulum_core/sockets_config.hpp"

namespace {
    using yamlcfg::ConfigNode;
    using yamlcfg::InRange;
    using yamlcfg::not_empty;
    using yamlcfg::positive;

    // Constraints specific to this hardware / protocol
    constexpr InRange<int> GPIO_PIN{1, 26};
    // Upper bound of the pigpio glitch filter delay (microseconds)
    constexpr InRange<int> GLITCH_FILTER_US{0, 300000};

    SwitchConfig parse_switch(const ConfigNode &n) {
        const SwitchConfig s = {
            .pin = n.get_required<int>("pin", GPIO_PIN),
            .normally_up = n.get_or<bool>("normally_up", true),
            .use_internal_pull_resistor = n.get_or<bool>("use_internal_pull_resistor", true),
            .debounce_us = n.get_or<int>("debounce_us", 500, positive)
        };
        return s;
    }

    void parse_encoder_shared_fields(const ConfigNode &n, EncoderConfig &e) {
        e.pin_a = n.get_required<int>("pin_a", GPIO_PIN);
        e.pin_b = n.get_required<int>("pin_b", GPIO_PIN);
        e.steps_per_turn = n.get_required<int>("steps_per_turn", positive);
        e.glitch_filter_us = n.get_or<int>("glitch_filter_us", 0, GLITCH_FILTER_US);
        n.ensure(e.pin_a != e.pin_b, "pin_b", "must be different from pin_a");
    }

    AngleEncoderConfig parse_angle_encoder(const ConfigNode &n) {
        AngleEncoderConfig e{};
        parse_encoder_shared_fields(n, e);
        // no additional field
        return e;
    }

    PositionEncoderConfig parse_position_encoder(const ConfigNode &n) {
        PositionEncoderConfig e{};
        parse_encoder_shared_fields(n, e);
        e.belt_module_mm = n.get_required<double>("belt_module_mm", positive);
        e.pulley_teeth_count = n.get_required<int>("pulley_teeth_count", positive);
        return e;
    }

    MotorConfig parse_motor(const ConfigNode &n) {
        const MotorConfig m = {
            .direction_pin = n.get_required<int>("direction_pin", GPIO_PIN),
            .pwm_pin = n.get_required<int>("pwm_pin", GPIO_PIN)
        };
        n.ensure(m.direction_pin != m.pwm_pin, "pwm_pin", "must be different from direction_pin");
        return m;
    }

    SafetyThresholds parse_safety_thresholds(const ConfigNode &n) {
        const SafetyThresholds t = {
            .hard = n.get_required<double>("hard", positive),
            .margin = n.get_required<double>("margin", positive)
        };
        n.ensure(t.margin > t.hard, "margin", "must be greater than 'hard'");
        return t;
    }

    PwmOffsets parse_pwm_offsets(const ConfigNode &n) {
        const PwmOffsets o = {
            .low = n.get_required<int>("low", InRange<int>{0, 255}),
            .high = n.get_required<int>("high")
        };
        n.ensure(o.high >= o.low, "high", "must be greater than or equal to 'low'");
        return o;
    }

    // HardwareSpecs parse_hardware_specs(const ConfigNode &n) {
    //     const HardwareSpecs h = {
    //         .timing_belt_module = n.get_required<double>("timing_belt_module", positive),
    //         .pulley_teeth_count = n.get_required<int>("pulley_teeth_count", positive)
    //     };
    //     return h;
    // }

    PendulumConfig build_config(const ConfigNode &root) {
        PendulumConfig c{};
        c.period_ms = root.get_or<int>("period_ms", 20, positive);
        c.calibration_pwm = root.get_required<int>("calibration_pwm", positive);
        c.cutoff_frequency = root.get_required<double>("cutoff_frequency", positive);
        c.right_switch = parse_switch(root.child("right_switch"));
        c.left_switch = parse_switch(root.child("left_switch"));
        c.motor = parse_motor(root.child("motor"));
        // c.hardware_specs = parse_hardware_specs(root.child("hardware_specs"));
        c.position_encoder = parse_position_encoder(root.child("position_encoder"));
        c.angle_encoder = parse_angle_encoder(root.child("angle_encoder"));
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
