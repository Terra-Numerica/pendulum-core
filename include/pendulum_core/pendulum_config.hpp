#pragma once
#include <array>
#include <string>
#include <string_view>

#include "sockets_config.hpp"


// Format of the pendulum configuration file.
// To handle another file, copy this pair of files and adapt the structures and
// the parsers: the generic machinery lives in the yamlcfg library (<yamlcfg/config_node.hpp>, <yamlcfg/constraints.hpp>).

struct SwitchConfig {
    int pin;
    bool normally_up;
    bool use_internal_pull_resistor;
    int debounce_us;
};

constexpr auto fields(const SwitchConfig *) {
    return std::make_tuple(
        YAMLCFG_FIELD(SwitchConfig, pin),
        YAMLCFG_FIELD(SwitchConfig, normally_up),
        YAMLCFG_FIELD(SwitchConfig, use_internal_pull_resistor),
        YAMLCFG_FIELD(SwitchConfig, debounce_us)
    );
}

struct EncoderConfig {
    int pin_a;
    int pin_b;
    int steps_per_turn;
    int glitch_filter_us;
};

struct AngleEncoderConfig : EncoderConfig {
    // nothing to be added
};

constexpr auto fields(const AngleEncoderConfig *) {
    return std::make_tuple(
        YAMLCFG_FIELD(EncoderConfig, pin_a),
        YAMLCFG_FIELD(EncoderConfig, pin_b),
        YAMLCFG_FIELD(EncoderConfig, steps_per_turn),
        YAMLCFG_FIELD(EncoderConfig, glitch_filter_us)
    );
}

struct PositionEncoderConfig : EncoderConfig {
    double belt_module_mm;
    int pulley_teeth_count;
};

constexpr auto fields(const PositionEncoderConfig *) {
    return std::make_tuple(
        YAMLCFG_FIELD(EncoderConfig, pin_a),
        YAMLCFG_FIELD(EncoderConfig, pin_b),
        YAMLCFG_FIELD(EncoderConfig, steps_per_turn),
        YAMLCFG_FIELD(EncoderConfig, glitch_filter_us),
        YAMLCFG_FIELD(PositionEncoderConfig, belt_module_mm),
        YAMLCFG_FIELD(PositionEncoderConfig, pulley_teeth_count)
    );
}

struct MotorConfig {
    int direction_pin;
    int pwm_pin;
};

constexpr auto fields(const MotorConfig *) {
    return std::make_tuple(
        YAMLCFG_FIELD(MotorConfig, direction_pin),
        YAMLCFG_FIELD(MotorConfig, pwm_pin)
    );
}

struct SafetyThresholds {
    double hard;
    double margin;
};

constexpr auto fields(const SafetyThresholds *) {
    return std::make_tuple(
        YAMLCFG_FIELD(SafetyThresholds, hard),
        YAMLCFG_FIELD(SafetyThresholds, margin)
    );
}

struct PwmOffsets {
    int low;
    int high;
};

constexpr auto fields(const PwmOffsets *) {
    return std::make_tuple(
        YAMLCFG_FIELD(PwmOffsets, low),
        YAMLCFG_FIELD(PwmOffsets, high)
    );
}

// struct HardwareSpecs {
//     double timing_belt_module{};
//     int pulley_teeth_count{};
// };

// constexpr auto fields(const HardwareSpecs *) {
//     return std::make_tuple(
//         YAMLCFG_FIELD(HardwareSpecs, timing_belt_module),
//         YAMLCFG_FIELD(HardwareSpecs, pulley_teeth_count)
//     );
// }

struct PendulumConfig {
    int period_ms{};
    int calibration_pwm{};
    double cutoff_frequency{};
    SwitchConfig right_switch{};
    SwitchConfig left_switch{};
    MotorConfig motor{};
    // HardwareSpecs hardware_specs{};
    PositionEncoderConfig position_encoder{};
    AngleEncoderConfig angle_encoder{};
    SafetyThresholds safety_thresholds{};
    PwmOffsets pwm_offsets{};
    SocketsConfig sockets{};
};

constexpr auto fields(const PendulumConfig *) {
    return std::make_tuple(
        YAMLCFG_FIELD(PendulumConfig, period_ms),
        YAMLCFG_FIELD(PendulumConfig, calibration_pwm),
        YAMLCFG_FIELD(PendulumConfig, cutoff_frequency),
        YAMLCFG_FIELD(PendulumConfig, right_switch),
        YAMLCFG_FIELD(PendulumConfig, left_switch),
        YAMLCFG_FIELD(PendulumConfig, motor),
        // YAMLCFG_FIELD(PendulumConfig, hardware_specs),
        YAMLCFG_FIELD(PendulumConfig, position_encoder),
        YAMLCFG_FIELD(PendulumConfig, angle_encoder),
        YAMLCFG_FIELD(PendulumConfig, safety_thresholds),
        YAMLCFG_FIELD(PendulumConfig, pwm_offsets),
        YAMLCFG_FIELD(PendulumConfig, sockets)
    );
}

// Throws yamlcfg::InvalidConfiguration (or a derived class) on any problem.
PendulumConfig load_pendulum_config(const std::string &path);

PendulumConfig parse_pendulum_config(const std::string &yaml_text);

std::string get_pendulum_config_example();

constexpr int GPIOS_COUNT = 8;

// A GPIO used by the process, with a human readable description of its role
struct GPIOPin {
    std::string_view label;
    int pin;
};

struct GPIOMapping {
    GPIOPin left_switch;
    GPIOPin right_switch;
    GPIOPin position_encoder_A;
    GPIOPin position_encoder_B;
    GPIOPin angle_encoder_A;
    GPIOPin angle_encoder_B;
    GPIOPin motor_direction;
    GPIOPin motor_pwm;

    // Return a lightweight array copy for generic iteration
    [[nodiscard]] constexpr std::array<GPIOPin, GPIOS_COUNT> all() const {
        return {
            left_switch, right_switch,
            position_encoder_A, position_encoder_B,
            angle_encoder_A, angle_encoder_B,
            motor_direction, motor_pwm
        };
    }
};

// Detects a field added to GPIOMapping and forgotten in all()
static_assert(sizeof(GPIOMapping) == GPIOS_COUNT * sizeof(GPIOPin), "GPIOMapping and all() are out of sync");

GPIOMapping get_GPIO_mapping(const PendulumConfig &config); // NOLINT
