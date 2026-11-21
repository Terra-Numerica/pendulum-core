#pragma once

#include <yamlcfg/fields.hpp>
#include <string>
#include "sockets_config.hpp"

struct Joystick {
    int axis_pwm{};
    bool uses_dpad{};
    double dpad_smoothing_tau_secs{};
    int btn_switch{};
    int btn_autopilot{};
    int btn_exit{};
    std::string device;
};

constexpr auto fields(const Joystick *) {
    return std::make_tuple(
        YAMLCFG_FIELD(Joystick, axis_pwm),
        YAMLCFG_FIELD(Joystick, uses_dpad),
        YAMLCFG_FIELD(Joystick, dpad_smoothing_tau_secs),
        YAMLCFG_FIELD(Joystick, btn_switch),
        YAMLCFG_FIELD(Joystick, btn_autopilot),
        YAMLCFG_FIELD(Joystick, btn_exit),
        YAMLCFG_FIELD(Joystick, device)
    );
}


struct LQRGains {
    double position{};
    double linear_velocity{};
    double angle{};
    double angular_velocity{};
};

constexpr auto fields(const LQRGains *) {
    return std::make_tuple(
        YAMLCFG_FIELD(LQRGains, position),
        YAMLCFG_FIELD(LQRGains, linear_velocity),
        YAMLCFG_FIELD(LQRGains, angle),
        YAMLCFG_FIELD(LQRGains, angular_velocity)
    );
}

struct SwingUpGains {
    double k{70.0};
    double kx{0.0};
};

constexpr auto fields(const SwingUpGains *) {
    return std::make_tuple(
        YAMLCFG_FIELD(SwingUpGains, k),
        YAMLCFG_FIELD(SwingUpGains, kx)
    );
}

struct Misc {
    double lqr_e_theta_max{};
    double lqr_to_swing_up_hysteresis{};
    double swing_up_to_lqr_ramp_secs{};
    int period_ms{};
};

constexpr auto fields(const Misc *) {
    return std::make_tuple(
        YAMLCFG_FIELD(Misc, lqr_e_theta_max),
        YAMLCFG_FIELD(Misc, lqr_to_swing_up_hysteresis),
        YAMLCFG_FIELD(Misc, swing_up_to_lqr_ramp_secs),
        YAMLCFG_FIELD(Misc, period_ms)

    );
}


struct DemoConfig {
    Joystick joystick{};
    SocketsConfig sockets{};
    LQRGains lqr_gains{};
    SwingUpGains swing_up_gains{};
    Misc misc{};
};

constexpr auto fields(const DemoConfig *) {
    return std::make_tuple(
        YAMLCFG_FIELD(DemoConfig, joystick),
        YAMLCFG_FIELD(DemoConfig, sockets),
        YAMLCFG_FIELD(DemoConfig, lqr_gains),
        YAMLCFG_FIELD(DemoConfig, swing_up_gains),
        YAMLCFG_FIELD(DemoConfig, misc)
    );
}

// Throws yamlcfg::InvalidConfiguration (or a derived class) on any problem.
DemoConfig load_demo_config(const std::string &path);

DemoConfig parse_demo_config(const std::string &yaml_text);

std::string get_demo_config_example();
