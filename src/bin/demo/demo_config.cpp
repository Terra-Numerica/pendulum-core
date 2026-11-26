#include <yamlcfg/config_node.hpp>
#include <yamlcfg/constraints.hpp>

#include "demo_config.hpp"
#include "pendulum_core/sockets_config.hpp"

namespace {
    using yamlcfg::ConfigNode;
    using yamlcfg::InRange;
    using yamlcfg::not_empty;
    using yamlcfg::non_negative;
    using yamlcfg::not_zero;
    using yamlcfg::positive;

    constexpr InRange<int> BUTTON{0, 6};
    constexpr InRange<int> AXIS{0, 3};

    Joystick parse_joystick(const ConfigNode &n) {
        Joystick j;
        j.device = n.get_or<std::string>("device", "/dev/input/js0");
        j.axis_pwm = n.get_required<int>("axis_pwm", AXIS);
        j.btn_switch = n.get_required<int>("btn_switch", BUTTON);
        j.btn_autopilot = n.get_required<int>("btn_autopilot", BUTTON);
        j.btn_exit = n.get_or<int>("btn_exit", -1, BUTTON);
        j.uses_dpad = n.get_or<bool>("uses_dpad", false);
        j.dpad_smoothing_tau_secs = n.get_or<double>("dpad_smoothing_tau_secs", 0.02);
        n.ensure(j.btn_switch != j.btn_autopilot, "btn_autopilot", "all buttons must be different");
        n.ensure(j.btn_switch != j.btn_exit, "btn_exit", "all buttons must be different");
        n.ensure(j.btn_autopilot != j.btn_exit, "btn_exit", "all buttons must be different");
        n.ensure_distinct<int>({
            {"btn_switch", j.btn_switch},
            {"btn_autopilot", j.btn_autopilot},
            {"btn_exit", j.btn_exit},
        });
        return j;
    }

    LQRGains parse_lqr_gains(const ConfigNode &n) {
        LQRGains g;
        g.position = n.get_required<double>("position", not_zero);
        g.angle = n.get_required<double>("angle", not_zero);
        g.linear_velocity = n.get_required<double>("linear_velocity", not_zero);
        g.angular_velocity = n.get_required<double>("angular_velocity", not_zero);
        return g;
    }

    SwingUpGains parse_swing_up_gains(const ConfigNode &n) {
        SwingUpGains g;
        g.k = n.get_required<double>("k", not_zero);
        g.kx = n.get_required<double>("kx");
        return g;
    }

    Misc parse_misc(const ConfigNode &n) {
        Misc m;
        m.lqr_e_theta_max = n.get_or<double>("lqr_e_theta_max", 0.25, positive);
        m.lqr_blend_margin_angle = n.get_or<double>("lqr_blend_margin_angle", 0.2, non_negative);
        m.lqr_to_swing_up_hysteresis = n.get_or<double>("lqr_to_swing_up_hysteresis", 0.2, positive);
        m.period_ms = n.get_or<int>("period_ms", 20, positive);
        return m;
    }

    DemoConfig build_config(const ConfigNode &root) {
        DemoConfig c;
        c.joystick = parse_joystick(root.child("joystick"));
        c.sockets = parse_sockets(root.child("sockets"));
        c.lqr_gains = parse_lqr_gains(root.child("lqr_gains"));
        c.swing_up_gains = parse_swing_up_gains(root.child("swing_up"));
        c.misc = parse_misc(root.child("misc"));
        return c;
    }
}

DemoConfig load_demo_config(const std::string &path) {
    return yamlcfg::load_file(path, build_config);
}

DemoConfig parse_demo_config(const std::string &yaml_text) {
    return yamlcfg::parse_text(yaml_text, build_config);
}

