#include <yamlcfg/config_node.hpp>
#include <yamlcfg/constraints.hpp>

#include "demo_config.hpp"
#include "pendulum_core/sockets_config.hpp"

namespace {
    using yamlcfg::ConfigNode;
    using yamlcfg::InRange;
    using yamlcfg::not_empty;
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

std::string get_demo_config_example() {
    return "joystick:\n"
            "  # the axis used to control the carriage in manual mode\n"
            "  axis_pwm: 0\n"
            "  # mode switch (manual -> LQR -> swing up)\n"
            "  btn_switch: 0\n"
            "  # gives control to app in LQR and swing up modes\n"
            "  btn_autopilot: 2\n"
            "  # comment the next line to disable the Exit button\n"
            "  btn_exit: 1\n"
            "  # edit and uncomment next line if you have several gamepads connected\n"
            "  # device: /dev/input/js0\n"
            "\n"
            "# communications between demo and hw_interface\n"
            "sockets:\n"
            "  host: localhost\n"
            "  state_port: 10001\n"
            "  command_port: 10002\n"
            "\n"
            "# coefficients of the LQR control law\n"
            "lqr_gains:\n"
            "  position: -127.45880662905581\n"
            "  linear_vel: -822.638944546691\n"
            "  angle: 2234.654627319883\n"
            "  angular_velocity: 437.1177135919267\n"
            "\n"
            "# coefficients of the swing-up control law\n"
            "swing_up:\n"
            "  k: 70\n"
            "  kx: 0\n"
            "\n"
            "misc:\n"
            "  # max angle error of the LQR range (> 0, default: 0.25)\n"
            "  lqr_e_theta_max: 0.25   # rd\n"
            "  # ratio of max LQR error to switch from swing up to LQR (in [0.0, 1.0], default: 1)\n"
            "  swingup_to_lqr_ratio: 0.9\n"
            "  period_ms: 20\n";
}
