// clang-format off
#include <iomanip>
#include <iostream>
#include <csignal>
#include <yaml-cpp/yaml.h>
#include "utils.hpp"
#include <pendule_pi/colors.h>
#include <pendule_pi/joystick.hpp>
#include <pendule_pi/pendule_cpp_client.hpp>
// clang-format on

static bool ok{true};

static void sigintHandler(int signum) { ok = false; }

enum class ControlMode { Manual, LQR, Swingup };

static std::string to_string(const ControlMode &mode) {
    if (mode == ControlMode::Manual) {
        return " manual";
    } else if (mode == ControlMode::LQR) {
        return "  lqr  ";
    } else if (mode == ControlMode::Swingup) {
        return "swingup";
    }
    return "ERROR";
}

static ControlMode next(const ControlMode &mode) {
    switch (mode) {
        case ControlMode::Manual:
            return ControlMode::LQR;
        case ControlMode::LQR:
            return ControlMode::Swingup;
        default:
            return ControlMode::Manual;
    }
}

namespace {
    class CachedButton {
    public:
        void update(const int state) {
            past_ = current_;
            current_ = state;
        }

        [[nodiscard]] bool down() const { return current_; }
        [[nodiscard]] bool switched() const { return current_ != past_; }
        [[nodiscard]] bool pressed() const { return current_ && !past_; }

    private:
        bool current_{false};
        bool past_{false};
    };
}

static void display_help(const std::string &program_name) {
    std::cout << "Usage: " << program_name << " [OPTION] CONFIG_FILE_PATH" << std::endl
            << std::endl
            << std::setw(20) << std::left << "  --example"
            << "shows an example of the expected configuration file content" << std::endl
            << std::setw(20) << std::left << "  -h, --help" << "displays this help" << std::endl;
}

static void display_configuration_example(const std::string &program_name) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "sockets";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "host";
    out << YAML::Value << "localhost";
    out << YAML::Key << "state_port";
    out << YAML::Value << "10001";
    out << YAML::Key << "command_port";
    out << YAML::Value << "10002";
    out << YAML::EndMap;
    out << YAML::Key << "gains";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "position";
    out << YAML::Value << -127.45880662905581;
    out << YAML::Key << "linvel";
    out << YAML::Value << -822.638944546691;
    out << YAML::Key << "angle";
    out << YAML::Value << 2234.654627319883;
    out << YAML::Key << "angvel";
    out << YAML::Value << 437.1177135919267;
    out << YAML::EndMap;
    out << YAML::Key << "swingup";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "k";
    out << YAML::Value << 70;
    out << YAML::Key << "kx";
    out << YAML::Value << 0;
    out << YAML::EndMap;
    out << YAML::Key << "joystick";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "device";
    out << YAML::Value << "/dev/input/js0";
    out << YAML::Key << "axis_pwm";
    out << YAML::Value << 0;
    out << YAML::Key << "btn_exit";
    out << YAML::Value << 0;
    out << YAML::Key << "btn_switch";
    out << YAML::Value << 1;
    out << YAML::Key << "btn_autopilot";
    out << YAML::Value << 4;
    out << YAML::EndMap;
    out << YAML::EndMap;
    std::cout << "Usage: " << program_name << " config.yaml" << std::endl;
    std::cout << "Example of configuration file to be given:" << std::endl;
    std::cout << out.c_str() << std::endl;
}

static void display_status_line(
    const ControlMode control_mode,
    const bool autopilot_engaged,
    const bool in_lqr_range,
    const double e_theta
) {
    std::cout << "\r[ Mode: " << CYAN << std::setw(7) << std::left << to_string(control_mode) << RESET
            << " ] [ LQR range: ";

    std::string msg;
    std::string color;

    const double e_theta_deg = e_theta * 180.0 / M_PI;

    if (in_lqr_range) {
        msg = "in";
        color = GREEN;
    } else {
        msg = "out";
        color = RED;
    }
    std::cout << color
            << std::setw(3) << std::left << msg << RESET
            << " | err=" << CYAN
            << std::setw(6) << std::right << std::setprecision(1) << e_theta_deg
            << RESET;

    std::cout << " ] [ AP: " << std::setw(3) << std::left;

    if (control_mode != ControlMode::Manual) {
        if (autopilot_engaged) {
            msg = "on";
            color = GREEN;
        } else {
            msg = "off";
            color = RED;
        }
    } else {
        msg = "---";
        color = RESET;
    }
    std::cout << color << std::setw(3) << msg << RESET << " ] " << std::flush;
}

int main(const int argc, char **argv) {
    namespace pp = pendule_pi;
    const std::string program_name = argv[0];

    if (argc == 1) {
        std::cerr << program_name << ": missing configuration file path argument." << std::endl
                << "Try '" << program_name << " --help' for more information." << std::endl;
        return EXIT_FAILURE;
    }

    // from now on, we know we have at least one CLI argument, so we can check for help or example requests

    if (argv[1] == std::string("-h") || argv[1] == std::string("--help")) {
        display_help(program_name);
        return EXIT_SUCCESS;
    }

    if (argv[1] == std::string("--example")) {
        display_configuration_example(program_name);
        return EXIT_SUCCESS;
    }

    // Load the configuration which path is passed as a command argument
    YAML::Node yaml;
    yaml = YAML::LoadFile(argv[1]);

    // Connect to the low-level interface.
    pp::PenduleCppClient pendulum(
        yaml["socket"]["host"].as<std::string>("localhost"),
        yaml["socket"]["state_port"].as<std::string>("10001"),
        yaml["socket"]["command_port"].as<std::string>("10002"),
        5
    );

    // Create the joystick device to be used for controlling the demo.
    const auto js_device = yaml["joystick"]["device"].as<std::string>("/dev/input/js0");
    pp::Joystick joy(js_device);

    // Joystick buttons.
    const int AXIS_PWM = yaml["joystick"]["axis_pwm"].as<int>(0);
    const int BTN_EXIT = yaml["joystick"]["btn_exit"].as<int>(0);
    const int BTN_SWITCH = yaml["joystick"]["btn_switch"].as<int>(1);
    const int BTN_AUTOPILOT = yaml["joystick"]["btn_autopilot"].as<int>(4);

    // Counters used to update the joystick at a slightly lower rate.
    unsigned int joy_update_counter = 0;

    // Mode switching button.
    CachedButton btn_mode_switch;

    // Autopilot button
    CachedButton btn_autopilot;

    // Variables used to perform control.
    int pwm = 0;
    auto control_mode(ControlMode::Manual);
    double target_pos = 0;

    std::cout << BRIGHT_WHITE
            << " ----------------------------------------------- " << std::endl
            << "|                                               |" << std::endl
            << "|         Pendulum demo starting...             |" << std::endl
            << "|                                               |" << std::endl
            << " ----------------------------------------------- " << std::endl
            << std::endl << RESET;

    std::cout << BRIGHT_CYAN << "--- Configuration ---" << RESET << std::endl << std::endl;

    // control gains for the swing-up phase
    const auto kswing = yaml["swingup"]["k"].as<double>(70.0);
    const auto ksx = yaml["swingup"]["kx"].as<double>(0.0);

    std::cout << BRIGHT_WHITE << "Swing-up gains:" << RESET << std::endl
            << BLUE << "  kswing : " << RESET
            << std::fixed << std::setw(12) << std::setprecision(6) << kswing << std::endl
            << BLUE << "  ksx    : " << RESET
            << std::fixed << std::setw(12) << std::setprecision(6) << ksx << std::endl
            << std::endl;

    bool autopilot_engaged = false;
    bool in_lqr_range = false;
    bool was_in_lqr_range = false;
    bool must_refresh_status = true;

    // control gains for the LQR phase
    const auto kp = yaml["gains"]["position"].as<double>(-127.45880662905581);
    const auto kpd = yaml["gains"]["linvel"].as<double>(-822.638944546691);
    const auto kt = yaml["gains"]["angle"].as<double>(2234.654627319883);
    const auto ktd = yaml["gains"]["angvel"].as<double>(437.1177135919267);

    std::cout << BRIGHT_WHITE << "LQR gains:" << RESET << std::endl
            << BLUE << "  kp     : " << RESET << std::fixed << std::setw(12) << std::setprecision(6) << kp << std::endl
            << BLUE << "  kpd    : " << RESET << std::fixed << std::setw(12) << std::setprecision(6) << kpd << std::endl
            << BLUE << "  kt     : " << RESET << std::fixed << std::setw(12) << std::setprecision(6) << kt << std::endl
            << BLUE << "  ktd    : " << RESET << std::fixed << std::setw(12) << std::setprecision(6) << ktd << std::endl
            << std::endl;

    std::cout << BRIGHT_WHITE << "Control:" << RESET << std::endl
            << BLUE << "  Joystick device            : " << RESET << js_device << std::endl
            << BLUE << "  Axis for carriage control  : " << RESET << AXIS_PWM << std::endl
            << BLUE << "  Buttons : " << RESET << std::endl
            << BLUE << "   - switch control mode     : " << RESET << BTN_SWITCH << std::endl
            << BLUE << "   - toggle autopilot        : " << RESET << BTN_AUTOPILOT << std::endl
            << BLUE << "   - exit                    : " << RESET << ((BTN_EXIT != -1)
                                                                          ? std::to_string(BTN_EXIT)
                                                                          : "-- disabled --")
            << std::endl << std::endl;

    std::cout << BRIGHT_GREEN << "Demo started " << RESET << "(use Ctrl-C ";

    if (BTN_EXIT != -1) {
        std::cout << "or the EXIT button of the gamepad ";
    }
    std::cout << "to exit)" << std::endl << std::endl;

    // Custom signal handler.
    std::signal(SIGINT, sigintHandler);

    // Periodic status refresh when no event triggers it
    const auto status_update_period = std::chrono::milliseconds(500);
    auto next_status_update = std::chrono::steady_clock::now() + status_update_period;

    double last_e_theta = 2 * M_PI; // out of range value to force the first update

    // Main loop!
    while (ok) {
        unsigned int JOY_UPDATE_MAX = 5;
        constexpr double target_angle = M_PI;
        constexpr double ANGLE_ERROR_THRESHOLD = 0.25;

        auto now = std::chrono::steady_clock::now();

        // Retrieve the current state of the pendulum.
        pendulum.readState(pendule_pi::PenduleCppClient::BLOCKING);

        // compute angle error whatever is the current mode so that the status line can be used for visual control
        double e_theta = shiftInCircle(pendulum.angle() - target_angle);

        in_lqr_range = std::fabs(e_theta) <= ANGLE_ERROR_THRESHOLD;

        // detect changes to refresh the status line only when needed
        if (in_lqr_range != was_in_lqr_range) {
            must_refresh_status = true;
            was_in_lqr_range = in_lqr_range;
        }

        // Do control!
        if (control_mode == ControlMode::Manual) {
            // Get the command from the joystick.
            int ref = joy.axis(AXIS_PWM);
            ref = robustZero(ref, 2500);
            pwm = static_cast<int>(ref * 250. / 32768.);
        } else if (control_mode == ControlMode::LQR || control_mode == ControlMode::Swingup) {
            // Errors (into [-pi,+pi] for the angle)
            target_pos = clip(joy.axis(AXIS_PWM), 2500) * 0.2 / 32768.;
            double e_pos = pendulum.position() - target_pos;
            double e_linear_velocity = pendulum.linvel();
            double e_angular_velocity = pendulum.angvel();

            if (in_lqr_range) {
                // in LQR range: do it!
                pwm = static_cast<int>(
                    -kp * e_pos
                    - kpd * e_linear_velocity
                    - kt * e_theta
                    - ktd * e_angular_velocity
                );
            } else {
                // Not in LQR range: the command depends on the current mode.
                if (control_mode == ControlMode::LQR) {
                    pwm = 0;
                } else {
                    // Swing-up
                    // TODO : make the formula less opaque by moving these magic numbers to explicitly named constants
                    auto ks = kswing * (0.1 + 0.9 * 0.5 * (1 - std::cos(e_theta)));
                    pwm = static_cast<int>(
                        +ks * (1 - std::cos(e_theta)) * sign(pendulum.angvel() * std::cos(e_theta)) -
                        ksx * pendulum.position()
                    );
                }
            }

            // discard the computed command if the autopilot is not engaged
            btn_autopilot.update(joy.button(BTN_AUTOPILOT));
            if (btn_autopilot.switched()) {
                autopilot_engaged = btn_autopilot.down();
                must_refresh_status = true;
            }

            if (!autopilot_engaged) {
                pwm = 0;
            }
        } else {
            std::cerr << std::endl << "ERROR: " << to_string(control_mode) << " mode not implemented yet";
            break;
        }

        pendulum.sendCommand(pwm);

        // update the joystick when needed
        joy_update_counter = (joy_update_counter + 1) % JOY_UPDATE_MAX;
        if (joy_update_counter == 0) {
            joy.update();

            // should we exit?
            if (BTN_EXIT != -1 && joy.button(BTN_EXIT)) {
                std::cout << std::endl << "Quitting!";
                break;
            }

            // should we switch control mode?
            btn_mode_switch.update(joy.button(BTN_SWITCH));
            if (btn_mode_switch.pressed()) {
                control_mode = next(control_mode);
                must_refresh_status = true;
            }
        }

        // ensure the angle error display is updated periodically if no specific event triggered it
        if (!must_refresh_status && now >= next_status_update) {
            next_status_update += status_update_period;
            if (e_theta != last_e_theta) {
                must_refresh_status = true;
                last_e_theta = e_theta;
            }
        }

        if (must_refresh_status) {
            display_status_line(control_mode, autopilot_engaged, in_lqr_range, e_theta);
            must_refresh_status = false;
        }
    } // end of the main loop

    pendulum.sendCommand(0);
    std::cout << std::endl;

    return EXIT_SUCCESS;
}
