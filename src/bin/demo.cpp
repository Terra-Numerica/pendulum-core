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

enum class ControlMode { Manual, LQR, Swingup };

static std::string to_string(const ControlMode &mode) {
    switch (mode) {
        case ControlMode::Manual:
            return "manual ";
        case ControlMode::LQR:
            return "  lqr  ";
        case ControlMode::Swingup:
            return "swingup";
        default:
            return " ERROR ";
    }
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

#define LABEL BRIGHT_BLUE

static void display_status_line(
    const ControlMode control_mode,
    const bool autopilot_engaged,
    const bool in_lqr_range,
    const double e_theta,
    const double e_position
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
            << " ] [ Err: angle=" << CYAN
            << std::setw(6) << std::right << std::setprecision(1) << e_theta_deg << RESET
            << " | pos=" << CYAN;

    if (control_mode == ControlMode::Manual) {
        std::cout << " --- ";
    } else {
        std::cout << std::setw(5) << std::right << std::setprecision(2) << e_position;
    }

    std::cout << RESET;

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

// main forever loop termination flag
static bool keep_running{true};

static void requestTermination([[maybe_unused]] int signum) { keep_running = false; }


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
    YAML::Node configuration;

    try {
        configuration = YAML::LoadFile(argv[1]);
    } catch (const YAML::Exception &e) {
        std::cerr << "ERROR: YAML configuration loading error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch (const std::exception &e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Connect to the low-level interface.
    const auto socket_config = configuration["socket"];
    pp::PenduleCppClient pendulum(
        socket_config["host"].as<std::string>("localhost"),
        socket_config["state_port"].as<std::string>("10001"),
        socket_config["command_port"].as<std::string>("10002"),
        5
    );

    // Create the joystick device to be used for controlling the demo.
    const auto joystick_config = configuration["joystick"];

    const auto js_device = joystick_config["device"].as<std::string>("/dev/input/js0");
    pp::Joystick joy(js_device);

    // Joystick buttons.
    const int AXIS_PWM = joystick_config["axis_pwm"].as<int>(0);
    const int BTN_EXIT = joystick_config["btn_exit"].as<int>(0);
    const int BTN_SWITCH = joystick_config["btn_switch"].as<int>(1);
    const int BTN_AUTOPILOT = joystick_config["btn_autopilot"].as<int>(4);

    // The joystick is not polled on every iteration of the control loop.
    unsigned int JOY_UPDATE_PACE = 5;
    unsigned int iterations_to_joystick_update = JOY_UPDATE_PACE;

    // Mode switching button.
    CachedButton btn_mode_switch;

    // Autopilot button
    CachedButton btn_autopilot;

    // Variables used to perform control.
    int pwm = 0;
    auto control_mode(ControlMode::Manual);
    double target_pos = 0;

    std::cout << BRIGHT_WHITE
            << " ------------------------------------------------ " << std::endl
            << "|                                                |" << std::endl
            << "|          Pendulum demo starting...             |" << std::endl
            << "|                                                |" << std::endl
            << " ------------------------------------------------ " << std::endl
            << std::endl << LABEL << "Version " << PENDULE_PI_VERSION << std::endl
            << std::endl << RESET;

    std::cout << BRIGHT_CYAN << "--- Configuration ---" << RESET << std::endl << std::endl;

    // control gains for the swing-up phase
    const auto swingup_config = configuration["swingup"];
    const auto kswing = swingup_config["k"].as<double>(70.0);
    const auto ksx = swingup_config["kx"].as<double>(0.0);

    std::cout << BRIGHT_WHITE << "Swing-up gains:" << RESET << std::endl
            << LABEL << "  kswing : " << RESET
            << std::fixed << std::setw(12) << std::setprecision(6) << kswing << std::endl
            << LABEL << "  ksx    : " << RESET
            << std::fixed << std::setw(12) << std::setprecision(6) << ksx << std::endl
            << std::endl;

    bool autopilot_engaged = false;
    bool in_lqr_range = false;
    bool was_in_lqr_range = false;
    bool refresh_status_requested = true;

    // control gains for the LQR phase
    const auto gains_config = configuration["gains"];
    const auto kp = gains_config["position"].as<double>(-127.45880662905581);
    const auto kpd = gains_config["linvel"].as<double>(-822.638944546691);
    const auto kt = gains_config["angle"].as<double>(2234.654627319883);
    const auto ktd = gains_config["angvel"].as<double>(437.1177135919267);

    std::cout << BRIGHT_WHITE << "LQR gains:" << RESET << std::endl
            << LABEL << "  kp     : " << RESET << std::fixed << std::setw(12) << std::setprecision(6) << kp << std::endl
            << LABEL << "  kpd    : " << RESET << std::fixed << std::setw(12) << std::setprecision(6) << kpd <<
            std::endl
            << LABEL << "  kt     : " << RESET << std::fixed << std::setw(12) << std::setprecision(6) << kt << std::endl
            << LABEL << "  ktd    : " << RESET << std::fixed << std::setw(12) << std::setprecision(6) << ktd <<
            std::endl
            << std::endl;

    std::cout << BRIGHT_WHITE << "Control:" << RESET << std::endl
            << LABEL << "  Joystick device            : " << RESET << js_device << std::endl
            << LABEL << "  Axis for carriage control  : " << RESET << AXIS_PWM << std::endl
            << LABEL << "  Buttons : " << RESET << std::endl
            << LABEL << "   - switch control mode     : " << RESET << BTN_SWITCH << std::endl
            << LABEL << "   - toggle autopilot        : " << RESET << BTN_AUTOPILOT << std::endl
            << LABEL << "   - exit                    : " << RESET << (BTN_EXIT != -1
                                                                           ? std::to_string(BTN_EXIT)
                                                                           : "-- disabled --")
            << std::endl << std::endl;

    std::cout << BRIGHT_GREEN << "Demo started " << RESET << "(use Ctrl-C ";

    if (BTN_EXIT != -1) {
        std::cout << "or the EXIT button of the gamepad ";
    }
    std::cout << "to exit)" << std::endl << std::endl;

    // Connect signals for graceful termination.
    std::signal(SIGINT, requestTermination);
    std::signal(SIGTERM, requestTermination);

    // Periodic status refresh when no event triggers it
    const auto status_update_period = std::chrono::milliseconds(500);
    auto next_status_update = std::chrono::steady_clock::now() + status_update_period;

    double last_e_theta = 2 * M_PI; // out of range value to force the first update

    // Main loop!
    while (keep_running) {
        constexpr double target_angle = M_PI;
        constexpr double ANGLE_ERROR_THRESHOLD = 0.25;

        auto now = std::chrono::steady_clock::now();

        // Retrieve the current state of the pendulum.
        pendulum.readState(pendule_pi::PenduleCppClient::BLOCKING);

        // compute errors whatever is the current mode so that the status line can be used for visual control
        double e_theta = shiftInCircle(pendulum.angle() - target_angle);
        double e_pos = 0;

        in_lqr_range = std::fabs(e_theta) <= ANGLE_ERROR_THRESHOLD;

        // detect changes to refresh the status line only when needed
        if (in_lqr_range != was_in_lqr_range) {
            refresh_status_requested = true;
            was_in_lqr_range = in_lqr_range;
        }

        constexpr unsigned int JOYSTICK_DEAD_ZONE = 2500;

        // Compute the PWM to be applied to the motor, depending on the current mode
        switch (control_mode) {
            case ControlMode::Manual: {
                // ensure autopilot is disengaged so that we'll start with a know state
                // when switching mode eventually
                autopilot_engaged = false;

                // in manual mode, the PWM is directly derived from the gamepad joystick position
                constexpr double PWM_LIMIT = 250.;

                int ref = joy.axis(AXIS_PWM);
                ref = robustZero(ref, JOYSTICK_DEAD_ZONE);
                pwm = static_cast<int>(ref * PWM_LIMIT / 32768.);
                break;
            }

            case ControlMode::LQR:
            case ControlMode::Swingup: {
                // check first if the autopilot button is pressed so that we don't make unneeded hard work
                btn_autopilot.update(joy.button(BTN_AUTOPILOT));
                if (btn_autopilot.switched()) {
                    autopilot_engaged = btn_autopilot.down();
                    refresh_status_requested = true;
                }

                if (autopilot_engaged) {
                    // the user asks us to take control => let's do the heavy lifting

                    // in LQR or SWINGUP modes, let the system compute the PWM by applying the control law
                    // related to the mode and present situation of the pendulum( in LQR zone or not)

                    double e_linear_velocity = pendulum.linear_velocity();
                    double e_angular_velocity = pendulum.angular_velocity();

                    // let the user shift the cart target position by using the gamepad
                    constexpr double POSITION_SHIFT_LIMIT = 0.2;
                    target_pos = deadZoneFilter(joy.axis(AXIS_PWM), JOYSTICK_DEAD_ZONE) * POSITION_SHIFT_LIMIT / 32768.;
                    e_pos = pendulum.position() - target_pos;

                    if (in_lqr_range) {
                        // apply the LQR control law when inside the proper range

                        pwm = static_cast<int>(
                            -kp * e_pos
                            - kpd * e_linear_velocity
                            - kt * e_theta
                            - ktd * e_angular_velocity
                        );
                    } else {
                        // Not in LQR range: the command depends on the current mode.
                        if (control_mode == ControlMode::LQR) {
                            // LQR mode => just do nothing
                            pwm = 0;
                        } else {
                            // Swingup mode => move the cart so that the pendulum gradually comes close to the LQR range

                            // TODO : make the formula less opaque by moving these magic numbers to explicitly named constants
                            auto ks = kswing * (0.1 + 0.9 * 0.5 * (1 - std::cos(e_theta)));
                            pwm = static_cast<int>(
                                +ks * (1 - std::cos(e_theta)) * sign(pendulum.angular_velocity() * std::cos(e_theta)) -
                                ksx * pendulum.position()
                            );
                        }
                    }
                } else {
                    // autopilot is off => don't move
                    pwm = 0;
                }
                break;
            }
                // no need for default case, since we have covered all the enumeration values
        }

        pendulum.sendCommand(pwm);

        // take joystick into account only if it's time for it
        if (--iterations_to_joystick_update == 0) {
            joy.update();

            // should we exit?
            if (BTN_EXIT != -1 && joy.button(BTN_EXIT)) {
                std::cout
                        << std::endl << std::endl
                        << BRIGHT_YELLOW << "--- Gamepad Exit button pressed. Quitting now ---" << RESET;
                break;
            }

            // should we switch control mode?
            btn_mode_switch.update(joy.button(BTN_SWITCH));
            if (btn_mode_switch.pressed()) {
                control_mode = next(control_mode);
                refresh_status_requested = true;
            }

            // reset the pace counter
            iterations_to_joystick_update = JOY_UPDATE_PACE;
        }

        // ensure the angle error display is updated periodically if no specific event triggered it
        if (!refresh_status_requested && now >= next_status_update) {
            next_status_update += status_update_period;
            if (e_theta != last_e_theta) {
                refresh_status_requested = true;
                last_e_theta = e_theta;
            }
        }

        if (refresh_status_requested) {
            display_status_line(control_mode, autopilot_engaged, in_lqr_range, e_theta, e_pos);
            // reset status line refresh request
            refresh_status_requested = false;
        }
    } // end of the main loop

    pendulum.sendCommand(0);
    std::cout << RESET << std::endl;

    return EXIT_SUCCESS;
}
