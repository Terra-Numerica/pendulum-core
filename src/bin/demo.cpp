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

namespace {
    enum class ControlMode { Manual, LQR, Swingup };
}

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
            << "outputs an configuration file example" << std::endl
            << std::setw(20) << std::left << "  -h, --help" << "displays this help" << std::endl;
}

static void print_configuration_example(const std::string &program_name) {
    std::cout <<
            "joystick:\n"
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
            "# communications between demo and low level controller\n"
            "sockets:\n"
            "  host: localhost\n"
            "  state_port: 10001\n"
            "  command_port: 10002\n"
            "\n"
            "# coefficients of the LQR control law\n"
            "lqr_gains:\n"
            "  position: -127.45880662905581\n"
            "  lin_vel: -822.638944546691\n"
            "  angle: 2234.654627319883\n"
            "  ang_vel: 437.1177135919267\n"
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
            "  swingup_to_lqr_ratio: 0.9\n";
}

#define LABEL BRIGHT_BLUE
#define ERROR BRIGHT_RED
#define CLEAR_LINE "\033[2K"

static void display_status_line(
    const ControlMode control_mode,
    const bool autopilot_engaged,
    const bool in_lqr_range,
    const double e_theta,
    const double e_position
) {
    std::cout << CLEAR_LINE << "\r[ Mode: " << CYAN << std::setw(7) << std::left << to_string(control_mode) << RESET
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
            << " | pos=" << CYAN
            // position error is displayed in cm
            << std::setw(5) << std::right << std::setprecision(1) << e_position * 100
            << RESET << " ] [ AP: " << std::setw(3) << std::left;

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

// Smoothing filter used to replace brutal control changes produced by a D-pad
class LowPassFilter {
private:
    double alpha; // Computed smoothing factor
    double smoothedValue; // Last computed value
    bool isInitialized; // Prevents an artificial startup from 0

public:
    // timeConstantSec (tau): How sluggish the filter is (e.g., 0.5s or 1.0s).
    // samplePeriodSec (dt):  Time between updates in seconds (20ms = 0.02s).
    LowPassFilter(const double timeConstantSec, const double samplePeriodSec)
        : smoothedValue(0.0), isInitialized(false) {
        if (timeConstantSec <= 0.0) {
            alpha = 1.0; // No filtering
        } else {
            alpha = samplePeriodSec / (timeConstantSec + samplePeriodSec);
        }
    }

    double update(double newReading) {
        if (!isInitialized) {
            smoothedValue = newReading;
            isInitialized = true;
        } else {
            smoothedValue = (alpha * newReading) + ((1.0 - alpha) * smoothedValue);
        }
        return smoothedValue;
    }

    void reset() {
        isInitialized = false;
    }
};


// main forever loop termination flag
static volatile std::sig_atomic_t keep_running = 1;

static void requestTermination([[maybe_unused]] int signum) { keep_running = false; }

static double rad_to_deg(const double rad) {
    return rad * 180. / M_PI;
}

static void print_error(const std::string &msg, const std::string &addit_info) {
    std::cerr << ERROR << "ERROR: " << msg << " " << addit_info << RESET << std::endl;
}

static void print_error(const std::string &msg) {
    print_error(msg, "");
}

static void print_config_error(const std::string &addit_info) {
    print_error("bad configuration", addit_info);
}

int main(const int argc, char **argv) {
    // intercept the version request before doing anything else
    if (argc == 2 && strcmp(argv[1], "--version") == 0) {
        std::cout << PENDULE_PI_VERSION << std::endl;
        return EXIT_SUCCESS;
    }

    namespace pp = pendule_pi;
    const std::string program_name = argv[0];

    if (argc == 1) {
        print_error("missing configuration file path argument.");
        std::cout << "Try '" << program_name << " --help' for more information." << std::endl;
        return EXIT_FAILURE;
    }

    // from now on, we know we have at least one CLI argument, so we can check for help or example requests

    if (argv[1] == std::string("-h") || argv[1] == std::string("--help")) {
        display_help(program_name);
        return EXIT_SUCCESS;
    }

    if (argv[1] == std::string("--example")) {
        print_configuration_example(program_name);
        return EXIT_SUCCESS;
    }

    // Load the configuration which path is passed as a command argument
    YAML::Node configuration;

    try {
        configuration = YAML::LoadFile(argv[1]);
    } catch (const YAML::Exception &e) {
        print_error("YAML configuration loading error: ", e.what());
        return EXIT_FAILURE;
    } catch (const std::exception &e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // load and validate configuration parameters

    const auto period_ms = configuration["period_ms"].as<int>(20);
    if (period_ms < 0) {
        print_config_error("period_ms must be positive");
        return EXIT_FAILURE;
    }

    const auto misc_config = configuration["misc"];
    const auto lqr_e_theta_max = std::fabs(misc_config["lqr_e_theta_max"].as<double>(0.25));
    const auto lqr_to_swingup_hysteresis = misc_config["lqr_to_swingup_hysteresis"].as<double>(0.2);
    if (lqr_to_swingup_hysteresis < 0) {
        print_config_error("lqr_to_swingup_hysteresis must be positive");
        return EXIT_FAILURE;
    }
    const auto swingup_to_lqr_ramp_secs = misc_config["swingup_to_lqr_ramp_secs"].as<double>(0.080);
    if (swingup_to_lqr_ramp_secs < 0) {
        print_config_error("swingup_to_lqr_ramp_secs must be positive");
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

    /*
     * Game controller setup
     */

    // Create the joystick device to be used for controlling the demo.
    const auto joystick_config = configuration["joystick"];

    // Joystick buttons.
    const auto AXIS_PWM = joystick_config["axis_pwm"].as<int>(0);
    const auto BTN_EXIT = joystick_config["btn_exit"].as<int>(0);
    const auto BTN_SWITCH = joystick_config["btn_switch"].as<int>(1);
    const auto BTN_AUTOPILOT = joystick_config["btn_autopilot"].as<int>(4);
    const auto USES_DPAD = joystick_config["uses_dpad"].as<bool>(false);
    const auto DPAD_SMOOTHING_TAU = joystick_config["dpad_smoothing_tau_secs"].as<double>(0.1);
    if (DPAD_SMOOTHING_TAU < 0) {
        print_config_error("dpad_smoothing_tau_secs must be positive");
        return EXIT_FAILURE;
    }

    const auto js_device = joystick_config["device"].as<std::string>("/dev/input/js0");
    pp::Joystick joy(js_device);

    // The joystick is not polled on every iteration of the control loop.
    unsigned int JOY_UPDATE_PACE = 5;
    unsigned int iterations_to_joystick_update = JOY_UPDATE_PACE;

    // Mode switching button.
    CachedButton btn_mode_switch;

    // Autopilot button
    CachedButton btn_autopilot;

    // D-pad smoothing
    auto dpad_filter = LowPassFilter(DPAD_SMOOTHING_TAU, period_ms / 1000.);

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

    auto coeff_fmt = [](std::ostream &os) -> std::ostream & {
        return os << RESET << std::fixed << std::setw(12) << std::setprecision(6);
    };

    // control gains for the swing-up phase
    const auto swingup_config = configuration["swing_up"];
    const auto kswing = swingup_config["k"].as<double>(70.0);
    const auto ksx = swingup_config["kx"].as<double>(0.0);

    std::cout << BRIGHT_WHITE << "Swing-up gains:" << RESET << std::endl
            << LABEL << "  kswing : " << coeff_fmt << kswing << std::endl
            << LABEL << "  ksx    : " << coeff_fmt << ksx << std::endl
            << std::endl;

    bool autopilot_engaged = false;
    bool in_lqr_range = false;
    bool was_in_lqr_range = false;
    bool refresh_status_requested = true;

    // control gains for the LQR phase
    const auto gains_config = configuration["lqr_gains"];
    const auto kp = gains_config["position"].as<double>(-127.45880662905581);
    const auto kpd = gains_config["lin_vel"].as<double>(-822.638944546691);
    const auto kt = gains_config["angle"].as<double>(2234.654627319883);
    const auto ktd = gains_config["ang_vel"].as<double>(437.1177135919267);

    std::cout << BRIGHT_WHITE << "LQR gains:" << RESET << std::endl
            << LABEL << "  kp     : " << coeff_fmt << kp << std::endl
            << LABEL << "  kpd    : " << coeff_fmt << kpd << std::endl
            << LABEL << "  kt     : " << coeff_fmt << kt << std::endl
            << LABEL << "  ktd    : " << coeff_fmt << ktd << std::endl
            << std::endl;

    std::cout << BRIGHT_WHITE << "Control:" << RESET << std::endl
            << LABEL << "  Joystick device            : " << RESET << js_device << std::endl
            << LABEL << "  Axis for carriage control  : " << RESET << AXIS_PWM << std::endl
            << LABEL << "  Uses D-pad                 : " << RESET << USES_DPAD << std::endl
            << LABEL << "  D-pad smoothing tau        : " << RESET << std::setprecision(2) << DPAD_SMOOTHING_TAU << " s"
            << std::endl
            << LABEL << "  Buttons : " << RESET << std::endl
            << LABEL << "   - switch control mode     : " << RESET << BTN_SWITCH << std::endl
            << LABEL << "   - toggle autopilot        : " << RESET << BTN_AUTOPILOT << std::endl
            << LABEL << "   - exit                    : " << RESET << (BTN_EXIT != -1
                                                                           ? std::to_string(BTN_EXIT)
                                                                           : "-- disabled --")
            << std::endl << std::endl;

    auto misc_fmt = [](std::ostream &os) -> std::ostream & {
        return os << RESET << std::fixed << std::setw(4) << std::setprecision(2);
    };

    std::cout << BRIGHT_WHITE << "Misc:" << RESET << std::endl
            << LABEL << "  - LQR angle error max     : " << misc_fmt << lqr_e_theta_max
            << " rd  (" << rad_to_deg(lqr_e_theta_max) << "°)" << std::endl
            << LABEL << "  - swingup->LQR hysteresis : " << misc_fmt << lqr_to_swingup_hysteresis
            << " rd  (" << rad_to_deg(lqr_to_swingup_hysteresis) << "°)" << std::endl
            << LABEL << "  - swingup->LQR ramp time  : " << misc_fmt << swingup_to_lqr_ramp_secs << " s" <<
            std::endl
            << std::endl;

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

    const auto lqr_to_swingup_threshold = lqr_e_theta_max + lqr_to_swingup_hysteresis;
    // swing-up / lqr PWMs mix factor for blending the transition
    double alpha = 0;

    bool last_time_is_set = false;
    std::chrono::time_point<std::chrono::steady_clock> last_time;
    // initialize the dt to the theoretical value of the control period
    std::chrono::duration<double> dt_secs(period_ms / 1000.);

    // tells if we are in LQR range while running in swing-up mode
    // (used to add an hysteresis to the LQR/swing-up transition)
    bool stabilizing = false;

    // Main loop!
    while (keep_running) {
        constexpr double target_angle = M_PI;

        auto now = std::chrono::steady_clock::now();

        if (last_time_is_set) {
            // compute the real dt if we are not on the first iteration
            dt_secs = now - last_time;
        }

        // Retrieve the current state of the pendulum.
        pendulum.readState(pendule_pi::PenduleCppClient::BLOCKING);

        // Since the low level sends state messages periodically, as configured by period_ms in YAML file
        // and readState has been called in blocking mode, we are here at regular intervals.

        // compute errors whatever is the current mode so that the status line can be used for visual control
        double e_theta = shiftInCircle(pendulum.angle() - target_angle);
        double e_pos = 0;

        in_lqr_range = std::fabs(e_theta) <= lqr_e_theta_max;

        // detect changes to refresh the status line only when needed
        if (in_lqr_range != was_in_lqr_range) {
            refresh_status_requested = true;
        }

        constexpr unsigned int JOYSTICK_DEAD_ZONE = 2500;

        // Compute the PWM to be applied to the motor, depending on the current mode
        if (control_mode == ControlMode::Manual) {
            // ensure autopilot is disengaged so that we'll start with a know state
            // when switching mode eventually
            autopilot_engaged = false;

            // in manual mode, the PWM is directly derived from the gamepad joystick position
            constexpr double PWM_LIMIT = 250.;

            double ref = robustZero(joy.axis(AXIS_PWM), JOYSTICK_DEAD_ZONE);
            if (USES_DPAD) {
                // smooth the input value to avoid brutal changes of the PWW that can damage
                // the mechanical part and make the PSU security circuit trip.
                ref = dpad_filter.update(ref);
            }
            pwm = static_cast<int>(ref * PWM_LIMIT / 32768.);

            // in manual mode, the center of the track is considered as the target position
            e_pos = pendulum.position();
        } else {
            // either in LQR or in swing-up mode

            // check first if the autopilot button is pressed so that we don't make unneeded hard work
            btn_autopilot.update(joy.button(BTN_AUTOPILOT));
            if (btn_autopilot.switched()) {
                autopilot_engaged = btn_autopilot.down();
                refresh_status_requested = true;
            }

            if (autopilot_engaged) {
                // the user asks us to take control => let's do the heavy lifting
                double e_linear_velocity = pendulum.linear_velocity();
                double e_angular_velocity = pendulum.angular_velocity();

                // let the user shift the cart target position by using the gamepad
                constexpr double POSITION_SHIFT_LIMIT = 0.2;
                target_pos = deadZoneFilter(joy.axis(AXIS_PWM), JOYSTICK_DEAD_ZONE) * POSITION_SHIFT_LIMIT / 32768.;
                e_pos = pendulum.position() - target_pos;

                // compute the LQR control law since we'll use its result whichever is the current mode
                int pwm_lqr = static_cast<int>(
                    -kp * e_pos
                    - kpd * e_linear_velocity
                    - kt * e_theta
                    - ktd * e_angular_velocity
                );

                if (control_mode == ControlMode::Swingup) {
                    const double cos_e_theta = std::cos(e_theta);
                    int pwm_swing_up = static_cast<int>(
                        +kswing * (1 - cos_e_theta)
                        * sign(pendulum.angular_velocity() * cos_e_theta)
                        - ksx * pendulum.position()
                    );

                    // the switch between LQR and swing-up control laws is made less brutal by applying :
                    // - a progressive mix of both laws when entering the LQR range
                    // - an hysteresis when exiting the LQR range
                    if (in_lqr_range) {
                        stabilizing = true;
                    } else if (std::abs(e_theta) > lqr_to_swingup_threshold) {
                        stabilizing = false;
                        // reset the swing-up to LQR ramp
                        alpha = 0;
                    }
                    if (stabilizing) {
                        // don't switch instantaneously, by let's transition smoothly by blending both control laws
                        alpha = std::min(
                            alpha + std::chrono::duration<double>(dt_secs).count() / swingup_to_lqr_ramp_secs,
                            1.0
                        );
                    } else {
                        // if not stabilizing anymore, apply the swing-up PWM only
                        alpha = 0.0;
                    }
                    pwm = static_cast<int>(alpha * pwm_lqr + (1.0 - alpha) * pwm_swing_up);
                } else {
                    // when in LQR mode, just apply the LQR PWM when in proper range
                    pwm = in_lqr_range ? pwm_lqr : 0;
                }
            } else {
                // autopilot is off => don't move
                pwm = 0;
                // consider the track center as the target position
                e_pos = pendulum.position();
            }
        }

        pendulum.sendCommand(pwm);

        // take joystick into account only if it's time for it
        if (--iterations_to_joystick_update == 0) {
            joy.update();

            // should we exit?
            if (BTN_EXIT != -1 && joy.button(BTN_EXIT)) {
                std::cout << std::endl << std::endl
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

        was_in_lqr_range = in_lqr_range;

        // prepare dt_secs computation on next iteration
        last_time = now;
        last_time_is_set = true;
    } // end of the main loop

    pendulum.sendCommand(0);
    std::cout << RESET << std::endl;

    return EXIT_SUCCESS;
}
