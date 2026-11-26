// clang-format off
#include <iomanip>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "../common/utils.hpp"
#include <pendulum_core/ansi_term.h>
#include <pendulum_core/joystick.hpp>
#include <pendulum_core/pendulum_cpp_client.hpp>
#include <pendulum_core/termination.hpp>
#include <pendulum_cli/cli_parser.hpp>
#include "demo_config.hpp"
// clang-format on

namespace {
    enum class ControlMode { Manual, LQR, Swingup };
}

namespace pc = pendulum_core;

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

#define C_LABEL C_BR_BLUE
#define CLEAR_LINE "\033[2K"

static void display_status_line(
    const ControlMode control_mode,
    const bool autopilot_engaged,
    const bool in_lqr_range,
    const double e_theta,
    const double e_position
) {
    std::cout << CLEAR_LINE << "\r[ Mode: " << C_CYAN << std::setw(7) << std::left << to_string(control_mode) << C_RESET
            << " ] [ LQR range: ";

    std::string msg; // NOLINT(*-init-variables)
    std::string color; // NOLINT(*-init-variables)

    const double e_theta_deg = e_theta * 180.0 / M_PI;

    if (in_lqr_range) {
        msg = "in";
        color = C_GREEN;
    } else {
        msg = "out";
        color = C_RED;
    }
    std::cout << color
            << std::setw(3) << std::left << msg << C_RESET
            << " ] [ Err: angle=" << C_CYAN
            << std::setw(6) << std::right << std::setprecision(1) << e_theta_deg << C_RESET
            << " | pos=" << C_CYAN
            // position error is displayed in cm
            << std::setw(5) << std::right << std::setprecision(1) << e_position * 100
            << C_RESET << " ] [ AP: " << std::setw(3) << std::left;

    if (control_mode != ControlMode::Manual) {
        if (autopilot_engaged) {
            msg = "on";
            color = C_GREEN;
        } else {
            msg = "off";
            color = C_RED;
        }
    } else {
        msg = "---";
        color = C_RESET;
    }
    std::cout << color << std::setw(3) << msg << C_RESET << " ]  " << std::flush;
}

namespace {
    // Smoothing filter used to replace brutal control changes produced by a D-pad
    class LowPassFilter {
    public:
        // time_constant_sec (tau): How sluggish the filter is (e.g., 0.5s or 1.0s).
        // sample_period_sec (dt):  Time between updates in seconds (20ms = 0.02s).
        LowPassFilter(const double time_constant_sec, const double sample_period_sec)
            : smoothed_value_(0.0), is_initialized_(false) {
            if (time_constant_sec <= 0.0) {
                alpha_ = 1.0; // No filtering
            } else {
                alpha_ = sample_period_sec / (time_constant_sec + sample_period_sec);
            }
        }

        double update(const double new_reading) {
            if (!is_initialized_) {
                smoothed_value_ = new_reading;
                is_initialized_ = true;
            } else {
                smoothed_value_ = alpha_ * new_reading + (1.0 - alpha_) * smoothed_value_;
            }
            return smoothed_value_;
        }

        [[maybe_unused]] void reset() {
            is_initialized_ = false;
        }

    private:
        double alpha_; // Computed smoothing factor
        double smoothed_value_; // Last computed value
        bool is_initialized_; // Prevents an artificial startup from 0
    };
}


static double rad_to_deg(const double rad) {
    return rad * 180. / M_PI;
}

static int run(const DemoConfig &config) {
    // Connect to the low-level interface.
    pc::PendulumCppClient pendulum(
        config.sockets.state_endpoint,
        config.sockets.command_endpoint,
        5
    );

    // Variables used to perform control.
    int pwm = 0;
    auto control_mode(ControlMode::Manual);
    double target_pos = 0;

    std::cout << C_HEADER
            << " ------------------------------------------------ " << std::endl
            << "|                                                |" << std::endl
            << "|          Pendulum demo starting...             |" << std::endl
            << "|                                                |" << std::endl
            << " ------------------------------------------------ " << std::endl
            << std::endl << C_MUTED << "Version " << PENDULUM_CORE_VERSION << std::endl
            << std::endl << C_RESET;

    std::cout << C_INFO << "--- Configuration ---" << C_RESET << std::endl << std::endl;

    auto coeff_fmt = [](std::ostream &os) -> std::ostream & {
        return os << C_RESET << std::fixed << std::setw(12) << std::setprecision(6);
    };

    // control gains for the swing-up phase
    const auto kswing = config.swing_up_gains.k;
    const auto ksx = config.swing_up_gains.kx;

    std::cout << C_HEADER << "Swing-up gains:" << C_RESET << std::endl
            << C_LABEL << "  kswing : " << coeff_fmt << kswing << std::endl
            << C_LABEL << "  ksx    : " << coeff_fmt << ksx << std::endl
            << std::endl;

    bool autopilot_engaged = false;
    bool in_lqr_range = false;
    bool was_in_lqr_range = false;
    bool refresh_status_requested = true;

    // control gains for the LQR phase
    const auto kp = config.lqr_gains.position;
    const auto kpd = config.lqr_gains.linear_velocity;
    const auto kt = config.lqr_gains.angle;
    const auto ktd = config.lqr_gains.angular_velocity;

    std::cout << C_HEADER << "LQR gains:" << C_RESET << std::endl
            << C_LABEL << "  kp     : " << coeff_fmt << kp << std::endl
            << C_LABEL << "  kpd    : " << coeff_fmt << kpd << std::endl
            << C_LABEL << "  kt     : " << coeff_fmt << kt << std::endl
            << C_LABEL << "  ktd    : " << coeff_fmt << ktd << std::endl
            << std::endl;

    // Create the joystick device to be used for controlling the demo.

    const auto AXIS_PWM = config.joystick.axis_pwm;
    const auto BTN_EXIT = config.joystick.btn_exit;
    const auto BTN_SWITCH = config.joystick.btn_switch;
    const auto BTN_AUTOPILOT = config.joystick.btn_autopilot;
    const auto USES_DPAD = config.joystick.uses_dpad;
    const auto DPAD_SMOOTHING_TAU = config.joystick.dpad_smoothing_tau_secs;

    const auto js_device = config.joystick.device;
    pc::Joystick joy(js_device);

    // The joystick is not polled on every iteration of the control loop.
    constexpr unsigned int JOY_UPDATE_PACE = 5;
    unsigned int iterations_to_joystick_update = JOY_UPDATE_PACE;

    // Mode switching button.
    CachedButton btn_mode_switch;

    // Autopilot button
    CachedButton btn_autopilot;

    // D-pad smoothing
    auto dpad_filter = LowPassFilter(DPAD_SMOOTHING_TAU, config.misc.period_ms / 1000.);

    std::cout << C_HEADER << "Control:" << C_RESET << std::endl
            << C_LABEL << "  Joystick device            : " << C_RESET << js_device << std::endl
            << C_LABEL << "  Axis for carriage control  : " << C_RESET << AXIS_PWM << std::endl
            << C_LABEL << "  Uses D-pad                 : " << C_RESET << USES_DPAD << std::endl
            << C_LABEL << "  D-pad smoothing tau        : " << C_RESET << std::setprecision(2) << DPAD_SMOOTHING_TAU <<
            " s"
            << std::endl
            << C_LABEL << "  Buttons : " << C_RESET << std::endl
            << C_LABEL << "   - switch control mode     : " << C_RESET << BTN_SWITCH << std::endl
            << C_LABEL << "   - toggle autopilot        : " << C_RESET << BTN_AUTOPILOT << std::endl
            << C_LABEL << "   - exit                    : " << C_RESET << (BTN_EXIT != -1
                                                                               ? std::to_string(BTN_EXIT)
                                                                               : "-- disabled --")
            << std::endl << std::endl;

    auto misc_fmt = [](std::ostream &os) -> std::ostream & {
        return os << C_RESET << std::fixed << std::setw(4) << std::setprecision(2);
    };

    const auto lqr_e_theta_max = config.misc.lqr_e_theta_max;
    const auto lqr_blend_margin_angle = config.misc.lqr_blend_margin_angle;
    const auto lqr_to_swing_up_hysteresis = config.misc.lqr_to_swing_up_hysteresis;

    std::cout << C_HEADER << "Misc:" << C_RESET << std::endl
            << C_LABEL << "  - LQR angle error max     : " << misc_fmt << lqr_e_theta_max
            << " rd  (" << rad_to_deg(lqr_e_theta_max) << "°)" << std::endl
            << C_LABEL << "  - LQR blend margin angle  : " << misc_fmt << lqr_blend_margin_angle
            << " rd  (" << rad_to_deg(lqr_blend_margin_angle) << "°)" << std::endl
            << C_LABEL << "  - LQR->swingup hysteresis : " << misc_fmt << lqr_to_swing_up_hysteresis
            << " rd  (" << rad_to_deg(lqr_to_swing_up_hysteresis) << "°)" << std::endl
            << std::endl;

    std::cout << C_SUCCESS << "Demo started " << C_RESET << "(use Ctrl-C ";

    if (BTN_EXIT != -1) {
        std::cout << "or the EXIT button of the gamepad ";
    }
    std::cout << "to exit)" << std::endl << std::endl;

    // Connect signals for graceful termination.
    pc::termination::install_handlers();

    // Periodic status refresh when no event triggers it
    const auto status_update_period = std::chrono::milliseconds(500);
    auto next_status_update = std::chrono::steady_clock::now() + status_update_period;

    double last_e_theta = 2 * M_PI; // out of range value to force the first update

    // LQR -> swing-up transition: the LQR is kept while the pendulum is not too far from the target,
    // or as long as it is coming back toward it, up to a hard limit
    const auto lqr_release_angle = lqr_e_theta_max + lqr_to_swing_up_hysteresis;
    const auto lqr_release_angle_max = lqr_release_angle + lqr_to_swing_up_hysteresis;

    // tells if the LQR is in charge while running in swing-up mode
    // (latched on entering the LQR range, released according to the above thresholds)
    bool lqr_engaged = false;

    std::cout << CURSOR_OFF << std::flush;

    // Main loop!
    while (!pc::termination::requested()) {
        constexpr double target_angle = M_PI;

        auto now = std::chrono::steady_clock::now();

        // Retrieve the current state of the pendulum.
        pendulum.read_state(pendulum_core::PendulumCppClient::BLOCKING);

        // Since the low level sends state messages periodically, as configured by period_ms in YAML file
        // and read_state has been called in blocking mode, we are here at regular intervals.

        // compute errors whatever is the current mode so that the status line can be used for visual control
        const double e_theta = shift_in_circle(pendulum.angle() - target_angle);
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

            double ref = robust_zero(joy.axis(AXIS_PWM), JOYSTICK_DEAD_ZONE);
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
                const double e_linear_velocity = pendulum.linear_velocity();
                const double e_angular_velocity = pendulum.angular_velocity();

                // let the user shift the cart target position by using the gamepad
                constexpr double POSITION_SHIFT_LIMIT = 0.2;
                target_pos = dead_zone_filter(joy.axis(AXIS_PWM), JOYSTICK_DEAD_ZONE) * POSITION_SHIFT_LIMIT / 32768.;
                e_pos = pendulum.position() - target_pos;

                // compute the LQR control law since we'll use its result whichever is the current mode
                const int pwm_lqr = static_cast<int>(
                    -kp * e_pos
                    - kpd * e_linear_velocity
                    - kt * e_theta
                    - ktd * e_angular_velocity
                );

                if (control_mode == ControlMode::Swingup) {
                    const double cos_e_theta = std::cos(e_theta);
                    const int pwm_swing_up = static_cast<int>(
                        +kswing * (1 - cos_e_theta)
                        * sign(pendulum.angular_velocity() * cos_e_theta)
                        - ksx * pendulum.position()
                    );

                    // the switch between LQR and swing-up control laws is made less brutal by applying :
                    // - a progressive mix of both laws in a transition zone located OUTSIDE the LQR range
                    //   (the LQR is the only law applied inside the LQR range)
                    // - an hysteresis when exiting the LQR range, to keep the LQR as long as it can recover
                    const double abs_e_theta = std::abs(e_theta);
                    if (in_lqr_range) {
                        lqr_engaged = true;
                    } else if (lqr_engaged) {
                        const bool moving_away = e_theta * pendulum.angular_velocity() > 0;
                        if ((abs_e_theta > lqr_release_angle && moving_away)
                            || abs_e_theta > lqr_release_angle_max) {
                            lqr_engaged = false;
                        }
                    }
                    const double alpha = lqr_engaged
                                             ? 1.0
                                             : blend_factor(abs_e_theta, lqr_e_theta_max, lqr_blend_margin_angle);
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

        pendulum.send_command(pwm);

        // take joystick into account only if it's time for it
        if (--iterations_to_joystick_update == 0) {
            joy.update();

            // should we exit?
            if (BTN_EXIT != -1 && joy.button(BTN_EXIT)) {
                std::cout << std::endl << std::endl
                        << C_BR_YELLOW << "--- Gamepad Exit button pressed. Quitting now ---" << C_RESET;
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
    } // end of the main loop

    pendulum.send_command(0);
    std::cout << CURSOR_ON << C_RESET << std::flush;

    return EXIT_SUCCESS;
}

int main(const int argc, char **argv) {
    int exit_code = EXIT_SUCCESS; // NOLINT(*-const-correctness)
    const auto cli_options = parse_cli(
        {
            .description = "Pendulum demonstrator",
            .features = CliFeature::Check | CliFeature::Example,
            .print_example = [] { std::cout << get_demo_config_example(); },
        },
        argc, argv,
        exit_code
    );

    if (!cli_options) {
        return exit_code;
    }

    try {
        const auto config = load_demo_config(cli_options->config_path);
        if (cli_options->check) {
            std::cout << C_SUCCESS << "No error found in configuration file." << C_RESET << std::endl;
            std::cout << C_HEADER << "Summary :\n" << C_RESET << yamlcfg::to_string(config, 2);
            return EXIT_SUCCESS;
        }
        return run(config);
    } catch (const yamlcfg::InvalidConfiguration &e) {
        std::cerr << C_ERROR << "ERROR: Invalid configuration\n" << e.what() << C_RESET << std::endl;
        return EXIT_FAILURE;
    } catch (const std::exception &e) {
        std::cerr << C_ERROR << "ERROR: " << e.what() << C_RESET << std::endl;
        return EXIT_FAILURE;
    }
}
