// clang-format off
#include <math.h>

#include <chrono>
#include <cmath>
#include <thread>
#include <iomanip>

#include "pendulum_core/pendulum.hpp"

#include "pendulum_core/colors.h"
#include "pendulum_core/debug.hpp"
#include "pendulum_core/pigpio.hpp"
// clang-format on

#define STEP C_BR_MAGENTA

namespace pendulum_core {
    Pendulum::Pendulum(
        std::unique_ptr<Motor> motor,
        std::unique_ptr<Switch> left_switch,
        std::unique_ptr<Switch> right_switch,
        std::unique_ptr<PositionEncoder> position_encoder,
        std::unique_ptr<AngleEncoder> angle_encoder,
        const double safety_margin_meters
    )
        : hard_stop_margin_meters_(safety_margin_meters),
          motor_(std::move(motor)),
          left_switch_(std::move(left_switch)),
          right_switch_(std::move(right_switch)),
          position_encoder_(std::move(position_encoder)),
          angle_encoder_(std::move(angle_encoder)
          ) {
        PENDULUM_CORE_DBG("Creating Pendulum object");
        // Check that pointers are not nullptr
#define THROW_IF_NULL(component)                                                                                       \
    {                                                                                                                  \
        if ((component) == nullptr)                                                                                      \
            throw std::runtime_error("While creating Pendulum instance: " #component " cannot be nullptr!");            \
    }
        THROW_IF_NULL(motor_);
        THROW_IF_NULL(left_switch_);
        THROW_IF_NULL(right_switch_);
        THROW_IF_NULL(position_encoder_);
        THROW_IF_NULL(angle_encoder_);
        PENDULUM_CORE_DBG("Pendulum object created");
    }

    void Pendulum::calibrate_angle_encoder(const bool verbose) const {
        // Calibrate the angle encoder. It simply consists in checking that it is still.
        // Note: don't be too sensitive since the encoder is configured as 4096 steps
        // per turn
        const int angle_steps = angle_encoder_->raw_steps();
        constexpr int ENCODER_TOLERANCE = 4;
        bool moving = false;

        // Reset the encoder to start from a known state.
        // There's no risk doing it just now because if it's not presently still, the motion detection will abort
        // the process, and we can't thus go ahead with an invalid initial state.
        angle_encoder_->set_origin_here();

        // clang-format on
        angle_encoder_->set_safety_callbacks(
            angle_steps - ENCODER_TOLERANCE,
            angle_steps + ENCODER_TOLERANCE,
            [&] {
                moving = true;
            }
        );
        // clang-format on
        if (verbose) {
            std::cout << std::endl
                    << STEP << "- waiting for the pendulum to be still..." << std::endl
                    << C_RESET;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        if (moving) {
            if (verbose) {
                std::cout << C_ERROR << "Cannot start calibration : the pendulum is not still" << std::endl << C_RESET;
            }
            throw CalibrationFailed("the pendulum is moving");
        }

        // Cleanup motion detection callback
        angle_encoder_->remove_safety_callbacks();

        if (verbose) {
            std::cout << C_SUCCESS << "-> angle encoder initialized." << C_RESET << std::endl << std::endl;
        }
    }

    void Pendulum::calibrate_position_encoder(const int calibration_pwm, const bool verbose) {
        // Track ends as raw encoder counts: they are only meaningful until the origin is set below.
        int min_raw_steps = 0; // NOLINT(CppInitializedValueIsAlwaysRewritten)
        int max_raw_steps = 0; // NOLINT(CppInitializedValueIsAlwaysRewritten)

        if (verbose) {
            std::cout << STEP << "- calibrating position encoder now..." << C_RESET << std::endl;
        }

        // if we are already at the rightmost end of the track, back off a bit before
        // starting the calibration process (we don't care if we are at the leftmost
        // end, since we're going to move rightwards first anyway)
        if (!right_switch_->at_rest()) {
            std::cout << C_WARNING
                    << "-> carriage already at the right end of the track => backing off..." <<
                    C_RESET << std::endl;
            // move leftwards enough to clear the switch
            motor_->set_pwm(-calibration_pwm);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            motor_->set_pwm(0);

            // check if the switch is still active. If so, we have a real problem
            if (!right_switch_->at_rest()) {
                throw CalibrationFailed("right limit switch stuck in active state");
            }
        }

        // prepare some auxiliary callbacks used to reach the switches
        bool wrong_switch = false;
        auto switch_cb = [&] { motor_->set_pwm(0); };
        auto wrong_switch_cb = [&] {
            motor_->set_pwm(0);
            wrong_switch = true;
        };

        // Go towards the right switch
        if (verbose) {
            std::cout << "... seeking for the right end limit switch..." << std::endl;
        }
        right_switch_->enable_interrupts(switch_cb);
        left_switch_->enable_interrupts(wrong_switch_cb);
        motor_->set_pwm(calibration_pwm);
        while (motor_->pwm() != 0) {
            if (wrong_switch) {
                if (verbose) {
                    std::cerr << C_ERROR
                            << "Left limit switch activated while moving rightwards." << std::endl
                            << C_RESET;
                }
                throw CalibrationFailed("unexpected left limit switch activation");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        max_raw_steps = position_encoder_->raw_steps();

        // Release the right switch
        right_switch_->enable_interrupts(nullptr);
        motor_->set_pwm(-calibration_pwm);
        while (!right_switch_->at_rest_cached())
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        motor_->set_pwm(0);

        // Go towards the left switch
        if (verbose) {
            std::cout << "... seeking for the left end limit switch..." << std::endl;
        }

        right_switch_->enable_interrupts(wrong_switch_cb);
        left_switch_->enable_interrupts(switch_cb);
        motor_->set_pwm(-calibration_pwm);
        while (motor_->pwm() != 0) {
            if (wrong_switch) {
                if (verbose) {
                    std::cerr << C_ERROR
                            << "Right limit switch activated while moving leftwards." << std::endl
                            << C_RESET;
                }
                throw CalibrationFailed("unexpected right limit switch activation");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        min_raw_steps = position_encoder_->raw_steps();

        // Release the left switch
        left_switch_->enable_interrupts(nullptr);
        motor_->set_pwm(calibration_pwm);
        while (!left_switch_->at_rest_cached()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        motor_->set_pwm(0);

        // Check that min and max are coherent
        if (min_raw_steps > max_raw_steps) {
            if (verbose) {
                std::cerr << C_ERROR
                        << "Minimum and maximum position are inverted. " << std::endl
                        << "Possible cause :" << std::endl
                        << "- A/B signals are swapped" << std::endl
                        << "Swap A/B pin settings in the configuration file." << std::endl
                        << C_RESET;
            }
            throw CalibrationFailed("the minimum step position is greater than the maximum one. This likely "
                "means that the encoder 'reads backward'. Try inverting the A/B pins settings in the configuration.");
        }

        if (min_raw_steps == max_raw_steps) {
            if (verbose) {
                std::cerr << C_ERROR
                        << "Minimum and maximum position steps are equal. " << std::endl
                        << "Possible causes :" << std::endl
                        << "- the belt is loose or not on the pulleys anymore" << std::endl
                        << "- the encoder pulley screws are loose and the pulley is slipping on the shaft"
                        << "- an encoder wire is disconnected"
                        << "- the encoder is broken" << std::endl
                        << C_RESET;
            }
            throw CalibrationFailed("position min and max step counts are equal");
        }

        // define the "home" position, at mid-distance from both ends, as the origin of the physical frame
        position_encoder_->set_origin((max_raw_steps + min_raw_steps) / 2);
        min_position_meters_ = position_encoder_->steps_to_position(min_raw_steps);
        max_position_meters_ = position_encoder_->steps_to_position(max_raw_steps);
    }

    void Pendulum::calibrate(const int calibration_pwm, const bool verbose) {
        PENDULUM_CORE_DBG("Requested calibration via Pendulum::calibrate(" << calibration_pwm << ")");

        // Reset it no matter what!
        calibrated_ = false;

        // Ensure that we are not in emergency stop
        if (emergency_stopped_)
            throw CalibrationFailed("e_stop has been called");

        // Reset all callbacks
        position_encoder_->remove_safety_callbacks();
        angle_encoder_->remove_safety_callbacks();
        left_switch_->disable_interrupts();
        right_switch_->disable_interrupts();

        calibrate_angle_encoder(verbose);

        calibrate_position_encoder(calibration_pwm, verbose);

        // Go to the central position
        if (verbose) {
            std::cout << "... homing to center..." << std::endl;
        }
        motor_->set_pwm(calibration_pwm);
        while (position_encoder_->position() < 0.) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        motor_->set_pwm(0);

        if (verbose) {
            std::cout << C_SUCCESS << "-> position encoder calibrated." << C_RESET << std::endl << std::endl;
        }

        // Configure end-limit switches as hard limits that always stop the motor whenever they are reached
        // IMPORTANT :
        //  When using mechanical switches, this can lead to violent collision if the carriage is
        //  traveling full speed. Better use magnetic switches that trigger before the physical contact.
        left_switch_->enable_interrupts([&] { e_stop("left limit-switch activated"); });
        right_switch_->enable_interrupts([&] { e_stop("right limit-switch activated"); });

        // Configure encoder based travel limitation as an additional precaution. Emergency stop is
        // triggered at a safety distance from the physical switch.
        // This is not mandatory when using magnetic switches since this distance exists already thanks
        // to the physical principle of these sensors.
        soft_min_position_meters_ = min_position_meters_ + std::fabs(hard_stop_margin_meters_);
        soft_max_position_meters_ = max_position_meters_ - std::fabs(hard_stop_margin_meters_);
        position_encoder_->set_position_limits(
            soft_min_position_meters_,
            soft_max_position_meters_,
            [&] { e_stop("position lower limit reached"); },
            [&] { e_stop("position upper limit reached"); }
        );

        calibrated_ = true;

#define C_LABEL(v) C_BR_BLUE << "- " << std::setw(20) << std::left << (v) << " : " << C_RESET
        // raw encoder count (rounded to the nearest step) corresponding to a position
#define STEPS(pos) "  (" << std::setw(6) << std::right << position_encoder_->position_to_steps(pos) << " steps)"
#define DIST(v)  std::setw(6) << std::fixed << std::setprecision(3) << std::right << (v) << " m"

        if (verbose) {
            // clang-format off
            std::cout << C_HEADER << "Calibration complete." << C_RESET << std::endl
                    << C_LABEL("min position") << DIST(min_position_meters_) << STEPS(min_position_meters_) << std::endl
                    << C_LABEL("max position") << DIST(max_position_meters_) << STEPS(max_position_meters_) << std::endl
                    << C_LABEL("track length") << DIST(max_position_meters_ - min_position_meters_) << std::endl
                    << C_LABEL("center position") << DIST(0.) << STEPS(0.) << std::endl
                    << C_LABEL("hard stop margin") << DIST(hard_stop_margin_meters_) << std::endl
                    << C_LABEL("soft min position") << DIST(soft_min_position_meters_) << STEPS(soft_min_position_meters_) << std::endl
                    << C_LABEL("soft max position") << DIST(soft_max_position_meters_) << STEPS(soft_max_position_meters_) << std::endl
                    << C_LABEL("travel length") << DIST(soft_max_position_meters_ - soft_min_position_meters_) << std::endl
                    << std::endl;
            // clang-format on
        }
    }

    void Pendulum::update(const double dt) {
        if (emergency_stopped_)
            throw std::runtime_error("Pendulum::update(): e_stop() has been called");
        if (!calibrated_)
            throw NotCalibrated("Pendulum::update()");
        // Get encoder readings in the physical frame
        const double new_position = position_encoder_->position();
        const double new_angle = angle_encoder_->angle();
        // State estimation: TODO!
        linear_velocity_ = (new_position - position_) / dt;
        angular_velocity_ = (new_angle - angle_) / dt;
        position_ = new_position;
        angle_ = new_angle;
    }

    bool Pendulum::set_command(int pwm) const {
        if (emergency_stopped_)
            throw std::runtime_error("Pendulum::set_command(): e_stop() has been called");
        if (!calibrated_)
            throw NotCalibrated("Pendulum::set_command()");

        bool retval = true;
        if (pwm != 0) {
            if (pwm > 0)
                pwm += offset_up_;
            if (pwm < 0)
                pwm -= offset_down_;
            if (pwm > 255) {
                pwm = 255;
                retval = false;
            }
            if (pwm < -255) {
                pwm = -255;
                retval = false;
            }
        }
        motor_->set_pwm(pwm);
        return retval;
    }

    void Pendulum::set_pwm_offsets(const int offset_down, const int offset_up) {
        offset_up_ = std::max(0, offset_up);
        offset_down_ = std::max(0, offset_down);
    }

    void Pendulum::e_stop(const std::string &why) {
        motor_->set_pwm(0);
        emergency_stopped_ = true;
        throw EmergencyStop(why);
    }
} // namespace pendulum_core
