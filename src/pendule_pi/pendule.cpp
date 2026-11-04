// clang-format off
#include <chrono>
#include <cmath>
#include <thread>

#include "pendule_pi/pendule.hpp"

#include "pendule_pi/colors.h"
#include "pendule_pi/debug.hpp"
#include "pendule_pi/pigpio.hpp"
// clang-format on

namespace pendule_pi {
    Pendule::Pendule(
        const double meters_per_step,
        const double radians_per_step,
        const double safety_margin_meters,
        const double rest_angle,
        std::unique_ptr<Motor> motor,
        std::unique_ptr<Switch> left_switch,
        std::unique_ptr<Switch> right_switch,
        std::unique_ptr<Encoder> position_encoder,
        std::unique_ptr<Encoder> angle_encoder
    )
        : calibrated_(false),
          emergency_stopped_(false),
          safety_margin_meters_(safety_margin_meters),
          meters_per_step_(meters_per_step),
          radians_per_step_(radians_per_step),
          rest_angle_(rest_angle),
          offset_up_(0),
          offset_down_(0),
          offset_static_(0),
          motor_(std::move(motor)),
          left_switch_(std::move(left_switch)),
          right_switch_(std::move(right_switch)),
          position_encoder_(std::move(position_encoder)),
          angle_encoder_(std::move(angle_encoder)
          ) {
        PENDULE_PI_DBG("Creating Pendule object");
        // Check that pointers are not nullptr
#define THROW_IF_NULL(component)                                                                                       \
    {                                                                                                                  \
        if (component == nullptr)                                                                                      \
            throw std::runtime_error("While creating Pendule instance: " #component " cannot be nullptr!");            \
    }
        THROW_IF_NULL(motor_);
        THROW_IF_NULL(left_switch_);
        THROW_IF_NULL(right_switch_);
        THROW_IF_NULL(position_encoder_);
        THROW_IF_NULL(angle_encoder_);
        PENDULE_PI_DBG("Pendule object created");
    }

    void Pendule::calibrate(const int calibration_pwm, const bool verbose) {
        PENDULE_PI_DBG("Requested calibration via Pendule::calibrate(" << calibration_pwm << ")");

        // Reset it no matter what!
        calibrated_ = false;

        // Ensure that we are not in emergency stop
        if (emergency_stopped_)
            throw CalibrationFailed("eStop has been called");

        // Reset all callbacks
        position_encoder_->removeSafetyCallbacks();
        angle_encoder_->removeSafetyCallbacks();
        left_switch_->disableInterrupts();
        right_switch_->disableInterrupts();

        // Calibrate the angle encoder. It simply means that we want to ensure that
        // it is static and that its initial position is zero!
        // Note: don't be to sensitive since the encoder is configured as 4096 steps
        // per turn
        const int angle_steps = angle_encoder_->steps();
        constexpr int encoder_tolerance = 4;
        bool moving = false;

        // clang-format on
        angle_encoder_->setSafetyCallbacks(
            angle_steps - encoder_tolerance,
            angle_steps + encoder_tolerance,
            [&]() {
                moving = true;
            }
        );
        // clang-format on
        if (verbose) {
            std::cout << std::endl
                    << BRIGHT_MAGENTA << "- waiting for the pendulum to be still..." << std::endl
                    << RESET;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        if (moving) {
            if (verbose) {
                std::cout << RED << "Cannot start calibration : the pendulum is not still" << std::endl << RESET;
            }
            throw CalibrationFailed("the pendulum is moving");
        }


        // [EP] is this test really useful ? It tends to throw false positives on a
        // lot of occasions.

        // Check the angle encoder position. It should be zero, otherwise we could have a problem.
        if (angle_encoder_->steps() != 0) {
            std::cout << YELLOW
                    << "WARNING: the angle encoder is at a non-zero position (" << angle_encoder_->steps()
                    << "). Resetting it to zero." << RESET << std::endl;
            // don't throw the fata error anymore but inform and reset the encoder instead
            // throw CalibrationFailed("the angle encoder is at a non-zero position");
            angle_encoder_->reset();
        }

        if (verbose) {
            std::cout << BRIGHT_GREEN << "-> angle encoder calibrated." << RESET << std::endl << std::endl;

            std::cout << BRIGHT_MAGENTA << "- calibrating position encoder now..." << RESET << std::endl;
        }

        // if we are already at the rightmost end of the track, back off a bit before
        // starting the calibration process (we don't care if we are at the leftmost
        // end, since we're going to move rightwards first anymway)
        if (!right_switch_->atRest()) {
            std::cout << YELLOW << "-> carriage already at the right end of the track => backing off..." <<
                    RESET << std::endl;
            // move leftwards enough to clear the switch
            motor_->setPWM(-calibration_pwm);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            motor_->setPWM(0);

            // check if the switch is still active. If so, we have a real problem
            if (!right_switch_->atRest()) {
                throw CalibrationFailed("right limit switch stuck in active state");
            }
        }

        // Remove callbacks on the angle encoder
        angle_encoder_->removeSafetyCallbacks();

        // prepare some auxiliary callbacks used to reach the switches
        bool wrong_switch = false;
        auto switch_cb = [&]() { motor_->setPWM(0); };
        auto wrong_switch_cb = [&]() {
            motor_->setPWM(0);
            wrong_switch = true;
        };

        // Go towards the right switch
        if (verbose) {
            std::cout << "... moving to the right end of the track..." << std::endl;
        }
        right_switch_->enableInterrupts(switch_cb);
        left_switch_->enableInterrupts(wrong_switch_cb);
        motor_->setPWM(calibration_pwm);
        while (motor_->getPWM() != 0) {
            if (wrong_switch) {
                if (verbose) {
                    std::cerr << RED
                            << "Left limit switch activated while moving rightwards." << std::endl
                            << RESET;
                }
                throw CalibrationFailed("unexpected left limit switch activation");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        max_position_steps_ = position_encoder_->steps();

        // Release the right switch
        right_switch_->enableInterrupts(nullptr);
        motor_->setPWM(-calibration_pwm);
        while (!right_switch_->atRestCached())
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        motor_->setPWM(0);

        // Go towards the left switch
        if (verbose) {
            std::cout << "... moving to the left end of the track..." << std::endl;
        }

        right_switch_->enableInterrupts(wrong_switch_cb);
        left_switch_->enableInterrupts(switch_cb);
        motor_->setPWM(-calibration_pwm);
        while (motor_->getPWM() != 0) {
            if (wrong_switch) {
                if (verbose) {
                    std::cerr << RED
                            << "Right limit switch activated while moving leftwards." << std::endl
                            << RESET;
                }
                throw CalibrationFailed("unexpected right limit switch activation");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        min_position_steps_ = position_encoder_->steps();

        // Release the left switch
        left_switch_->enableInterrupts(nullptr);
        motor_->setPWM(calibration_pwm);
        while (!left_switch_->atRestCached()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        motor_->setPWM(0);

        // Check that min and max are coherent
        if (min_position_steps_ > max_position_steps_) {
            if (verbose) {
                std::cerr << RED
                        << "Minimum and maximum position are inverted. " << std::endl
                        << "Possible cause :" << std::endl
                        << "- A/B signals are swapped" << std::endl
                        << "Swap A/B pin settings in the configuration file." << std::endl
                        << RESET;
            }
            throw CalibrationFailed("the minimum step position is greater than the maximum one. This likely "
                "means that the encoder 'reads backward'. Try inverting the A/B pins settings in the configuration.");
        }

        if (min_position_steps_ == max_position_steps_) {
            if (verbose) {
                std::cerr << RED
                        << "Minimum and maximum position steps are equal. " << std::endl
                        << "Possible causes :" << std::endl
                        << "- the belt is loose or not on the pulleys anymore" << std::endl
                        << "- the encoder pulley screws are loose and the pulley is slipping on the shaft"
                        << "- an encoder wire is disconnected"
                        << "- the encoder is broken" << std::endl
                        << RESET;
            }
            throw CalibrationFailed("position min and max step counts are equal");
        }
        mid_position_steps_ = (max_position_steps_ + min_position_steps_) / 2;

        // Go to the central position
        if (verbose) {
            std::cout << "... moving to the central position..." << std::endl;
        }
        right_switch_->disableInterrupts();
        left_switch_->disableInterrupts();
        motor_->setPWM(calibration_pwm);
        while (position_encoder_->steps() < mid_position_steps_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        motor_->setPWM(0);

        if (verbose) {
            std::cout << BRIGHT_GREEN << "-> position encoder calibrated." << RESET << std::endl << std::endl;
        }

        // Ok, we are in the central position!
        left_switch_->enableInterrupts([&]() { eStop("left switch hit"); });
        right_switch_->enableInterrupts([&]() { eStop("right switch hit"); });
        int safety_margin_steps = static_cast<int>(std::ceil(std::fabs(safety_margin_meters_ / meters_per_step_)));
        int soft_min = min_position_steps_ + safety_margin_steps;
        int soft_max = max_position_steps_ - safety_margin_steps;
        position_encoder_->setSafetyCallbacks(
            soft_min,
            soft_max,
            [&]() { eStop("Software-imposed minimum position limit reached"); },
            [&]() { eStop("Software-imposed maximum position limit reached"); }
        );
        soft_minmax_position_meters_ = std::fabs(steps2meters(soft_max));
        calibrated_ = true;

        if (verbose) {
            // clang-format off
            std::cout << BRIGHT_WHITE << "Calibration complete." << RESET << std::endl
                    << "- min steps          : " << min_position_steps_ << " (in meters: " << steps2meters(min_position_steps_) << ")" << std::endl
                    << "- max steps          : " << max_position_steps_ << " (in meters: " << steps2meters(max_position_steps_) << ")" << std::endl
                    << "- middle steps       : " << mid_position_steps_ << " (in meters: " << steps2meters(mid_position_steps_) << ")" << std::endl
                    << "- safety_margin_steps: " << safety_margin_steps << " (in meters: " << safety_margin_meters_ << ")" << std::endl
                    << "- soft_min           : " << soft_min << " (in meters: " << steps2meters(soft_min) << ")" << std::endl
                    << "- soft_max           : " << soft_max << " (in meters: " << steps2meters(soft_max) << ")" << std::endl
                    << std::endl;
            // clang-format on
        }
    }

    void Pendule::update(double dt) {
        if (emergency_stopped_)
            throw std::runtime_error("Pendule::update(): eStop() has been called");
        if (!calibrated_)
            throw NotCalibrated("Pendule::update()");
        // Get raw encoder reading
        const double new_position = steps2meters(position_encoder_->steps());
        const double new_angle = steps2radians(angle_encoder_->steps());
        // State estimation: TODO!
        linvel_ = (new_position - position_) / dt;
        angvel_ = (new_angle - angle_) / dt;
        position_ = new_position;
        angle_ = new_angle;
    }

    bool Pendule::setCommand(int pwm) const {
        if (emergency_stopped_)
            throw std::runtime_error("Pendule::setCommand(): eStop() has been called");
        if (!calibrated_)
            throw NotCalibrated("Pendule::setCommand()");

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
        motor_->setPWM(pwm);
        return retval;
    }

    const int &Pendule::minPositionSteps() const {
        if (!calibrated_)
            throw NotCalibrated("Pendule::minPositionSteps()");
        return min_position_steps_;
    }

    const int &Pendule::maxPositionSteps() const {
        if (!calibrated_)
            throw NotCalibrated("Pendule::maxPositionSteps()");
        return max_position_steps_;
    }

    const int &Pendule::midPositionSteps() const {
        if (!calibrated_)
            throw NotCalibrated("Pendule::midPositionSteps()");
        return mid_position_steps_;
    }

    const double &Pendule::softMinMaxPosition() const {
        if (!calibrated_)
            throw NotCalibrated("Pendule::softMinMaxPosition()");
        return soft_minmax_position_meters_;
    }

    void Pendule::setPwmOffsets(int offset_down, int offset_up) {
        offset_up_ = std::max(0, offset_up);
        offset_down_ = std::max(0, offset_down);
    }

    void Pendule::eStop(const std::string &why) {
        motor_->setPWM(0);
        emergency_stopped_ = true;
        throw EmergencyStop(why);
    }
} // namespace pendule_pi
