#pragma once
#include <yaml-cpp/yaml.h>
#include <pendule_pi/pigpio.hpp>
#include <pendule_pi/switch.hpp>
#include <pendule_pi/encoder.hpp>
#include <pendule_pi/motor.hpp>
#include <pendule_pi/pendule.hpp>
#include <pendule_pi/pendule_cpp_server.hpp>
#include <digital_filters/filter.hpp>


namespace pendule_pi {
    struct GPIOMapping {
        uint8_t left_switch_pin;
        uint8_t right_switch_pin;
        uint8_t position_encoder_A_pin;
        uint8_t position_encoder_B_pin;
        uint8_t angle_encoder_A_pin;
        uint8_t angle_encoder_B_pin;
        uint8_t motor_direction_pin;
        uint8_t motor_pwm_pin;

        // Return a lightweight array copy for generic iteration
        [[nodiscard]] constexpr std::array<uint8_t, 8> pins() const {
            return {
                left_switch_pin, right_switch_pin,
                position_encoder_A_pin, position_encoder_B_pin,
                angle_encoder_A_pin, angle_encoder_B_pin,
                motor_direction_pin, motor_pwm_pin
            };
        }
    };

    /// Class that stores configuration parameters read from a YAML file.
    class Config {
    public:
        /// Class to signal that a configuration file is missing a requested parameter.
        class MissingParameter : public std::runtime_error {
        public:
            /// Fill-in the exception message.
            explicit MissingParameter(const std::string &param)
                : std::runtime_error(
                    "The configuration file does not contain the requested parameter '" + param + "'") {
            }
        };

        /// Reads the configuration from the given YAML file.
        explicit Config(const std::string &file_name);

        /// Returns a Switch instance given the configuration for the left switch.
        std::unique_ptr<Switch> getLeftSwitch() { return getSwitch("left_switch"); }

        /// Returns a Switch instance given the configuration for the right switch.
        std::unique_ptr<Switch> getRightSwitch() { return getSwitch("right_switch"); }

        /// Returns an Encoder instance given the configuration for the position encoder.
        std::unique_ptr<Encoder> getPositionEncoder() { return getEncoder("position_encoder"); }

        /// Returns an Encoder instance given the configuration for the angle encoder.
        std::unique_ptr<Encoder> getAngleEncoder() { return getEncoder("angle_encoder"); }

        /// Returns a Motor instance that is set up according to the configuration file.
        std::unique_ptr<Motor> getMotor();

        /// Returns a Pendule instance, setup according to the configuration file.
        std::unique_ptr<Pendule> getPendule();

        /// Returns a PenduleCppServer instance, setup according to the configuration file.
        std::unique_ptr<PenduleCppServer> getPenduleServer();

        int getCalibrationPWM();

        double getSoftSafetyThreshold();

        std::unique_ptr<pigpio::Rate> getControlRate();

        std::unique_ptr<digital_filters::Filter<double, double> > getFilter();

        GPIOMapping getGPIOMapping();

    private:
        /// Helper function to return a Switch instance.
        std::unique_ptr<Switch> getSwitch(const std::string &switch_name);

        /// Helper function to return a Encoder instance.
        std::unique_ptr<Encoder> getEncoder(const std::string &encoder_name);

        YAML::Node yaml_;
    };
} // pendule_pi
