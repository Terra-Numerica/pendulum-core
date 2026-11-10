#include <pendule_pi/config.hpp>
#include <digital_filters/filters.hpp>
#include <fstream>

#define THROW_IF_MISSING_1(x) { if(!yaml_[x]) throw MissingParameter(x); }
#define THROW_IF_MISSING_2(x,y) { if(!yaml_[x][y]) throw MissingParameter((x) + std::string("/") + (y)); }
#define THROW_IF_MISSING_3(x,y,z) { if(!yaml_[x][y][z]) throw MissingParameter((x) + std::string("/") + (y) + std::string("/") + (z)); }

#define GET_MACRO(_1,_2,_3,NAME,...) NAME
#define THROW_IF_MISSING(...) GET_MACRO(__VA_ARGS__, THROW_IF_MISSING_3, THROW_IF_MISSING_2, THROW_IF_MISSING_1)(__VA_ARGS__)
// NOTE: add new "overloads" by adding _4 to the list in GET_MACRO and
// THROW_IF_MISSING_4 just before the __VA_ARGS__ in GET_MACRO

constexpr unsigned int DEFAULT_SWITCH_DEBOUNCE_US = 5000;

namespace pendule_pi {
    Config::Config(
        const std::string &file_name
    )
        : yaml_(YAML::LoadFile(file_name)) {
    }


    std::unique_ptr<Switch> Config::getSwitch(
        const std::string &switch_name
    ) {
        THROW_IF_MISSING(switch_name);
        THROW_IF_MISSING(switch_name, "pin");
        THROW_IF_MISSING(switch_name, "normally_up");
        THROW_IF_MISSING(switch_name, "use_internal_pull_resistor");
        const auto config = yaml_[switch_name];
        return std::make_unique<Switch>(
            config["pin"].as<int>(),
            config["normally_up"].as<bool>(),
            config["use_internal_pull_resistor"].as<bool>(),
            config["debounce_us"].as<int>(DEFAULT_SWITCH_DEBOUNCE_US)
        );
    }


    std::unique_ptr<Encoder> Config::getEncoder(
        const std::string &encoder_name
    ) {
        THROW_IF_MISSING(encoder_name);
        THROW_IF_MISSING(encoder_name, "pin_a");
        THROW_IF_MISSING(encoder_name, "pin_b");
        const auto config = yaml_[encoder_name];
        return std::make_unique<Encoder>(
            config["pin_a"].as<int>(),
            config["pin_b"].as<int>()
        );
    }


    std::unique_ptr<Motor> Config::getMotor() {
        constexpr auto motor_name = "motor";
        THROW_IF_MISSING(motor_name);
        THROW_IF_MISSING(motor_name, "pwm_pin");
        THROW_IF_MISSING(motor_name, "direction_pin");
        const auto config = yaml_[motor_name];
        return std::make_unique<Motor>(
            config["pwm_pin"].as<int>(),
            config["direction_pin"].as<int>()
        );
    }


    std::unique_ptr<Pendule> Config::getPendule() {
        THROW_IF_MISSING("meters_per_step");
        THROW_IF_MISSING("radians_per_step");
        THROW_IF_MISSING("angle_offset");
        THROW_IF_MISSING("safety_thresholds");
        THROW_IF_MISSING("safety_thresholds", "hard");
        THROW_IF_MISSING("pwm_offsets");
        THROW_IF_MISSING("pwm_offsets", "low");
        THROW_IF_MISSING("pwm_offsets", "high");
        auto pendule = std::make_unique<Pendule>(
            yaml_["meters_per_step"].as<double>(),
            yaml_["radians_per_step"].as<double>(),
            yaml_["safety_thresholds"]["hard"].as<double>(),
            yaml_["angle_offset"].as<double>(),
            getMotor(),
            getLeftSwitch(),
            getRightSwitch(),
            getPositionEncoder(),
            getAngleEncoder()
        );
        const auto config = yaml_["pwm_offsets"];
        pendule->setPwmOffsets(
            config["low"].as<int>(),
            config["high"].as<int>()
        );
        return pendule;
    }


    std::unique_ptr<PenduleCppServer> Config::getPenduleServer() {
        THROW_IF_MISSING("sockets");
        THROW_IF_MISSING("sockets", "host");
        THROW_IF_MISSING("sockets", "state_port");
        THROW_IF_MISSING("sockets", "command_port");
        const auto config = yaml_["sockets"];
        return std::make_unique<PenduleCppServer>(
            config["host"].as<std::string>(),
            config["state_port"].as<std::string>(),
            config["command_port"].as<std::string>()
        );
    }


    int Config::getCalibrationPWM() {
        THROW_IF_MISSING("calibration_pwm");
        const auto pwm = yaml_["calibration_pwm"].as<int>();
        if (pwm <= 0)
            throw std::runtime_error("Parameter 'calibration_pwm' should be positive!");
        return pwm;
    }


    double Config::getSoftSafetyThreshold() {
        THROW_IF_MISSING("safety_thresholds");
        THROW_IF_MISSING("safety_thresholds", "soft");
        return yaml_["safety_thresholds"]["soft"].as<double>();
    }


    std::unique_ptr<pigpio::Rate> Config::getControlRate() {
        THROW_IF_MISSING("period_ms");
        const auto period_ms = yaml_["period_ms"].as<int>();
        if (period_ms <= 0)
            throw std::runtime_error("Parameter 'period_ms' should be positive!");
        return std::make_unique<pigpio::Rate>(1000 * period_ms);
    }


    std::unique_ptr<digital_filters::Filter<double, double> > Config::getFilter() {
        THROW_IF_MISSING("period_ms");
        THROW_IF_MISSING("cutoff_frequency");
        const auto period_ms = yaml_["period_ms"].as<int>();
        const auto cutoff_frequency = yaml_["cutoff_frequency"].as<double>();
        if (period_ms <= 0)
            throw std::runtime_error("Parameter 'period_ms' should be positive!");
        if (cutoff_frequency <= 0)
            throw std::runtime_error("Parameter 'cutoff_frequency' should be positive!");
        return std::make_unique<digital_filters::Filter<double, double> >(
            digital_filters::butterworth<double, double>(4, cutoff_frequency, 1000. / period_ms)
        );
    }
} // pendule_pi
