// clang-format off
#include <chrono>
#include <csignal>
#include <thread>
#include <unistd.h>
#include <fstream>


#include <pendule_pi/cli_parser.hpp>
#include <pendule_pi/colors.h>
#include <pendule_pi/pendulum_config.hpp>
#include <pendule_pi/pigpio.hpp>
#include <pendule_pi/pendule.hpp>
#include <digital_filters/filters_implementations/butterworth.hpp>

#include "utils.hpp"
#include "pendule_pi/pendule_cpp_server.hpp"
// clang-format on

// if set to 1, don't brake abruptly when reaching the evolution area limits
#define PROGRESSIVE_STOP 1


constexpr auto ERROR = C_BR_RED;
constexpr auto LABEL = C_BR_BLUE;

constexpr auto STATUS_FILE = "/var/run/pendulum.status";

static std::unique_ptr<pendule_pi::Pendule> make_pendulum(const PendulumConfig &config) {
    const auto [
        encoders_step_per_turn,
        timing_belt_module,
        pulley_teeth_count
    ] = config.hardware_specs;
    const double meters_per_step = pulley_teeth_count * timing_belt_module / encoders_step_per_turn / 1000.;
    const double radian_per_step = 2 * M_PI / encoders_step_per_turn;

    const auto make_switch = [](const SwitchConfig &cfg) {
        return std::make_unique<pendule_pi::Switch>(
            cfg.pin,
            cfg.normally_up,
            cfg.use_internal_pull_resistor,
            cfg.debounce_us
        );
    };

    const auto make_encoder = [](const EncoderConfig &cfg) {
        return std::make_unique<pendule_pi::Encoder>(cfg.pin_a, cfg.pin_b);
    };

    auto pendulum = std::make_unique<pendule_pi::Pendule>(
        meters_per_step,
        radian_per_step,
        config.safety_thresholds.hard,
        std::make_unique<pendule_pi::Motor>(config.motor.pwm_pin, config.motor.direction_pin),
        make_switch(config.left_switch),
        make_switch(config.right_switch),
        make_encoder(config.position_encoder),
        make_encoder(config.angle_encoder)
    );

    pendulum->setPwmOffsets(config.pwm_offsets.low, config.pwm_offsets.high);

    return pendulum;
}

/*
 * Digital filters
 */

static std::string filter_creation_error{};

using DigitalFilter = std::unique_ptr<digital_filters::Filter<double, double> >;

namespace {
    class FilterCreationError : public std::runtime_error {
    public:
        FilterCreationError(const std::string &filter_name, const std::string &reason)
            : std::runtime_error(filter_name + " creation failed (reason: " + reason + ")") {
        }
    };
}


static DigitalFilter make_filter(const std::string &name, const PendulumConfig &config) {
    try {
        return std::make_unique<digital_filters::Filter<double, double> >(
            digital_filters::butterworth<double, double>(4, config.cutoff_frequency, 1000. / config.period_ms)
        );
    } catch (const std::exception &e) {
        throw FilterCreationError(name, e.what());
    } catch (...) {
        throw FilterCreationError(name, "unknown exception");
    }
    return nullptr;
}

namespace {
    struct Filters {
        DigitalFilter position;
        DigitalFilter angle;
        DigitalFilter linear_velocity;
        DigitalFilter angular_velocity;
    };
}

static Filters create_filters(const PendulumConfig &config) {
    return Filters{
        make_filter("position", config),
        make_filter("angle", config),
        make_filter("linear_velocity", config),
        make_filter("angular_velocity", config),
    };
}

/*
 * Mainline of the process
 *
 * Invoked only when all the pre-flight checks have passed.
 */

static int run(const Filters &filters, const PendulumConfig &config) {
    // ensure the status file is cleared !
    std::remove(STATUS_FILE);

    // Create the timer used for enforcing a stable control rate.
    const auto rate = std::make_unique<pigpio::Rate>(config.period_ms * 1000);
    const double PERIOD_SEC = config.period_ms / 1000.0;

    // Socket connections.
    // const auto tcp = config->getPenduleServer();
    const auto tcp = std::make_unique<pendule_pi::PenduleCppServer>(
        config.sockets.host,
        config.sockets.state_port,
        config.sockets.command_port
    );

    const unsigned int MAX_MISSED_MESSAGES = 1 + static_cast<int>(1. / PERIOD_SEC);

    // Initialize the pigpio token that takes care of housekeeping at finalization time, among others.
    pigpio::ActivationToken token;

    signal(SIGINT, pigpio::ActivationToken::pleaseStop);
    signal(SIGTERM, pigpio::ActivationToken::pleaseStop);

    // Create the pendulum instance and perform the calibration.
    const auto pendule = make_pendulum(config);

    std::cout << std::endl << C_BR_WHITE
            << " ------------------------------------------------ " << std::endl
            << "|                                                |" << std::endl
            << "|    Pendulum low level controller starting...   |" << std::endl
            << "|                                                |" << std::endl
            << " ------------------------------------------------ " << std::endl
            << std::endl << LABEL << "Version " << PENDULE_PI_VERSION << std::endl
            << C_RESET;

    try {
        pendule->calibrate(config.calibration_pwm, true);
    } catch (...) {
        // create the status file to report we failed and exit now
        std::ofstream(STATUS_FILE).put('0');
        return EXIT_FAILURE;
    }

    // configure the filters with the initial conditions

    filters.position->initInput(pendule->position());
    filters.position->initOutput(pendule->position());
    filters.angle->initInput(pendule->angle());
    filters.angle->initOutput(pendule->angle());
    filters.linear_velocity->initInput(0.0);
    filters.linear_velocity->initOutput(0.0);
    filters.angular_velocity->initInput(0.0);
    filters.angular_velocity->initOutput(0.0);

    // ready to run now

    int exit_status = EXIT_SUCCESS;

    std::cout
            << C_BR_GREEN << "--- Pendulum ready ---" << C_RESET << std::endl
            << std::endl;

    // create the status file to report that we are up and running
    std::ofstream(STATUS_FILE).put('1');

    // sleep a little bit before starting with the main loop
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // set up the no speed-limit zone bounds
    const auto full_speed_max_position = pendule->softMaxPosition() - config.safety_thresholds.margin;
    const auto full_speed_min_position = pendule->softMinPosition() + config.safety_thresholds.margin;

#if PROGRESSIVE_STOP
    // braking distance must be shorter that the safety margin, otherwise the inertia
    // can make the base pass the position limits and trip the emergency stop
    const auto braking_distance = config.safety_thresholds.margin / 2;
#endif

    try {
        int pwm = 0;
        while (true) {
            // Sleep enough to adjust the loop rate and get the pendulum state
            const double hw_time = 1e-6 * rate->sleep();
            pendule->update(PERIOD_SEC);

            // perform state filtering
            const double filtered_position = filters.position->filter(pendule->position());
            const double filtered_angle = filters.angle->filter(pendule->angle());
            const double filtered_linear_vel = filters.linear_velocity->filter(pendule->linearVelocity());
            const double filtered_angular_vel = filters.angular_velocity->filter(pendule->angularVelocity());

            // publish the current state on the dedicated socket
            tcp->sendState(
                hw_time,
                filtered_position,
                filtered_angle,
                filtered_linear_vel,
                filtered_angular_vel
            );

            // read the command (i.e. PWM to be applied to the motor) from the dedicated socket
            const int max_wait_time = static_cast<int>(std::max(rate->residual() / 1000, 2u) - 2);
            if (tcp->readCommand(max_wait_time)) {
                pwm = tcp->command();
            } else if (tcp->missedCommands() > MAX_MISSED_MESSAGES) {
                // we lost too many messages: override the command!
                pwm = 0;
            }

            // Enforce soft safety limits before executing the command.

            // Don't stop brutally when reached, but slow progressively along the braking distance.
            // Note : we consider the track symmetrical, and thus use the same limits for both ends.
            if (pwm > 0 && pendule->position() > full_speed_max_position) {
#if PROGRESSIVE_STOP
                pwm = static_cast<int>(std::round(
                    pwm * std::fmax(1.0 - (pendule->position() - full_speed_max_position) / braking_distance, 0)
                ));
#else
                pwm = 0;
#endif
            } else if (pwm < 0 && pendule->position() < full_speed_min_position) {
#if PROGRESSIVE_STOP
                pwm = static_cast<int>(std::round(
                    pwm * std::fmax(1.0 - (full_speed_min_position - pendule->position()) / braking_distance, 0)
                ));
#else
                pwm = 0;
#endif
            }

            // ReSharper disable once CppExpressionWithoutSideEffects
            pendule->setCommand(pwm);
        }
    } catch (const pendule_pi::Pendule::EmergencyStop &e) {
        std::cerr << std::endl << C_BR_RED << "ERROR: " << e.what() << C_RESET << std::endl;
        exit_status = EXIT_FAILURE;
    } catch (const pigpio::ActivationToken::PleaseStop &) {
        std::cout << std::endl << C_BR_YELLOW << "PleaseStop token received, terminating..." << C_RESET <<
                std::endl;
        // ensure we leave with a happy status
        exit_status = EXIT_SUCCESS;
    }

    // clear the status file before leaving, whatever is the exit status
    std::remove(STATUS_FILE);
    return exit_status;
}

int main(const int argc, char **argv) {
    int exit_code = EXIT_SUCCESS;
    const auto cli_options = parse_cli(
        {
            .description = "Pendulum low level controller\n\n"
            "- reads the position and angle encoders, filters the measurements and publishes them on the state socket\n"
            "- gets the motor command from the control socket and generates the control signals for the driver\n"
            "- monitors the limit switches",
            .features = CliFeature::Check | CliFeature::Example,
            .print_example = [] { std::cout << get_pendulum_config_example(); },
        },
        argc, argv,
        exit_code
    );
    if (!cli_options) {
        return exit_code;
    }

    try {
        const auto config = load_pendulum_config(cli_options->config_path);
        if (cli_options->check) {
            std::cout << C_SUCCESS << "No error found in configuration." << C_RESET << std::endl;
            std::cout << C_HEADER << "Summary :\n" << C_RESET << yamlcfg::to_string(config, 2);
            return EXIT_SUCCESS;
        }
        // we need to be root for the rest of the processing
        if (getuid() != 0) {
            std::cout
                    << C_ERROR
                    << "ERROR: " << program_invocation_short_name << " must be executed as root."
                    << C_RESET << std::endl;
            return EXIT_FAILURE;
        }
        const auto filters = create_filters(config);
        // pre-flight check-list passed => let's start the real work now
        return run(filters, config);
    } catch (const FilterCreationError &e) {
        std::cerr << C_ERROR << "ERROR: Signal filter creation error\n" << e.what() << C_RESET << std::endl;
        return EXIT_FAILURE;
    } catch (const yamlcfg::InvalidConfiguration &e) {
        std::cerr << C_ERROR << "ERROR: Invalid configuration\n" << e.what() << C_RESET << std::endl;
        return EXIT_FAILURE;
    }

    // we never reach this point
}
