// clang-format off
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <fstream>


#include <pendulum_cli/cli_parser.hpp>
#include <pendulum_core/ansi_term.h>
#include <pendulum_core/pendulum_config.hpp>
#include <pendulum_core/pigpio.hpp>
#include <pendulum_core/pendulum.hpp>
#include <pendulum_core/termination.hpp>
#include <digital_filters/filters_implementations/butterworth.hpp>

#include "../common/loop_headroom.hpp"
#include "../common/utils.hpp"
#include "pendulum_core/pendulum_cpp_server.hpp"
// clang-format on

// if set to 1, don't brake abruptly when reaching the evolution area limits
#define PROGRESSIVE_STOP 1


constexpr auto ERROR = C_BR_RED;
constexpr auto LABEL = C_BR_BLUE;

constexpr auto STATUS_FILE = "/var/run/pendulum.status";

// Contents of the status file, read by the launcher script to tell what happened during the initialization
constexpr auto STATUS_READY = "1";
constexpr auto STATUS_FAILED = "0";
constexpr auto STATUS_INTERRUPTED = "-1"; // the user asked for a termination (CTRL+C) while initializing

static std::unique_ptr<pendulum_core::Pendulum> make_pendulum(const PendulumConfig &config) {
    const auto make_switch = [](const SwitchConfig &cfg) {
        return std::make_unique<pendulum_core::Switch>(
            cfg.pin,
            cfg.normally_up,
            cfg.use_internal_pull_resistor,
            cfg.debounce_us
        );
    };

    const auto make_position_encoder = [](const PositionEncoderConfig &cfg) {
        return std::make_unique<pendulum_core::PositionEncoder>(
            cfg.pin_a, cfg.pin_b, cfg.steps_per_turn,
            cfg.belt_module_mm, cfg.pulley_teeth_count, cfg.glitch_filter_us
        );
    };

    const auto make_angle_encoder = [](const AngleEncoderConfig &cfg) {
        return std::make_unique<pendulum_core::AngleEncoder>(
            cfg.pin_a, cfg.pin_b, cfg.steps_per_turn, cfg.glitch_filter_us
        );
    };

    auto pendulum = std::make_unique<pendulum_core::Pendulum>(
        std::make_unique<pendulum_core::Motor>(config.motor.pwm_pin, config.motor.direction_pin),
        make_switch(config.left_switch),
        make_switch(config.right_switch),
        make_position_encoder(config.position_encoder),
        make_angle_encoder(config.angle_encoder),
        config.safety_thresholds.hard
    );

    pendulum->set_pwm_offsets(config.pwm_offsets.low, config.pwm_offsets.high);

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
}

namespace {
    struct Filters { // NOLINT(*-pro-type-member-init)
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

    const unsigned int MAX_MISSED_MESSAGES = 1 + static_cast<int>(1. / PERIOD_SEC);

    // Initialize the pigpio session that takes care of housekeeping at finalization time, among others.
    const pigpio::Session session;

    pendulum_core::termination::install_handlers();

    // Create the pendulum instance and perform the calibration.
    const auto pendulum = make_pendulum(config);

    std::cout << std::endl << C_BR_WHITE
            << " ------------------------------------------------ " << std::endl
            << "|                                                |" << std::endl
            << "|    Pendulum hardware interface starting...     |" << std::endl
            << "|                                                |" << std::endl
            << " ------------------------------------------------ " << std::endl
            << std::endl << LABEL << "Version " << PENDULUM_CORE_VERSION << std::endl
            << C_RESET;

    try {
        pendulum->calibrate(config.calibration_pwm, true);
    } catch (...) {
        // create the status file to report why we leave and exit now
        std::ofstream(STATUS_FILE) << (pendulum_core::termination::requested() ? STATUS_INTERRUPTED : STATUS_FAILED);
        return EXIT_FAILURE;
    }

    // configure the filters with the initial conditions

    filters.position->init_input(pendulum->position());
    filters.position->init_output(pendulum->position());
    filters.angle->init_input(pendulum->angle());
    filters.angle->init_output(pendulum->angle());
    filters.linear_velocity->init_input(0.0);
    filters.linear_velocity->init_output(0.0);
    filters.angular_velocity->init_input(0.0);
    filters.angular_velocity->init_output(0.0);

    // Open the sockets only now that everything else is done, so that the files of the IPC sockets do not remain
    // after a failed or interrupted initialization. It must be done before reporting that we are ready.
    std::unique_ptr<pendulum_core::PendulumCppServer> server;
    try {
        server = std::make_unique<pendulum_core::PendulumCppServer>(
            config.sockets.state_endpoint,
            config.sockets.command_endpoint
        );
    } catch (const std::exception &e) {
        std::cerr << std::endl << C_BR_RED << "ERROR: cannot create the sockets: " << e.what() << std::endl
                << "Check the endpoints in the 'sockets' section of the configuration file." << C_RESET << std::endl;
        std::ofstream(STATUS_FILE) << STATUS_FAILED;
        return EXIT_FAILURE;
    }

    // ready to run now

    int exit_status = EXIT_SUCCESS; // NOLINT(CppInitializedValueIsAlwaysRewritten)

    std::cout
            << C_BR_GREEN << "--- Pendulum ready ---" << C_RESET << std::endl
            << std::endl;

    // create the status file to report that we are up and running
    std::ofstream(STATUS_FILE) << STATUS_READY;

    // sleep a little bit before starting with the main loop
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // set up the no speed-limit zone bounds
    const auto full_speed_max_position = pendulum->soft_max_position() - config.safety_thresholds.margin;
    const auto full_speed_min_position = pendulum->soft_min_position() + config.safety_thresholds.margin;

#if PROGRESSIVE_STOP
    // braking distance must be shorter that the safety margin, otherwise the inertia
    // can make the base pass the position limits and trip the emergency stop
    const auto braking_distance = config.safety_thresholds.margin / 2;
#endif

    // Headroom of the loop: part of the period left once the hardware work is done. The time spent
    // waiting for the command from the controller is not included, since it is not work of this process.
    LoopHeadroom headroom(PERIOD_SEC);
    double max_command_wait_secs = 0;

    try {
        int pwm = 0;
        while (!pendulum_core::termination::requested()) {
            // Sleep enough to adjust the loop rate and get the pendulum state
            const double hw_time = 1e-6 * rate->sleep();
            const auto work_start = std::chrono::steady_clock::now();
            pendulum->update(PERIOD_SEC);

            // perform state filtering
            const double filtered_position = filters.position->filter(pendulum->position());
            const double filtered_angle = filters.angle->filter(pendulum->angle());
            const double filtered_linear_vel = filters.linear_velocity->filter(pendulum->linear_velocity());
            const double filtered_angular_vel = filters.angular_velocity->filter(pendulum->angular_velocity());

            // publish the current state on the dedicated socket
            server->send_state(
                hw_time,
                filtered_position,
                filtered_angle,
                filtered_linear_vel,
                filtered_angular_vel
            );

            // read the command (i.e. PWM to be applied to the motor) from the dedicated socket
            const int max_wait_time = static_cast<int>(std::max(rate->residual() / 1000, 2u) - 2);
            const auto wait_start = std::chrono::steady_clock::now();
            const bool command_received = server->read_command(max_wait_time);
            const auto wait_end = std::chrono::steady_clock::now();
            if (command_received) {
                max_command_wait_secs = std::max(
                    max_command_wait_secs, std::chrono::duration<double>(wait_end - wait_start).count()
                );
                pwm = server->command();
            } else if (server->missed_commands() > MAX_MISSED_MESSAGES) {
                // we lost too many messages: override the command!
                pwm = 0;
            }

            // Enforce soft safety limits before executing the command.

            // Don't stop brutally when reached, but slow progressively along the braking distance.
            // Note : we consider the track symmetrical, and thus use the same limits for both ends.
            if (pwm > 0 && pendulum->position() > full_speed_max_position) {
#if PROGRESSIVE_STOP
                pwm = static_cast<int>(std::round(
                    pwm * std::fmax(1.0 - (pendulum->position() - full_speed_max_position) / braking_distance, 0)
                ));
#else
                pwm = 0;
#endif
            } else if (pwm < 0 && pendulum->position() < full_speed_min_position) {
#if PROGRESSIVE_STOP
                pwm = static_cast<int>(std::round(
                    pwm * std::fmax(1.0 - (full_speed_min_position - pendulum->position()) / braking_distance, 0)
                ));
#else
                pwm = 0;
#endif
            }

            // ReSharper disable once CppExpressionWithoutSideEffects
            pendulum->set_command(pwm);

            const auto work_end = std::chrono::steady_clock::now();
            headroom.update(
                std::chrono::duration<double>((wait_start - work_start) + (work_end - wait_end)).count()
            );
        }
    } catch (const pendulum_core::Pendulum::EmergencyStop &e) {
        std::cerr << std::endl << C_BR_RED << "ERROR: " << e.what() << C_RESET << std::endl;
        exit_status = EXIT_FAILURE;
    }

    if (headroom.count() > 0) {
        std::cout << std::endl << std::fixed << std::setprecision(2)
                << C_HEADER << "Control loop headroom (period: " << config.period_ms << " ms):" << C_RESET << std::endl
                << LABEL << "  - min  : " << C_RESET << headroom.min() * 1000. << " ms" << std::endl
                << LABEL << "  - mean : " << C_RESET << headroom.mean() * 1000. << " ms" << std::endl
                << LABEL << "  - overruns : " << C_RESET << headroom.overruns() << " / " << headroom.count()
                << std::endl
                << LABEL << "  - max command wait : " << C_RESET << max_command_wait_secs * 1000. << " ms"
                << std::endl;
    }

    if (pendulum_core::termination::requested()) {
        std::cout << std::endl << C_BR_YELLOW << "Termination requested, terminating..." << C_RESET << std::endl;
    }

    // clear the status file before leaving, whatever is the exit status
    std::remove(STATUS_FILE);
    return exit_status;
}

int main(const int argc, char **argv) {
    int exit_code = EXIT_SUCCESS;
    const auto cli_options = parse_cli(
        {
            .description = "Pendulum hardware interface\n\n"
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
