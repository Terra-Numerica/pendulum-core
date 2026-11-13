// clang-format off
#include <chrono>
#include <csignal>
#include <thread>
#include <unistd.h>
#include <fstream>

#include <pendule_pi/colors.h>
#include <pendule_pi/config.hpp>
#include <pendule_pi/debug.hpp>
#include <pendule_pi/pigpio.hpp>

#include "utils.hpp"
// clang-format on

namespace pp = pendule_pi;


constexpr auto ERROR = BRIGHT_RED;
constexpr auto LABEL = BRIGHT_BLUE;

constexpr auto STATUS_FILE = "/var/run/pendulum.status";


int main(const int argc, char **argv) {
    const std::string program_name = argv[0];

    // check if we are root
    if (getuid() != 0) {
        std::cout << ERROR
                << "ERROR: this program must be executed as root." << std::endl
                << "Usage: sudo " << program_name << " config_file.yaml" << RESET << std::endl;
        return EXIT_FAILURE;
    }

    // ensure the status file is cleared
    std::remove(STATUS_FILE);

    // check if the configuration file is passed as argument
    if (argc < 2) {
        std::cout << ERROR
                << "ERROR: missing configuration file argument." << std::endl
                << "Usage: sudo " << program_name << " config_file.yaml" << RESET << std::endl;
        return EXIT_FAILURE;
    }

    const auto config = pp::Config::loadFromFile(argv[1]);
    if (!config) {
        return EXIT_FAILURE;
    }

    try {
        // Let the token manage the pigpio library!
        pigpio::ActivationToken token;

        signal(SIGINT, pigpio::ActivationToken::pleaseStop);
        signal(SIGTERM, pigpio::ActivationToken::pleaseStop);

        // Create the pendulum instance and perform the calibration.
        const auto pendule = config->getPendule();

        std::cout << std::endl << BRIGHT_WHITE
                << " ------------------------------------------------ " << std::endl
                << "|                                                |" << std::endl
                << "|    Pendulum low level controller starting...   |" << std::endl
                << "|                                                |" << std::endl
                << " ------------------------------------------------ " << std::endl
                << std::endl << LABEL << "Version " << PENDULE_PI_VERSION << std::endl
                << RESET;

        try {
            pendule->calibrate(config->getCalibrationPWM(), true);
        } catch (...) {
            // create the status file to report we failed and exit now
            std::ofstream(STATUS_FILE).put('0');
            return EXIT_FAILURE;
        }

        // if here, the calibration process went fine, anomalies being reported by raising exceptions.

        std::cout
                << BRIGHT_GREEN << "--- Pendulum ready ---" << RESET << std::endl
                << std::endl;

        // create the status file to report that we are up and running
        std::ofstream(STATUS_FILE).put('1');

        // Define soft limits for the pendulum.
        const double MAX_POSITION = pendule->softMinMaxPosition() - config->getSoftSafetyThreshold();

        // Create the timer used for enforcing a stable control rate.
        const auto rate = config->getControlRate();
        const double PERIOD_SEC = rate->period() / 1000000.0;

        // Setup filters.
        const auto filter_position = config->getFilter();
        const auto filter_angle = config->getFilter();
        const auto filter_linear_vel = config->getFilter();
        const auto filter_angular_vel = config->getFilter();

        filter_position->initInput(pendule->position());
        filter_position->initOutput(pendule->position());
        filter_angle->initInput(pendule->angle());
        filter_angle->initOutput(pendule->angle());
        filter_linear_vel->initInput(0.0);
        filter_linear_vel->initOutput(0.0);
        filter_angular_vel->initInput(0.0);
        filter_angular_vel->initOutput(0.0);

        // Control and process variables.
        int pwm = 0;

        // Socket connections.
        const auto tcp = config->getPenduleServer();
        const unsigned int MAX_MISSED_MESSAGES = 1 + static_cast<int>(1. / PERIOD_SEC);

        // sleep a little bit before starting with the main loop
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Main loop!
        while (true) {
            // Sleep and update the state of the pendulum
            const double hw_time = 1e-6 * rate->sleep();
            pendule->update(PERIOD_SEC);

            // perform state filtering
            const double filtered_position = filter_position->filter(pendule->position());
            const double filtered_angle = filter_angle->filter(pendule->angle());
            const double filtered_linear_vel = filter_linear_vel->filter(pendule->linearVelocity());
            const double filtered_angular_vel = filter_angular_vel->filter(pendule->angularVelocity());

            // send the current state
            tcp->sendState(hw_time, filtered_position, filtered_angle, filtered_linear_vel, filtered_angular_vel);

            // read the current command
            const int max_wait_time = static_cast<int>(std::max(rate->residual() / 1000, 2u) - 2);
            if (tcp->readCommand(max_wait_time)) {
                pwm = tcp->command();
            } else if (tcp->missedCommands() > MAX_MISSED_MESSAGES) {
                // we lost too many messages: override the command!
                pwm = 0;
            }

            // Enforce soft safety limits, then send the command.
            if (
                (pendule->position() > MAX_POSITION && pwm > 0) ||
                (pendule->position() < -MAX_POSITION && pwm < 0)
            ) {
                pwm = 0;
            }

            // ReSharper disable once CppExpressionWithoutSideEffects
            pendule->setCommand(pwm);
        }
    } catch (const pigpio::ActivationToken::PleaseStop &) {
        std::cout << std::endl << BRIGHT_YELLOW << "PleaseStop token received, terminating..." << RESET << std::endl;
    }

    // clear the status file before leaving
    std::remove(STATUS_FILE);

    return EXIT_SUCCESS;
}
