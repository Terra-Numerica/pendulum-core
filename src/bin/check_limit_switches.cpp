// clang-format off
#include <iomanip>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <csignal>
#include <cstdlib>

#include <pendule_pi/pendule.hpp>
#include <pendule_pi/config.hpp>
#include <pendule_pi/colors.h>
// clang-format on


constexpr auto ACT_COLOR = BRIGHT_GREEN;
constexpr auto INACT_COLOR = RESET;
constexpr auto LABEL_COLOR = BRIGHT_BLUE;
constexpr auto HI = BRIGHT_WHITE;


static void display_state(const std::string &label, const bool active) {
    constexpr auto STATIC_COLOR = RESET;
    std::cout << STATIC_COLOR << "[ " << label << " : " << RESET;
    if (active)
        std::cout << ACT_COLOR << "ACT";
    else
        std::cout << INACT_COLOR << "---";
    std::cout << STATIC_COLOR << " ]  " << RESET;
}

int main(const int argc, char **argv) {
    namespace pp = pendule_pi;
    const std::string program_name = argv[0];

    // Load all parameters from a configuration file.
    if (argc < 2) {
        std::cout << RED << "Usage: sudo " << program_name << " config_file.yaml" << RESET << std::endl;
        std::cout << "- 'sudo' is needed for pigpio to work" << std::endl;
        std::cout << "- the configuration file is needed to setup the interface" << std::endl;
        return EXIT_FAILURE;
    }

    try {
        pp::Config config(argv[1]);

        try {
            // Let the token manage the pigpio library!
            pigpio::ActivationToken token;

            signal(SIGINT, pigpio::ActivationToken::pleaseStop);
            signal(SIGTERM, pigpio::ActivationToken::pleaseStop);

            std::cout
                    << HI
                    << " ------------------------------------------------ " << std::endl
                    << "|                                                |" << std::endl
                    << "|   Limit switches configuration checking tool   |" << std::endl
                    << "|                                                |" << std::endl
                    << " ------------------------------------------------ " << std::endl
                    << RESET << std::endl
                    << "This tool checks that the track limit switches are properly configured and working." <<
                    std::endl
                    << std::endl;

            const auto left_switch = config.getLeftSwitch();
            const auto right_switch = config.getRightSwitch();

            std::cout << HI << "Configuration :" << RESET << std::endl;
            std::cout << LABEL_COLOR << "- left switch  : " << RESET << "GPIO " << left_switch->getPin() << std::endl;
            std::cout << LABEL_COLOR << "- right switch : " << RESET << "GPIO " << right_switch->getPin() << std::endl;

            std::cout << std::endl
                    << "If the wiring and configuration are OK, the switches must be shown as active" << std::endl
                    << "when a magnet is moved close to them." << std::endl
                    << std::endl;

            while (true) {
                constexpr auto POLLING_PERIOD_MS = 100;
                std::cout << '\r';
                display_state("left", !left_switch->atRest());
                display_state("right", !right_switch->atRest());
                std::cout.flush();

                std::this_thread::sleep_for(std::chrono::milliseconds(POLLING_PERIOD_MS));
            }
        } catch (const pigpio::ActivationToken::PleaseStop &) {
            std::cout << std::endl << YELLOW << "Termination signal received, terminating..." << std::endl;
        }

        std::cout << RESET;
        return EXIT_SUCCESS;
    } catch (const YAML::Exception &e) {
        std::cerr << "ERROR: YAML configuration loading error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch (const std::exception &e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
