#include <iostream>
#include <cstdlib>
#include <unistd.h>

#include <pigpio.h>
#include <pendule_pi/config.hpp>
#include <pendule_pi/colors.h>

namespace pp = pendule_pi;

constexpr auto ERROR = BRIGHT_RED;


int main(const int argc, char **argv) {
    const std::string program_name = argv[0];

    // check if we are root
    if (getuid() != 0) {
        std::cout << ERROR
                << "ERROR: this program must be executed as root." << std::endl
                << "Usage: sudo " << program_name << " config_file.yaml" << RESET << std::endl;
        return EXIT_FAILURE;
    }

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

    constexpr auto HI = BRIGHT_WHITE;
    constexpr auto SUCCESS = BRIGHT_GREEN;

    const auto rc = gpioInitialise();
    if (rc < 0) {
        std::cerr << ERROR << "ERROR: gpioInitialise failed with rc=" << rc << RESET << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << HI << "Resetting GPIOs to HiZ state..." << RESET << std::endl;

    for (const int pin: config->getGPIOMapping().pins()) {
        gpioSetMode(pin, PI_INPUT);
        gpioSetPullUpDown(pin, PI_PUD_OFF);
        std::cout << "- GPIO " << pin << " ok" << std::endl;
    }
    std::cout << SUCCESS << "All done." << RESET << std::endl;

    gpioTerminate();
    return EXIT_SUCCESS;
}
