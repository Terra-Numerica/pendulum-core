#include <pigpio.h>
#include <iostream>
#include <pendule_pi/config.hpp>
#include <pendule_pi/colors.h>

int main(const int argc, char **argv) {
    namespace pp = pendule_pi;

    const std::string program_name = argv[0];

    // Load all parameters from a configuration file.
    if (argc < 2) {
        std::cout << RED << "Usage: sudo " << argv[0] << " config_file.yaml" << RESET << std::endl;
        std::cout << "- 'sudo' is needed for pigpio to work" << std::endl;
        std::cout << "- the configuration file is needed to know which GPIOs to reset" << std::endl;
        return EXIT_FAILURE;
    }

    const auto rc = gpioInitialise();
    if (rc < 0) {
        std::cerr << "gpioInitialise failed with rc=" << rc << std::endl;
        return 1;
    }

    pp::Config config(argv[1]);

    std::cout << "Resetting GPIOs to HiZ state..." << std::endl;

    for (const int pin: config.getGPIOMapping().pins()) {
        gpioSetMode(pin, PI_INPUT);
        gpioSetPullUpDown(pin, PI_PUD_OFF);
        std::cout << "- GPIO " << pin << " ok" << std::endl;
    }
    std::cout << "All done." << std::endl;

    gpioTerminate();
    return 0;
}
