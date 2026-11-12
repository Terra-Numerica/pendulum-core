#include <iostream>
#include <cstdlib>
#include <pigpio.h>
#include <pendule_pi/config.hpp>
#include <pendule_pi/colors.h>

int main(const int argc, char **argv) {
    namespace pp = pendule_pi;

    const std::string program_name = argv[0];

    // Load all parameters from a configuration file.
    if (argc < 2) {
        std::cout << RED << "Usage: sudo " << program_name << " config_file.yaml" << RESET << std::endl;
        std::cout << "- 'sudo' is needed for pigpio to work" << std::endl;
        std::cout << "- the configuration file is needed to know which GPIOs to reset" << std::endl;
        return EXIT_FAILURE;
    }

    try {
        pp::Config config(argv[1]);


        const auto rc = gpioInitialise();
        if (rc < 0) {
            std::cerr << "gpioInitialise failed with rc=" << rc << std::endl;
            return EXIT_FAILURE;
        }

        std::cout << "Resetting GPIOs to HiZ state..." << std::endl;

        for (const int pin: config.getGPIOMapping().pins()) {
            gpioSetMode(pin, PI_INPUT);
            gpioSetPullUpDown(pin, PI_PUD_OFF);
            std::cout << "- GPIO " << pin << " ok" << std::endl;
        }
        std::cout << "All done." << std::endl;

        gpioTerminate();
        return EXIT_SUCCESS;
    } catch (const YAML::Exception &e) {
        std::cerr << "ERROR: YAML configuration loading error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch (const std::exception &e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
