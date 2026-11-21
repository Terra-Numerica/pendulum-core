#include <iomanip>
#include <iostream>
#include <cstdlib>
#include <unistd.h>

#include <pigpio.h>
#include <pendule_pi/cli_parser.hpp>
#include <pendule_pi/pendulum_config.hpp>
#include <pendule_pi/colors.h>

static int do_reset(const PendulumConfig &config) {
    // check if we are root
    if (getuid() != 0) {
        std::cout << C_ERROR
                << "ERROR: this program must be executed as root." << std::endl
                << "Usage: sudo " << program_invocation_short_name << " -f config_file.yaml" << C_RESET << std::endl;
        return EXIT_FAILURE;
    }

    const auto rc = gpioInitialise();
    if (rc < 0) {
        std::cerr << C_ERROR << "ERROR: gpioInitialise failed with rc=" << rc << C_RESET << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << C_HEADER << "Resetting GPIOs to HiZ state..." << C_RESET << std::endl;

    for (const auto &[label, pin]: get_GPIO_mapping(config).all()) {
        gpioSetMode(pin, PI_INPUT);
        gpioSetPullUpDown(pin, PI_PUD_OFF);
        std::cout << "- GPIO " << pin << " (" << label << ") ok" << std::endl;
    }
    std::cout << C_SUCCESS << "All done." << C_RESET << std::endl;

    gpioTerminate();
    return EXIT_SUCCESS;
}

static int list_gpios(const PendulumConfig &config) {
    std::cout << C_HEADER << "GPIOs touched by the process :" << C_RESET << std::endl;
    for (const auto &[label, pin]: get_GPIO_mapping(config).all()) {
        std::cout << "- GPIO " << std::setw(2) << pin << " : " << label << std::endl;
    }
    return EXIT_SUCCESS;
}

int main(const int argc, char **argv) {
    int exit_code = EXIT_SUCCESS;
    const auto cli_options = parse_cli(
        {
            .description = "GPIO reset tool\n\n"
            "Resets the GPIOs used in the pendulum configuration to their default hardware state.",
            .features = CliFeature::ListGpios,
        },
        argc, argv,
        exit_code
    );
    if (!cli_options) {
        return exit_code;
    }

    try {
        const auto config = load_pendulum_config(cli_options->config_path);

        if (cli_options->list_gpios)
            return list_gpios(config);

        return do_reset(config);
    } catch (const yamlcfg::InvalidConfiguration &e) {
        std::cerr << C_ERROR << "ERROR: Invalid configuration\n" << e.what() << C_RESET << std::endl;
        return EXIT_FAILURE;
    }
}
