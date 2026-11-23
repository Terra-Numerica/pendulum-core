// clang-format off
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <csignal>
#include <cstdlib>

#include <pendulum_cli/cli_parser.hpp>
#include <pendulum_core/ansi_term.h>
#include <pendulum_core/pendulum_config.hpp>
#include <pendulum_core/pigpio.hpp>
#include <pendulum_core/switch.hpp>
// clang-format on


constexpr auto ACT_COLOR = C_BR_GREEN;
constexpr auto INACT_COLOR = C_RESET;
constexpr auto LABEL_COLOR = C_BR_BLUE;
constexpr auto HI = C_BR_WHITE;


static void display_state(const std::string &label, const bool active) {
    constexpr auto STATIC_COLOR = C_RESET;
    std::cout << STATIC_COLOR << "[ " << label << " : " << C_RESET;
    if (active)
        std::cout << ACT_COLOR << "ACT";
    else
        std::cout << INACT_COLOR << "---";
    std::cout << STATIC_COLOR << " ]  " << C_RESET;
}

static int run(const PendulumConfig &config) {
    namespace pc = pendulum_core;

    try {
        // Let the token manage the pigpio library!
        const pigpio::ActivationToken token;

        signal(SIGINT, pigpio::ActivationToken::please_stop);
        signal(SIGTERM, pigpio::ActivationToken::please_stop);

        std::cout
                << HI
                << " ------------------------------------------------ " << std::endl
                << "|                                                |" << std::endl
                << "|   Limit switches configuration checking tool   |" << std::endl
                << "|                                                |" << std::endl
                << " ------------------------------------------------ " << std::endl
                << std::endl << LABEL_COLOR << "Version " << PENDULUM_CORE_VERSION << std::endl
                << C_RESET << std::endl
                << "This tool checks that the track limit switches are properly configured and working." <<
                std::endl
                << std::endl;

        const auto make_switch = [](const SwitchConfig &cfg) {
            return std::make_unique<pc::Switch>(
                cfg.pin, cfg.normally_up, cfg.use_internal_pull_resistor, cfg.debounce_us
            );
        };
        const auto left_switch = make_switch(config.left_switch);
        const auto right_switch = make_switch(config.right_switch);

        std::cout << HI << "Configuration :" << C_RESET << std::endl;
        std::cout << LABEL_COLOR << "- left switch  : " << C_RESET << "GPIO " << left_switch->pin() << std::endl;
        std::cout << LABEL_COLOR << "- right switch : " << C_RESET << "GPIO " << right_switch->pin() << std::endl;

        std::cout << std::endl
                << "If the wiring and configuration are OK, the switches must be shown as active" << std::endl
                << "when a magnet is moved close to them." << std::endl
                << std::endl;

        while (true) {
            constexpr auto POLLING_PERIOD_MS = 100;
            std::cout << '\r';
            display_state("left", !left_switch->at_rest());
            display_state("right", !right_switch->at_rest());
            std::cout.flush();

            std::this_thread::sleep_for(std::chrono::milliseconds(POLLING_PERIOD_MS));
        }
    } catch (const pigpio::ActivationToken::PleaseStop &) {
        std::cout << std::endl << C_YELLOW << "Termination signal received, terminating..." << std::endl;
    }

    std::cout << C_RESET;
    return EXIT_SUCCESS;
}

static int list_gpios(const PendulumConfig &config) {
    std::cout << C_HEADER << "GPIOs used by the process :" << C_RESET << std::endl;
    const auto mapping = get_GPIO_mapping(config);

    const auto print_gpio_usage = [](const GPIOPin &gpio) {
        std::cout << "- GPIO " << std::setw(2) << gpio.pin << " : " << gpio.label << std::endl;
    };

    print_gpio_usage(mapping.left_switch);
    print_gpio_usage(mapping.right_switch);

    return EXIT_SUCCESS;
}


int main(const int argc, char **argv) {
    int exit_code = EXIT_SUCCESS;
    const auto cli_options = parse_cli(
        {
            .description = "Limit switches configuration checking tool\n\n"
            "Must be run as root (needed by pigpio).",
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

        return run(config);
    } catch (const yamlcfg::InvalidConfiguration &e) {
        std::cerr << C_ERROR << "ERROR: Invalid configuration\n" << e.what() << C_RESET << std::endl;
        return EXIT_FAILURE;
    }
}
