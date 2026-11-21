// clang-format off
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <csignal>
#include <cstdlib>

#include <pendule_cli/cli_parser.hpp>
#include <pendule_pi/colors.h>
#include <pendule_pi/pendulum_config.hpp>
#include <pendule_pi/pigpio.hpp>
#include <pendule_pi/motor.hpp>
// clang-format on


constexpr auto ANOM_COLOR = C_BR_CYAN;
constexpr auto ACTION_COLOR = C_BR_GREEN;
constexpr auto LABEL_COLOR = C_BR_BLUE;


static int run(const PendulumConfig &config) {
    namespace pp = pendule_pi;

    pigpio::ActivationToken token;

    std::cout
            << C_HEADER
            << " ----------------------------------------- " << std::endl
            << "|                                         |" << std::endl
            << "|       Motor control checking tool       |" << std::endl
            << "|                                         |" << std::endl
            << " ----------------------------------------- " << std::endl
            << std::endl << LABEL_COLOR << "Version " << PENDULE_PI_VERSION << std::endl
            << C_RESET << std::endl
            << "This tool checks that the motor is working and wired the right way." << std::endl
            << std::endl;

    const auto motor = std::make_unique<pp::Motor>(config.motor.pwm_pin, config.motor.direction_pin);

    std::cout << C_HEADER << "Configuration :" << C_RESET << std::endl;
    std::cout << LABEL_COLOR << "- direction GPIO : " << C_RESET << motor->getDirectionPin() << std::endl;
    std::cout << LABEL_COLOR << "- PWM GPIO       : " << C_RESET << motor->getPWMPin() << std::endl;

    std::cout << std::endl
            << "The motor will be moved in both directions to check its configuration." << std::endl << std::endl
            << C_HEADER << "Move the cart to a central position, then hit Enter." << C_RESET <<
            std::endl;

    std::string line;
    std::getline(std::cin, line);

    constexpr auto TEST_PWM = 50;
    constexpr auto MOTION_DURATION_SECS = 2;

    std::cout << LABEL_COLOR << "-> cart moving rightwards for " << MOTION_DURATION_SECS << " seconds..."
            << C_RESET << std::endl;
    motor->setPWM(TEST_PWM);
    std::this_thread::sleep_for(std::chrono::milliseconds(MOTION_DURATION_SECS * 1000));
    motor->setPWM(0);

    std::cout << LABEL_COLOR << "-> cart moving leftwards for " << 2 * MOTION_DURATION_SECS << " seconds..."
            << C_RESET << std::endl;
    motor->setPWM(-TEST_PWM);
    std::this_thread::sleep_for(std::chrono::milliseconds(2 * MOTION_DURATION_SECS * 1000));
    motor->setPWM(0);


    std::cout << LABEL_COLOR << "-> cart going back to start position..." << C_RESET << std::endl;
    motor->setPWM(TEST_PWM);
    std::this_thread::sleep_for(std::chrono::milliseconds(MOTION_DURATION_SECS * 1000));
    motor->setPWM(0);

    std::cout << std::endl
            << "The sequence should have been : right, left, back to center." << std::endl << std::endl
            << C_HEADER << "Troubleshooting guide if it didn't go well :" << C_RESET << std::endl
            << ANOM_COLOR << "- the motor moved, but in the wrong directions " << C_RESET << std::endl
            << ACTION_COLOR << "  => swap the wires on the driver output" << C_RESET << std::endl
            << ANOM_COLOR << "- the motor didn't move at all" << C_RESET << std::endl
            << ACTION_COLOR << "  => check that the driver is powered" << C_RESET << std::endl
            << ANOM_COLOR << "- the driver is powered but the motor didn't move" << C_RESET << std::endl
            << ACTION_COLOR << "  => check that the DIR/PWM signals are connected according to the configuration" <<
            C_RESET
            << std::endl << std::endl;

    std::cout << C_RESET;
    return EXIT_SUCCESS;
}

static int list_gpios(const PendulumConfig &config) {
    std::cout << C_HEADER << "GPIOs used by the process :" << C_RESET << std::endl;
    const auto mapping = get_GPIO_mapping(config);

    const auto print_gpio_usage = [](const GPIOPin &gpio) {
        std::cout << "- GPIO " << std::setw(2) << gpio.pin << " : " << gpio.label << std::endl;
    };

    print_gpio_usage(mapping.motor_direction);
    print_gpio_usage(mapping.motor_pwm);

    return EXIT_SUCCESS;
}


int main(const int argc, char **argv) {
    int exit_code = EXIT_SUCCESS;
    const auto cli_options = parse_cli(
        {
            .description = "Motor control checking tool\n\n"
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
