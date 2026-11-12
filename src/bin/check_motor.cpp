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


constexpr auto ANOM_COLOR = BRIGHT_RED;
constexpr auto SUGG_COLOR = BRIGHT_GREEN;
constexpr auto LABEL_COLOR = BRIGHT_BLUE;
constexpr auto HI = BRIGHT_WHITE;


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

    const auto config = pp::Config::loadFromFile(argv[1]);
    if (!config) {
        return EXIT_FAILURE;
    }

    // Let the token manage the pigpio library!
    pigpio::ActivationToken token;

    std::cout
            << HI
            << " ----------------------------------------- " << std::endl
            << "|                                         |" << std::endl
            << "|       Motor control checking tool       |" << std::endl
            << "|                                         |" << std::endl
            << " ----------------------------------------- " << std::endl
            << RESET << std::endl
            << "This tool checks that the motor is working and wired the right way." << std::endl
            << std::endl;

    const auto motor = config->getMotor();

    std::cout << HI << "Configuration :" << RESET << std::endl;
    std::cout << LABEL_COLOR << "- direction GPIO : " << RESET << motor->getDirectionPin() << std::endl;
    std::cout << LABEL_COLOR << "- PWM GPIO       : " << RESET << motor->getPWMPin() << std::endl;

    std::cout << std::endl
            << "The motor will be moved in both directions to check its configuration." << std::endl << std::endl
            << HI << "Move the cart to a central position, then hit Enter." << RESET <<
            std::endl;

    std::string line;
    std::getline(std::cin, line);

    constexpr auto TEST_PWM = 50;
    constexpr auto MOTION_DURATION_SECS = 1;

    std::cout << LABEL_COLOR << "-> cart moving rightwards for " << MOTION_DURATION_SECS << " seconds..."
            << RESET << std::endl;
    motor->setPWM(TEST_PWM);
    std::this_thread::sleep_for(std::chrono::milliseconds(MOTION_DURATION_SECS * 1000));
    motor->setPWM(0);

    std::cout << LABEL_COLOR << "-> cart moving leftwards for " << 2 * MOTION_DURATION_SECS << " seconds..."
            << RESET << std::endl;
    motor->setPWM(-TEST_PWM);
    std::this_thread::sleep_for(std::chrono::milliseconds(2 * MOTION_DURATION_SECS * 1000));
    motor->setPWM(0);


    std::cout << LABEL_COLOR << "-> cart going back to start position..." << RESET << std::endl;
    motor->setPWM(TEST_PWM);
    std::this_thread::sleep_for(std::chrono::milliseconds(MOTION_DURATION_SECS * 1000));
    motor->setPWM(0);

    std::cout << std::endl
            << "The sequence should have been : right, left, back to center." << std::endl << std::endl
            << HI << "Troubleshooting suggestions if it didn't go well :" << RESET << std::endl
            << ANOM_COLOR << "- the motor moved, but in the wrong directions " << RESET << std::endl
            << SUGG_COLOR << "  => swap the wires on the driver output" << RESET << std::endl
            << ANOM_COLOR << "- the motor didn't move at all" << RESET << std::endl
            << SUGG_COLOR << "  => check that the driver is powered" << RESET << std::endl
            << ANOM_COLOR << "- the driver is powered but the motor didn't move" << RESET << std::endl
            << SUGG_COLOR << "  => check that the DIR/PWM signals are connected according to the configuration" <<
            RESET
            << std::endl;

    std::cout << RESET;
    return EXIT_SUCCESS;
}
