// clang-format off
#include <iomanip>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include <pendule_pi/pendule_cpp_client.hpp>
#include <pendule_pi/config.hpp>
#include <pendule_pi/colors.h>
// clang-format on

// main forever loop termination flag
static bool keep_running{true};

static void requestTermination([[maybe_unused]] int signum) { keep_running = false; }

constexpr auto POS_COLOR = BRIGHT_GREEN;
constexpr auto NEG_COLOR = BRIGHT_RED;
constexpr auto LABEL_COLOR = BRIGHT_BLUE;
constexpr auto HI = BRIGHT_WHITE;


static void display_value(const std::string &label, const int value, const int last_value) {
    constexpr auto STATIC_COLOR = RESET;
    std::cout << STATIC_COLOR << "[ " << label << " : " << RESET;
    if (value > last_value)
        std::cout << POS_COLOR;
    else if (value < last_value)
        std::cout << NEG_COLOR;
    else
        std::cout << RESET;
    std::cout << std::setw(6) << std::right << value << STATIC_COLOR << " ]  " << RESET;
}

int main(const int argc, char **argv) {
    namespace pp = pendule_pi;
    const std::string program_name = argv[0];

    // Load all parameters from a configuration file.
    if (argc < 2) {
        std::cout << RED << "Usage: sudo " << argv[0] << " config_file.yaml" << RESET << std::endl;
        std::cout << "- 'sudo' is needed for pigpio to work" << std::endl;
        std::cout << "- the configuration file is needed to setup the interface" << std::endl;
        return EXIT_FAILURE;
    }

    pp::Config config(argv[1]);

    try {
        // Let the token manage the pigpio library!
        pigpio::ActivationToken token;

        signal(SIGINT, pigpio::ActivationToken::pleaseStop);
        signal(SIGTERM, pigpio::ActivationToken::pleaseStop);

        std::cout
                << HI
                << " ------------------------------------------ " << std::endl
                << "|                                          |" << std::endl
                << "|   Encoders configuration checking tool   |" << std::endl
                << "|                                          |" << std::endl
                << " ------------------------------------------ " << std::endl
                << RESET << std::endl
                << "This tool checks that the encoders are configured so that the returned values" << std::endl
                << "progress according to the conventions used in the control laws." << std::endl << std::endl;

        const auto position_encoder = config.getPositionEncoder();
        const auto angle_encoder = config.getAngleEncoder();
        constexpr auto POLLING_PERIOD_MS = 100;

        std::cout << HI << "Configuration :" << RESET << std::endl;
        std::cout << LABEL_COLOR << "- position : " << RESET
                << "A -> GPIO " << position_encoder->getPinA() << " - B -> GPIO " << position_encoder->getPinB()
                << std::endl;
        std::cout << LABEL_COLOR << "- angle    : " << RESET
                << "A -> GPIO " << angle_encoder->getPinA() << " - B -> GPIO " << angle_encoder->getPinB()
                << std::endl;

        std::cout << std::endl
                << "If the wiring and configuration are OK, the following behaviours are expected :" << std::endl
                << "- position values " << HI << "increase" << RESET << " when the cart moves " << HI << "rightwards" <<
                LABEL_COLOR << " (positive X)" << RESET << std::endl
                << "- angle values " << HI << "increase" << RESET << " when the arm is rotated " << HI << "CCW" <<
                LABEL_COLOR << " (trigonometric convention)" << RESET << std::endl
                << std::endl
                << "Reading encoders every " << POLLING_PERIOD_MS << " ms... "
                << LABEL_COLOR << "(Ctrl-C to end)" << RESET << std::endl
                << std::endl;

        // Connect signals for graceful termination.
        std::signal(SIGINT, requestTermination);
        std::signal(SIGTERM, requestTermination);

        auto last_position = position_encoder->steps();
        auto last_angle = angle_encoder->steps();

        while (keep_running) {
            const auto position = position_encoder->steps();
            const auto angle = angle_encoder->steps();

            std::cout << '\r';
            display_value("position", position, last_position);
            display_value("angle", angle, last_angle);
            std::cout
                    << LABEL_COLOR << "(legend: "
                    << POS_COLOR << "INC" << RESET
                    << " "
                    << NEG_COLOR << "DEC" << LABEL_COLOR << ")  " << RESET;
            std::cout.flush();

            last_position = position;
            last_angle = angle;

            std::this_thread::sleep_for(std::chrono::milliseconds(POLLING_PERIOD_MS));
        }
    } catch (const pigpio::ActivationToken::PleaseStop &) {
        std::cout << std::endl << "PleaseStop token received, terminating..." << std::endl;
    } catch (...) {
        throw;
    }

    std::cout << RESET << std::endl << std::endl;

    return EXIT_SUCCESS;
}
