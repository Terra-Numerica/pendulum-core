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
#include <pendulum_core/colors.h>
#include <pendulum_core/pendulum_config.hpp>
#include <pendulum_core/pigpio.hpp>
#include <pendulum_core/encoder.hpp>
// clang-format on

constexpr auto POS_COLOR = C_BR_GREEN;
constexpr auto NEG_COLOR = C_BR_RED;
constexpr auto LABEL_COLOR = C_BR_BLUE;


/// Displays the raw step count (colored according to its evolution) and the corresponding physical value.
static void display_value(
    const std::string &label, const int raw_steps, const int last_raw_steps,
    const double physical_value, const std::string &unit
) {
    constexpr auto STATIC_COLOR = C_RESET;
    std::cout << STATIC_COLOR << "[ " << label << " : " << C_RESET;
    if (raw_steps > last_raw_steps)
        std::cout << POS_COLOR;
    else if (raw_steps < last_raw_steps)
        std::cout << NEG_COLOR;
    else
        std::cout << C_RESET;
    std::cout << std::setw(6) << std::right << raw_steps << " steps" << STATIC_COLOR
            << " | " << std::fixed << std::setprecision(3) << std::setw(8) << physical_value << " " << unit
            << " ]  " << C_RESET;
}

static int run(const PendulumConfig &config) {
    namespace pc = pendulum_core;

    try {
        pigpio::ActivationToken token;

        signal(SIGINT, pigpio::ActivationToken::please_stop);
        signal(SIGTERM, pigpio::ActivationToken::please_stop);

        std::cout
                << C_HEADER
                << " ------------------------------------------ " << std::endl
                << "|                                          |" << std::endl
                << "|   Encoders configuration checking tool   |" << std::endl
                << "|                                          |" << std::endl
                << " ------------------------------------------ " << std::endl
                << std::endl << LABEL_COLOR << "Version " << PENDULUM_CORE_VERSION << std::endl
                << C_RESET << std::endl
                << "This tool checks that the encoders are configured so that the returned values" << std::endl
                << "progress according to the conventions used in the control laws." << std::endl << std::endl;

        const auto &position_cfg = config.position_encoder;
        const auto &angle_cfg = config.angle_encoder;
        const auto position_encoder = std::make_unique<pc::PositionEncoder>(
            position_cfg.pin_a, position_cfg.pin_b, position_cfg.steps_per_turn,
            position_cfg.belt_module_mm, position_cfg.pulley_teeth_count, position_cfg.glitch_filter_us
        );
        const auto angle_encoder = std::make_unique<pc::AngleEncoder>(
            angle_cfg.pin_a, angle_cfg.pin_b, angle_cfg.steps_per_turn, angle_cfg.glitch_filter_us
        );
        constexpr auto POLLING_PERIOD_MS = 100;

        std::cout << C_HEADER << "Configuration :" << C_RESET << std::endl;
        std::cout << LABEL_COLOR << "- position : " << C_RESET
                << "A->GPIO " << position_encoder->pin_a() << " - B->GPIO " << position_encoder->pin_b()
                << std::endl;
        std::cout << LABEL_COLOR << "- angle    : " << C_RESET
                << "A->GPIO " << angle_encoder->pin_a() << " - B->GPIO " << angle_encoder->pin_b()
                << std::endl;

        std::cout << std::endl
                << "If the wiring and configuration are OK, the following behaviours are expected :" << std::endl
                << "- position values " << C_HEADER << "increase" << C_RESET << " when the cart moves " << C_HEADER <<
                "rightwards" <<
                LABEL_COLOR << " (positive X)" << C_RESET << std::endl
                << "- angle values " << C_HEADER << "increase" << C_RESET << " when the arm is rotated " << C_HEADER <<
                "CCW"
                <<
                LABEL_COLOR << " (trigonometric convention)" << C_RESET << std::endl
                << std::endl
                << "Reading encoders every " << POLLING_PERIOD_MS << " ms... "
                << LABEL_COLOR << "(Ctrl-C to end)" << C_RESET << std::endl
                << std::endl;

        // values are relative to the encoders position at startup
        auto last_position = position_encoder->raw_steps();
        auto last_angle = angle_encoder->raw_steps();

        while (true) {
            const auto position = position_encoder->raw_steps();
            const auto angle = angle_encoder->raw_steps();

            std::cout << '\r';
            display_value("position", position, last_position, position_encoder->position(), "m");
            display_value("angle", angle, last_angle, angle_encoder->angle(), "rad");
            std::cout
                    << LABEL_COLOR << "(legend: "
                    << POS_COLOR << "INC" << C_RESET
                    << " "
                    << NEG_COLOR << "DEC" << LABEL_COLOR << ")  " << C_RESET;
            std::cout.flush();

            last_position = position;
            last_angle = angle;

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

    print_gpio_usage(mapping.position_encoder_A);
    print_gpio_usage(mapping.position_encoder_B);
    print_gpio_usage(mapping.angle_encoder_A);
    print_gpio_usage(mapping.angle_encoder_B);

    return EXIT_SUCCESS;
}


int main(const int argc, char **argv) {
    int exit_code = EXIT_SUCCESS;
    const auto cli_options = parse_cli(
        {
            .description = "Encoders configuration checking tool\n\n"
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
