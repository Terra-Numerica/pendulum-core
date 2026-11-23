#include "pendulum_core/encoder.hpp"
#include "pendulum_core/pigpio.hpp"
#include "pendulum_core/debug.hpp"
#include <pigpio.h>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <thread>


namespace pendulum_core {
    namespace {
        /// Time given to the lines to reach their level after the pull-up resistors are enabled.
        constexpr auto PULL_UP_SETTLING_TIME = std::chrono::milliseconds(10);
    }


    int Encoder::encode(
        const bool past_a,
        const bool past_b,
        const bool a,
        const bool b
    ) {
        return (past_a << 3) | (past_b << 2) | (a << 1) | b;
    }


    Encoder::Encoder(
        const int gpio_a,
        const int gpio_b,
        const int steps_per_turn,
        const unsigned int glitch_filter_us
    )
        : pin_a_(gpio_a)
          , pin_b_(gpio_b)
          , a_current_(false)
          , b_current_(false)
          , a_past_(false)
          , b_past_(false)
          , steps_(0)
          , lower_threshold_(0)
          , upper_threshold_(0)
          , lower_cb_(nullptr)
          , upper_cb_(nullptr)
          , steps_per_turn_(steps_per_turn)
          , counting_multiplier_(DEFAULT_COUNTING_MULTIPLIER) {
        if (steps_per_turn <= 0) {
            throw std::invalid_argument("Encoder: steps_per_turn must be positive");
        }
        PENDULUM_CORE_DBG("Creating Encoder on pins " << pin_a_ << " and " << pin_b_);
        // Configure the pins using pigpio
        PIGPIO_CHECKED_CALL(gpioSetMode(pin_a_, PI_INPUT));
        PIGPIO_CHECKED_CALL(gpioSetMode(pin_b_, PI_INPUT));

        // This assumes that the phases are common grounded.
        PIGPIO_CHECKED_CALL(gpioSetPullUpDown(pin_a_, PI_PUD_UP));
        PIGPIO_CHECKED_CALL(gpioSetPullUpDown(pin_b_, PI_PUD_UP));

        // With open-collector outputs, the internal pull-up resistor is the only thing raising the lines, and it
        // does so slowly (resistance combined with the cable capacitance). Reading the levels too early would give
        // a wrong initial state, and the lines reaching their actual level afterwards would be counted as steps.
        std::this_thread::sleep_for(PULL_UP_SETTLING_TIME);

        // Initialize properly the pin reading, before the interrupts are enabled so that the initial levels are
        // known when the first one is handled.
        a_current_ = a_past_ = PIGPIO_CHECKED_CALL(gpioRead(pin_a_));
        b_current_ = b_past_ = PIGPIO_CHECKED_CALL(gpioRead(pin_b_));
        PENDULUM_CORE_DBG("Encoder initial levels: A=" << a_current_ << " B=" << b_current_);

        // Ignore the level changes shorter than the requested delay (electrical noise)
        if (glitch_filter_us > 0) {
            PIGPIO_CHECKED_CALL(gpioGlitchFilter(pin_a_, glitch_filter_us));
            PIGPIO_CHECKED_CALL(gpioGlitchFilter(pin_b_, glitch_filter_us));
        }

        // Setup interrupts so that we handle the rotation without polling.
        PIGPIO_CHECKED_CALL(gpioSetAlertFuncEx(pin_a_, pulse_static, this));
        PIGPIO_CHECKED_CALL(gpioSetAlertFuncEx(pin_b_, pulse_static, this));
        PENDULUM_CORE_DBG("Encoder created successfully");
    }


    Encoder::~Encoder() {
        PENDULUM_CORE_DBG("Destroying Encoder on pins " << pin_a_ << " and " << pin_b_);
        // Disable interrupts
        gpioSetAlertFuncEx(pin_a_, nullptr, nullptr);
        gpioSetAlertFuncEx(pin_b_, nullptr, nullptr);
        // Set all pins in high impedance, just in case
        gpioSetPullUpDown(pin_a_, PI_PUD_OFF);
        gpioSetPullUpDown(pin_b_, PI_PUD_OFF);
        gpioSetMode(pin_a_, PI_INPUT);
        gpioSetMode(pin_b_, PI_INPUT);
        PENDULUM_CORE_DBG("Encoder destroyed successfully");
    }


    void Encoder::set_safety_callbacks(
        const int lower_threshold,
        const int upper_threshold,
        const std::function<void()> &lower_cb,
        const std::function<void()> &upper_cb
    ) {
        lower_threshold_ = lower_threshold;
        upper_threshold_ = upper_threshold;
        lower_cb_ = lower_cb;
        upper_cb_ = upper_cb;
    }


    void Encoder::remove_safety_callbacks() {
        lower_cb_ = nullptr;
        upper_cb_ = nullptr;
        lower_threshold_ = std::numeric_limits<int>::min();
        upper_threshold_ = std::numeric_limits<int>::max();
    }


    void Encoder::pulse(const int gpio, const int level, unsigned int/*tick*/) {
        // This is more for debug purposes. I guess this condition should never pass.
        if (gpio != pin_a_ && gpio != pin_b_) {
            throw std::runtime_error(
                "Expected Encoder interrupt on pins " + std::to_string(pin_a_) + " or "
                + std::to_string(pin_b_) + ", however an interrupt was fired for pin "
                + std::to_string(gpio)
            );
        }

        // pigpio may report the initial level of a pin when the interrupt is enabled: this is not an edge, and
        // counting it would shift the step count depending on the initial state of the phases.
        if (static_cast<bool>(level) == (gpio == pin_a_ ? a_current_ : b_current_)) {
            return;
        }

        // "Shift in time" the pin states
        a_past_ = a_current_;
        b_past_ = b_current_;
        // update the pint state that caused the interrupt
        (gpio == pin_a_ ? a_current_ : b_current_) = level;

        // Update the current step count
        direction_ = ENCODER_TABLE[encode(a_past_, b_past_, a_current_, b_current_)];
        steps_ += direction_;

        // execute safety callbacks if needed
        if (steps_ <= lower_threshold_ && lower_cb_ != nullptr)
            lower_cb_();
        if (steps_ >= upper_threshold_ && upper_cb_ != nullptr)
            upper_cb_();
    }


    void Encoder::pulse_static(
        const int gpio,
        const int level,
        const unsigned int tick,
        void *THIS
    ) {
        static_cast<Encoder *>(THIS)->pulse(gpio, level, tick);
    }
}
