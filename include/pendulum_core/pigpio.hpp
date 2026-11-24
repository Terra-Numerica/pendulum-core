/** @file pigpio.hpp
  * @brief Header containing utilities to extend pigpio C++ API.
  */
#pragma once

#include <stdexcept>
#include <map>
#include <pigpio.h>


/// Runs a pigpio call and throws a pigpio::Exception if it returns a negative error code.
/** The macro evaluates to the value returned by the call, so that you can write:
  * ```
  * PIGPIO_CHECKED_CALL(gpioSetMode(10, PI_OUTPUT));
  * const int level = PIGPIO_CHECKED_CALL(gpioRead(10));
  * ```
  * The text of the call is used in the message of the exception.
  * @param call the pigpio function call, with its arguments.
  * @see pigpio::check() for the function this macro relies on.
  */
#define PIGPIO_CHECKED_CALL(call) pigpio::check((call), #call)


/// Namespace grouping the "extra" pigpio functionalities.
namespace pigpio {
    /// Simple exception wrapper over pigpio's error codes.
    class Exception : public std::runtime_error {
    public:
        /// Initialize the exception given the function and the error code.
        Exception(
            const std::string &caller,
            int return_code
        );

        /// Returns a human-readable description of the given error code.
        /** @param error_code an error code that can be produced by one of pigpio's
          *   functions.
          * @return The human-readable correspondant of error_code, or a dummy text
          *   if the code is unknown.
          */
        static std::string error2msg(int error_code);

    private:
        /// Auxiliary static method that creates exception messages.
        /** @param caller the name of the pigpio function that caused the error.
          * @param return_code the (negative) value returned by the pigpio function.
          * @return a human-readable string in which the error code is replaced by
          *   its name as defined in
          *   <a href="https://github.com/joan2937/pigpio/blob/db1fd9cf6c3431314b04edc486d900151a0dfd78/pigpio.h#L6389">pigpio's header.</a>
          */
        static std::string make_message(
            const std::string &caller,
            int return_code
        );

        /// Map from a pigpio's error code to its human-readable name.
        static const std::map<int, std::string> ERROR_CODES;
    };

    /// Throws a pigpio::Exception if a pigpio function returned a negative error code.
    /** @param return_code the value returned by the pigpio function.
      * @param what text describing the call that returned return_code, used in the message of the exception.
      * @return return_code, if it is not negative.
      */
    inline int check(const int return_code, const char *what) {
        if (return_code < 0) {
            throw Exception(what, return_code);
        }
        return return_code;
    }

    /// Utility class that activates and deactivates the pigpio library.
    /** This class follows the RAII idiom (the C++ counterpart of a Python context manager): the library is
      * initialized by the constructor, which calls gpioInitialise(), and the destructor puts all the GPIO pins in
      * high impedance mode when the object goes out of scope, even if an exception is thrown in the meantime.
      * Only one instance can exist at a time. Usage:
      * ```
      * int main() {
      *   // code that does not need pigpio
      *   const pigpio::ActivationToken token;
      *   // execute your pigpio-based instructions here!
      *   return 0;
      * }
      * ```
      * Termination requests (SIGINT, SIGTERM) are not handled by this class, see pendulum_core::termination.
      */
    class ActivationToken {
    public:
        /// Forwards a call to gpioInitialise().
        /** The constructor also registers abort() as a handler for SIGABRT (to be executed upon "catastrophic
          * failure").
          */
        ActivationToken();

        /// Ensures that all GPIO pins are changed to high impedance mode.
        ~ActivationToken();

        ActivationToken(const ActivationToken &) = delete;
        ActivationToken &operator=(const ActivationToken &) = delete;

        /// Auxiliary exception class to be thrown when multiple tokens are generated.
        class MultipleTokensCreated : public std::runtime_error {
        public:
            /// Fill-in the exception message.
            /** @param during_ctor If true, it means that this exception is being
              *   thrown from the constructor of ActivationToken. If false, it is
              *   being thrown from the destructor instead. It is merely used to
              *   properly fill the exception message.
              */
            explicit MultipleTokensCreated(
                const bool during_ctor
            )
                : std::runtime_error(
                    during_ctor
                        ? "Attempting to create multiple ActivationToken instances. This is forbidden, sorry."
                        : "While destroying an ActivationToken, another registered token was found. "
                        "This is a bug, please contact the maintainer."
                ) {
            }
        };

    private:
        static ActivationToken *active_token; ///< Pointer to the currently active token.

        /// Puts all GPIO pins in high impedance mode.
        /** This method allows to "disconnect" all GPIO pins, by putting them into
          * high impedance mode (inputs with no pull-up/pull-down resistors).
          * @note `gpioTerminate()` is not called, since doing it while handling a signal seems to kill the process.
          */
        static void reset_pins();

        /// Static method that can be registered as handler for SIGABRT.
        /** This method calls .
          * @param sig signal to be handled. Ignored in this method, but required
          *   by the syntax of std::signal.
          */
        static void abort(int sig);
    };


    /// Class that allows to execute instructions at a fixed rate.
    /** This class is similar to
      * <a href="http://wiki.ros.org/roscpp/Overview/Time#Sleeping_and_Rates">ROS Rate objects</a>.
      * Consider the snippet below:
      *
      * ```c++
      * const unsigned int micros = 100000; // 100ms
      * pigpio::Rate rate(micros);
      * while(true) {
      *   unsigned int now = rate.sleep();
      *   std::cout << "Current time: " << now << "us" << std::endl;
      *   // other instructions here
      * }
      * ```
      *
      * Every time the loop reaches the sleep instruction, the Rate object will
      * sleep for the amount of time required to obtain the given loop-period
      * (100 milliseconds in the example). The interesting fact is that the sleep
      * time is "adaptive": if the instructions following the Rate::sleep() call
      * require, *e.g.*, 50ms to execute, the thread would be paused for 50ms
      * when calling sleep(). If the instructions require 80ms, the thread would
      * sleep instead for just 20ms. In other words, the first instruction after
      * the sleep() call should be executed at a fixed rate. In the example above,
      * you should be able to see an output such as:
      * ```
      * Current time: 492306565
      * Current time: 492406565
      * Current time: 492506565
      * Current time: 492606565
      * Current time: 492706565
      * Current time: 492806565
      * Current time: 492906565
      * Current time: 493006565
      * Current time: 493106565
      * ```
      *
      * Of course, for this to work properly, the total time required to execute
      * the instructions after Rate::sleep() must be lower than a Rate's period!
      */
    class Rate {
    public:
        /// Create a rate of given period.
        /** @param period_us desired period in microseconds.
          */
        explicit Rate(unsigned int period_us);

        /// Sleep for the required time.
        /** @return Clock time in microseconds when the thread "wakes up".
          */
        unsigned int sleep();

        /// Time that remains in the current cycle.
        /** @return Time in microseconds that the timer would sleep at the next call
          *   to `sleep()`. It is just an estimation, the actual time might be
          *   slightly different.
          */
        [[nodiscard]] unsigned int residual() const;

        /// Get the last "wake up" time.
        /** @return The last value returned by a call to sleep().
          */
        [[nodiscard]] const unsigned int &last_tick() const { return time_past_; }

        /// Get the period of this Rate object.
        /** @return Period in microseconds as passed to the constructor.
          */
        [[nodiscard]] const unsigned int &period() const { return period_; }

    private:
        const unsigned int period_; ///< Target period of this Rate.
        unsigned int time_now_; ///< Used inside sleep() to store the current clock time.
        unsigned int time_past_; ///< Clock time corresponding to the last time sleep() exited.
    };


    /// Simple timer that can be used to execute instructions at a given rate.
    /** This class is similar to Rate, the main difference being that it does not
      * block the main thread by sleeping. It is meant for executing instructions
      * at a rate that is lower than that of the control loop.
      */
    class Timer {
    public:
        /// Create a timer of given period.
        /** @param period_us desired period in microseconds.
          * @param auto_reset if false, the timer is not automatically reset upon
          *   calling expired().
          */
        explicit Timer(
            unsigned int period_us,
            bool auto_reset
        );

        /// Tells if the timer expired.
        /** This method checks if the timer has expired. If auto_reset=true was
          * passed to the constructor, a call to reset() is also performed. Otherwise,
          * the timer is not reset, meaning that further calls to expired() will
          * keep returning true. In this case, the timer has to be reset manually by
          * a call to reset().
          * @return true if the timer expired.
          */
        bool expired();

        /// Reset the timer so that it will expire again in the future.
        void reset();

    private:
        const unsigned int period_; ///< Target period of this Rate.
        const bool auto_reset_; ///< If true, this timer automatically resets upon expiration.
        unsigned int time_past_; ///< Clock time corresponding to the last time reset() was called.
    };
} // end of namespace pigpio
