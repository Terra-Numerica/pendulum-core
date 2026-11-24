/** @file termination.hpp
  * @brief Graceful termination request, triggered by SIGINT or SIGTERM.
  */
#pragma once

#include <csignal>


/// Namespace grouping the utilities for the graceful termination of the processes.
/** The signal handler only sets a flag. The code which can run for a long time (main loops, calibration, ...) is
  * responsible for polling requested() and for leaving cleanly when it is set. Typical usage:
  * ```
  * pendulum_core::termination::install_handlers();
  * while (!pendulum_core::termination::requested()) {
  *   // do the periodic work here
  * }
  * ```
  * Objects going out of scope on the normal exit path release their resources (see pigpio::Session).
  */
namespace pendulum_core::termination {
    namespace detail {
        /// Flag set by the signal handler (only a `volatile std::sig_atomic_t` can be safely written by a handler).
        inline volatile std::sig_atomic_t requested_flag = 0;

        /// Signal handler: it only sets the flag, the rest is done by the code polling it.
        inline void handler([[maybe_unused]] const int signum) { requested_flag = 1; }
    }

    /// Installs the handler asking for a termination when SIGINT (CTRL+C) or SIGTERM is received.
    /** It must be called after pigpio::Session creation, since gpioInitialise() installs its own handlers.
      */
    inline void install_handlers() {
        std::signal(SIGINT, detail::handler);
        std::signal(SIGTERM, detail::handler);
    }

    /// Tells if a termination has been requested.
    [[nodiscard]] inline bool requested() noexcept { return detail::requested_flag != 0; }
}
