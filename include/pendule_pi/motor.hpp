/** @file motor.hpp
  * @brief Header file for the Motor class.
  */
#pragma once

#include <atomic>

namespace pendule_pi {
    /// Class that handles a simple DC motor.
    class Motor {
    public:
        /// Constructor.
        /** @param pwm_pin Pin used to control the force of the motor via PWM.
          * @param direction_pin Pin used to control the direction of the motor.
          */
        Motor(
            int pwm_pin,
            int direction_pin
        );

        // Prevent the user from making copies of a Motor.
        Motor(const Motor &) = delete;

        Motor &operator=(const Motor &) = delete;

        /// Destructor.
        virtual ~Motor();

        /// Sets the pwm of the motor.
        /** @param pwm the (signed) pwm, between -255 and 255 (both included).
          */
        void set_pwm(
            int pwm
        );

        /// Get the current PWM value.
        [[nodiscard]] int pwm() const noexcept { return pwm_; }

        /// read access to pins
        [[nodiscard]] int direction_pin() const { return dir_pin_; };
        [[nodiscard]] int pwm_pin() const { return pwm_pin_; };

    private:
        int pwm_pin_; ///< Pin used to control the force of the motor via PWM.
        int dir_pin_; ///< Pin used to control the direction of the motor.
        std::atomic<int> pwm_; ///< Current PWM applied to pwm_pin_ (read by other threads than the one setting it).
    };
}
