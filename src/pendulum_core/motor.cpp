#include "pendulum_core/motor.hpp"
#include "pendulum_core/pigpio.hpp"
#include "pendulum_core/debug.hpp"
#include <pigpio.h>


namespace pendulum_core {
    Motor::Motor(
        const int pwm_pin,
        const int direction_pin
    )
        : pwm_pin_(pwm_pin)
          , dir_pin_(direction_pin)
          , pwm_(0) {
        PENDULUM_CORE_DBG("Creating Motor on pins " << pwm_pin_ << " and " << dir_pin_);
        // Set the direction pin in output mode. No need to do it with the PWM.
        PIGPIO_CHECKED_CALL(gpioSetMode(dir_pin_, PI_OUTPUT));
        // Make sure the PWM starts at zero.
        PIGPIO_CHECKED_CALL(gpioPWM(pwm_pin_, 0));
        PENDULUM_CORE_DBG("Motor created successfully");
    }


    Motor::~Motor() {
        PENDULUM_CORE_DBG("Destroying Motor connected to pins " << pwm_pin_ << " and " << dir_pin_);
        // write a zero on the speed pin
        gpioPWM(pwm_pin_, 0);
        // set all pins in high impedance, just in case
        gpioSetPullUpDown(pwm_pin_, PI_PUD_OFF);
        gpioSetPullUpDown(dir_pin_, PI_PUD_OFF);
        gpioSetMode(pwm_pin_, PI_INPUT);
        gpioSetMode(dir_pin_, PI_INPUT);
        PENDULUM_CORE_DBG("Motor destroyed successfully");
    }


    void Motor::set_pwm(
        const int pwm
    ) {
        if (pwm > 0) {
            PIGPIO_CHECKED_CALL(gpioWrite(dir_pin_, PI_HIGH));
            PIGPIO_CHECKED_CALL(gpioPWM(pwm_pin_, pwm));
        } else {
            PIGPIO_CHECKED_CALL(gpioWrite(dir_pin_, PI_LOW));
            PIGPIO_CHECKED_CALL(gpioPWM(pwm_pin_, -pwm));
        }
        pwm_ = pwm;
    }
}
