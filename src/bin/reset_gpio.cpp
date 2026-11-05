#include <pigpio.h>
#include <iostream>

int main() {
    const auto rc = gpioInitialise();
    if (rc < 0) {
        std::cerr << "gpioInitialise failed with rc=" << rc << std::endl;
        return 1;
    }

    for (int i = 0; i <= 26; i++) {
        gpioSetMode(i, PI_INPUT);
        gpioSetPullUpDown(i, PI_PUD_OFF);
    }

    gpioTerminate();
    return 0;
}
