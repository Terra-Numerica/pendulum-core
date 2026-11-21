#pragma once
#include <iostream>
#include "colors.h"

#ifdef PENDULE_PI_DEBUG_ENABLED
#define PENDULE_PI_DBG(x) {std::cout << C_GREEN << "[DEBUG] " << C_RESET << x << std::endl;}
#else
#define PENDULE_PI_DBG(x) {};
#endif

#define WARNING_BEGIN std::cout << C_YELLOW << "[WARN]" << C_RESET ;
#define WARNING_END std::cout << C_RESET << std::endl ;
