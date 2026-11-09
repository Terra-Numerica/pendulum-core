#pragma once
#include <iostream>
#include "colors.h"

#ifdef PENDULE_PI_DEBUG_ENABLED
#define PENDULE_PI_DBG(x) {std::cout << GREEN << "[DEBUG] " << RESET << x << std::endl;}
#else
#define PENDULE_PI_DBG(x) {};
#endif

#define WARNING_BEGIN std::cout << YELLOW << "[WARN]" << RESET ;
#define WARNING_END std::cout << RESET << std::endl ;
