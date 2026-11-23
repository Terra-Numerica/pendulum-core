#pragma once
#include <iostream>
#include "ansi_term.h"

#ifdef PENDULUM_CORE_DEBUG_ENABLED
#define PENDULUM_CORE_DBG(x) {std::cout << C_GREEN << "[DEBUG] " << C_RESET << x << std::endl;}
#else
#define PENDULUM_CORE_DBG(x) {};
#endif

#define WARNING_BEGIN std::cout << C_YELLOW << "[WARN]" << C_RESET ;
#define WARNING_END std::cout << C_RESET << std::endl ;
