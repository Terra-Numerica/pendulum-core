#pragma once

#define CSI "\x1b["
#define C_RED CSI"0;31m"
#define C_GREEN CSI"0;32m"
#define C_YELLOW CSI"0;33m"
#define C_BLUE CSI"0;34m"
#define C_MAGENTA CSI"0;35m"
#define C_CYAN CSI"0;36m"
#define C_WHITE CSI"0;37m"

#define C_BR_RED CSI"0;91m"
#define C_BR_GREEN CSI"0;92m"
#define C_BR_YELLOW CSI"0;93m"
#define C_BR_BLUE CSI"0;94m"
#define C_BR_MAGENTA CSI"0;95m"
#define C_BR_CYAN CSI"0;96m"
#define C_BR_WHITE CSI"0;97m"

#define C_RESET CSI"0m"

#define C_ERROR C_BR_RED
#define C_SUCCESS C_BR_GREEN
#define C_WARNING C_BR_YELLOW
#define C_INFO C_BR_CYAN
#define C_HEADER C_BR_WHITE
#define C_MUTED C_BLUE

#define CURSOR_OFF CSI"?25l"
#define CURSOR_ON CSI"?25h"

#define CLR_EOL CSI"K"

#define HIDE_CTRL_C "\b\b" CLR_EOL
