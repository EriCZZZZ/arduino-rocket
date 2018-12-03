#ifndef __PT_TIMER_ARDUINO_H__
#define __PT_TIMER_ARDUION_H__

#include "pt.h"

typedef unsigned long pta_timer;

#define PT_DELAY_MILLIS(pt, timer, milliseconds) \
        *timer = millis(); \
        PT_WAIT_UNTIL(pt, millis() - *timer > milliseconds);

#define PT_WAIT_UNTIL_OR_TIMEOUT(pt, condition, timer, milliseconds) \
        *timer = millis(); \
        PT_WAIT_UNTIL(pt, (millis() - *timer > milliseconds) || (condition))

#endif
