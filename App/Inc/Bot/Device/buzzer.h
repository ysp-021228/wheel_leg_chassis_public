#ifndef BUZZER_H
#define BUZZER_H

#include "stdint-gcc.h"

extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);

#endif //BUZZER_H
