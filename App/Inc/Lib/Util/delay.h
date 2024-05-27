#ifndef DELAY_H
#define DELAY_H

#include "stdint-gcc.h"

extern void delay_init(void);
extern void delay_us(uint16_t nus);
extern void delay_ms(uint16_t nms);

#endif //DELAY_H
