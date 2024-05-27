#ifndef ERROR_H
#define ERROR_H

#include <stdint-gcc.h>
#include <stdbool.h>

enum ErrorType{
  ERROR_JOINT_MOTOR_CONTROLLER_OFFLINE,
  ERROR_WHEEL_MOTOR_CONTROLLER_OFFLINE,
  ERROR_RC_OFFLINE,
};

void set_error(enum ErrorType error, bool is_error);

uint32_t get_errors();

#endif //ERROR_H
