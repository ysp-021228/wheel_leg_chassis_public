#include "error.h"

static uint32_t errors = 0;

void set_error(enum ErrorType error, bool is_error) {
  if (is_error) {
    errors |= (1 << error);
  } else {
    errors &= ~(1 << error);
  }
}

uint32_t get_errors() {
  return errors;
}