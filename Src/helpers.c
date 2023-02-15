/**
 * @file helpers.c
 * @author Rich Baird (rich.baird@utah.edu)
 * @brief The implementation of the helper functions declared in helpers.h
 * @version 0.1
 * @date 2023-02-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "helpers.h"
#include <stdarg.h>
void debug_exec(void (*fn)(void *), ...) {
#ifdef DBG
  va_list args;
  va_start(args, fn);

  while (fn != NULL) {
    void *arg = va_arg(args, void *);
    (*fn)(arg);

    fn = va_arg(args, void (*)(void *));
  }

  va_end(args);
#endif
}