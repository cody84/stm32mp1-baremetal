#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include "secondary_core.h"

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#define LED_RED_PORT GPIOA
#define LED_RED_PIN GPIO_PIN_13
#define LED_GREEN_PORT GPIOA
#define LED_GREEN_PIN GPIO_PIN_14

#endif /* MAIN_H_ */
