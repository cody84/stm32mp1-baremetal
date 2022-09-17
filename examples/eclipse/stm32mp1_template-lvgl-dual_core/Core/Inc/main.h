#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include "secondary_core.h"

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#define LED_RED_PORT GPIOA
#define LED_RED_PIN GPIO_PIN_13
#define LED_GREEN_PORT GPIOA
#define LED_GREEN_PIN GPIO_PIN_14
#define TP_INT_PIN GPIO_PIN_2
#define TP_INT_PORT GPIOF
#define I2C1_SCL_PIN GPIO_PIN_12
#define I2C1_SCL_PORT GPIOD
#define I2C1_SDA_PIN GPIO_PIN_15
#define I2C1_SDA_PORT GPIOF

#endif /* MAIN_H_ */
