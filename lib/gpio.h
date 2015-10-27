
#ifndef __GPIO
#define __GPIO

#include "inc/LPC11xx.h"

// #define GPORTN(x)  LPC_GPIO ## x 
// sets mode of pin number pin on port number port to passed mode value
// used for setting up alternative functions of periphs
void gpio_set_mode(uint8_t port, uint8_t pin, uint8_t mode);


LPC_GPIO_TypeDef* gpio_port(uint8_t port);

typedef void (*IntHandler)(void);

IntHandler gpio0_handler;
// enables gpio input/output functionality on port + pin 
// direction == 0: set as input
// direction /= 0: set as output
void gpio_enable(uint8_t port, uint8_t pin, uint8_t direction);

// setups handler as the interrupt handler on the port/pin pair
void gpio_setup_int(uint8_t port, uint8_t pin, uint8_t enable, void (*handler)(void));

// returns the value of pin # pin on port # port
// 0 if value is zero, nonzero if 1
int32_t gpio_read(uint8_t port, uint8_t pin);


void gpio_write(uint8_t port, uint8_t pin, uint8_t value);





#endif