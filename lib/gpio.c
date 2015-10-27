
#include "inc/LPC11xx.h"
#include "inc/core_cm0.h"
#include "uart.h"

#include "gpio.h"

void gpio_set_mode(uint8_t port, uint8_t pin, uint8_t mode) {
//  probably a major pain to implement(see LPC11xx.h:199)
//  leave for other modules to modify pin modes as needed?
}

void set_bit(volatile int32_t* addr, uint8_t bit, uint8_t value) {
    if (value){
        *addr |= 1 << bit;
    } else {
        *addr &= ~(1 << bit);
    }
}

uint32_t get_bit(volatile int32_t* addr, uint8_t bit) {
    uint32_t addr_value = *addr;
    return (addr_value & (1 << bit)) >> 1;
}

void gpio_enable(uint8_t port, uint8_t pin, uint8_t direction) {
    LPC_GPIO_TypeDef* gport = gpio_port(port);
    if (gport) {

        set_bit(&gport->DIR, pin, direction);
    }
}

LPC_GPIO_TypeDef* gpio_port(uint8_t port) {
    switch (port) {
        case 0:
            return LPC_GPIO0;
            break;
        case 1:
            return LPC_GPIO1;
            break;
        default:
            return 0;
    }
}

static IntHandler gpioInt;

static IntHandler gpio0_handler;
static IntHandler gpio1_handler;

void PIOINT0_IRQHandler(void) {
    gpio0_handler();
    LPC_GPIO0->IC |= 0xfff;
    __NOP;
    __NOP;
}

void PIOINT1_IRQHandler(void) {
    gpio1_handler();
    LPC_GPIO1->IC |= 0xfff;
    __NOP;
    __NOP;
}


void gpio_setup_int(uint8_t port, uint8_t pin, uint8_t enable, IntHandler h) {
    LPC_GPIO_TypeDef* gport = gpio_port(port);
    if (gport) {
        set_bit(&gport->IE, pin, enable);
                
        switch (port) {
            case 0:
                NVIC_EnableIRQ(EINT0_IRQn); // enable GPIO0 interrupt    
                gpio0_handler = h;
                break;
            case 1:
                NVIC_EnableIRQ(EINT1_IRQn); // enable GPIO0 interrupt    
                gpio1_handler = h;
                write_uart("setup_port0 int");
                break;
            default:
                break;
        }
    }
}

int32_t gpio_read(uint8_t port, uint8_t pin) {
    LPC_GPIO_TypeDef* gport = gpio_port(port);
    int32_t result = -1;
    if (gport) {
        result = get_bit(&gport->DATA, pin);
    }
    return result;  
}

void gpio_write(uint8_t port, uint8_t pin, uint8_t value) {
LPC_GPIO_TypeDef* gport = gpio_port(port);
    if (gport && (get_bit(&gport->DIR, pin)))
    {
        // direction is output
        set_bit(&gport->DATA, pin, value);
    }
}

