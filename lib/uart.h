
#ifndef __UART
#define __UART

#include "buffer.h"
#define TEMT (1<<6)
#define TREAT_AS_HEX 0

// void init_uart(Buffer* rxBuf);
void init_uart(void);

void write_uart_len(char* data, uint8_t len);

void write_uart(char* data);

#endif
