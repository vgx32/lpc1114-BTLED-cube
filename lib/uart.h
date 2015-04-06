

#ifndef __UART
#define __UART

#include "buffer.h"
#define TEMT (1<<6)

// void init_uart(Buffer* rxBuf);
void init_uart(void);

void write_uart(char* data, uint8_t len);


#endif __UART
