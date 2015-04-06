
#include "../inc/LPC11xx.h"
#include "buffer.h"
#include "../inc/core_cm0.h"
#include "uart.h"


extern Buffer rxBuf;


#define RBRIE   0x1
#define THREIE  0x2
#define RXLIE   0x4

#define IIR_RDA 0x2
#define IIR_RLS 0x3
#define IIR_CTI 0x6
#define FIFO_SIZE 16


char dataChar = 0;
void UART_IRQHandler(void)
{
  uint32_t iir;
  /* read IIR and clear it */
  iir = LPC_UART->IIR;
  iir >>= 1; /* skip pending bit in IIR */
  iir &= 0x07; /* check bit 1~3, interrupt identification */
  if (iir == IIR_RDA) // Receive Data Available
  {
    char data = LPC_UART->RBR;
    dataChar = data;
    writeCharToBuf(dataChar, &rxBuf);
    
  
  } else if (iir == IIR_RLS) {
    write_uart("error-rls", 9);
  }
}


void init_uart(void) {
  //SET UP UART (sec. 13.2 in datasheet "BASIC CONFIGURATION")
  LPC_IOCON->PIO1_6             |= 0x01;      //configure UART RXD pin (sec 7.4.40)
  LPC_IOCON->PIO1_7         |= 0x01;       //configure UART TXD pin (sec. 7.4.41)
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);    //enable clock to UART (sec. 3.5.14)
  LPC_SYSCON->UARTCLKDIV    |= 0x9B;       //0x9B will give approx. 19.2K baud signal (sec. 3.5.16)
  LPC_UART->FCR             |= 0x01;       //enable UART FIFOs (necessary for operation) (sec. 13.5.6)
  LPC_UART->LCR             |= 0x03;       //set for 8 bit data width (sec. 13.5.7)
  LPC_UART->TER             |= 0x80;       //transmit enable (sec. 13.5.16)

  LPC_UART->IER = RBRIE | RXLIE; // enabling THREIE makes stuck in handler because we're sending...
  NVIC_EnableIRQ(UART_IRQn); // enable UART interrupt

}

void write_uart(char* data, uint8_t len){
  unsigned int i,j, innerLimit;
  for (j = 0; j < len; j = j + FIFO_SIZE)
  {
    // innerLimit = j > len? (16 -(j - len)) : 16;
    innerLimit = len - j > FIFO_SIZE ? FIFO_SIZE : len - j;
    for(i = 0; i < innerLimit; i++) {
       LPC_UART->THR |= data[j + i];              //transmit data (sec. 13.5.2)
    }
    while(!(LPC_UART->LSR & TEMT));
  }
  LPC_UART->THR |= '\n';
}