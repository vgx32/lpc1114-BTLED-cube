
#include "inc/LPC11xx.h"
#include "ssp.h"


void init_ssp(void) {
  //SET UP UART (sec. 13.2 in datasheet "BASIC CONFIGURATION")
  LPC_IOCON->PIO0_8         |= 0x01;      // MISO0
  LPC_IOCON->PIO0_9         |= 0x01;      // MOSI0
  LPC_IOCON->SWCLK_PIO0_10  |= 0x02;      // SCK0 *- IOCON_SCK0_LOC dependent(reset value selects PIO0_10)


  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<11) | (1<<18);    //enable clock to SPI0 & SPI1

  LPC_SYSCON->SSP0CLKDIV    |= 200;   // divide main clock by 100 (50MHz in this file) = 500kHz
                                        // max sck on the nrF8001 is 3MHz
  LPC_SYSCON->PRESETCTRL    |= 1;     //de-assert SPI reset
    
  LPC_SSP0->CR0             = 0xf;  // DSS = 16-bit data transfer
  // LPC_SSP0->CR0            = 0x0707;   // DSS = 8-bit data transfer

  LPC_SSP0->CPSR            |= 0x16;
  LPC_SSP0->CR1             |= (1 << 1); // spi control enable
 
  // LPC_UART->IER = RBRIE | RXLIE; // enabling THREIE makes stuck in handler because we're sending...
  // NVIC_EnableIRQ(UART_IRQn); // enable UART interrupt

}
