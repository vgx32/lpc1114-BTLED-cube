
#include "inc/LPC11xx.h"
#include "ssp.h"
#include "uart.h"
#include "buffer.h"

// ACI protocol definitions on nrf8001
// commands
#define TEST 0x01
#define ECHO 0x02
#define TEST_ACI 0x02

// Events
#define DEVICE_STARTED_E    0x81
#define COMMAND_RESPONSE_E  0x84

#define SSPSR_TNF (1<<1)
#define SSPSR_RNE (1<<2)

void init_ssp(void) {
  //SET UP UART (sec. 13.2 in datasheet "BASIC CONFIGURATION")
  LPC_IOCON->PIO0_8         |= 0x01;      // MISO0
  LPC_IOCON->PIO0_9         |= 0x01;      // MOSI0
  LPC_IOCON->SWCLK_PIO0_10  |= 0x02;      // SCK0 *- IOCON_SCK0_LOC dependent(reset value selects PIO0_10)
                              
                              // use PIO0_3 to listen for RDYn signal on nrf8001
  LPC_GPIO0->DIR            |= (1 << 5); // PIO0_5 is REQ_N/output
  LPC_GPIO0->DATA           |= (1 << 5);
  LPC_GPIO0->DIR            |= (1 << 6); // PIO0_6 is RESET control
  LPC_GPIO0->DATA           |= (1 << 6);

  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<11);    //enable clock to SPI0 

  LPC_SYSCON->SSP0CLKDIV    |= 200;   // divide main clock by 100 (50MHz in this file) = 500kHz
                                        // max sck on the nrF8001 is 3MHz
  LPC_SYSCON->PRESETCTRL    |= 1;     //de-assert SPI reset
    
  // LPC_SSP0->CR0             = 0xf;  // DSS = 16-bit data transfer
  LPC_SSP0->CR0            = 0x0707;   // DSS = 8-bit data transfer

  LPC_SSP0->CPSR            |= 0x16;
  LPC_SSP0->CR1             |= (1 << 1); // spi control enable
  
  // empty RX fifo
  char dummy;
  while(LPC_SSP0->SR & SSPSR_RNE) {
    dummy = LPC_SSP0->DR;
  }
  // LPC_UART->IER = RBRIE | RXLIE; // enabling THREIE makes stuck in handler because we're sending...
   // Enable the SSP Interrupt 
  // NVIC_EnableIRQ(SSP0_IRQn);
  
  reset_chip();

}


void write_ssp(char* data, uint8_t len){
  uint32_t i;
  // set the REQn pin low 
  // wait for RDYn signal from BLE module
  // write data...
  for (i =0; i < len; i++) {
    while(~LPC_SSP0->SR & SSPSR_TNF); // make sure tx fifo is not full
    LPC_SSP0->DR = data[i];
  }
  // set REQn pin high
  // flush rx fifo -- optional?
}



#define TREAT_AS_HEX 0

void flushNonProtocolBytes(Buffer* buf){
  char firstChar;
  firstChar = peekBuf(buf);
  while(getNumBytesToRead(buf) > 0 && 
        (firstChar == 0xff || firstChar == 0x00)) {
    readBuf(buf);  // dump invalid char from buf
    firstChar = peekBuf(buf);
  }
}


void reqn(char level) {
  if (level == 0) {
    LPC_GPIO0->DATA &= ~(1<<5);
  } else {
    LPC_GPIO0->DATA |= (1<<5);
  }
}

void wait_for_rdy(void) {
  while(LPC_GPIO0->DATA & (1<<11));
}

void wait_until_bsy(void) {
  while(LPC_SSP0->SR & (1<<4)); // wait until we've finished sending (BSY flag)
}

void reset_chip(void) {
  int i;
  LPC_GPIO0->DATA &= ~(1<<6);
  for(i=0; i < 0xFFFFF; ++i);         // delay to make scope readings easier to read
  LPC_GPIO0->DATA |= (1<<6);
}

// static Buffer in;
void test_ssp(void){
  
  
  Buffer out, in;
  int rbuflen;
  reset_chip();
  // write_uart("testing BTLE connection");
  resetBuf(&out);
  resetBuf(&in);
  rbuflen = 0;
  
  char received_char;
  
  writeCharToBuf(2, &out);
  writeCharToBuf(TEST, &out);
  writeCharToBuf(TEST_ACI, &out);
  
  writeCharToBuf(TREAT_AS_HEX, &in);
  
  // set REQn low
  reqn(0);
  // wait for RDYn to go low
  wait_for_rdy();

  write_ssp(out.data, 3);
  resetBuf(&out);
  wait_until_bsy();
  // set REQn high
  reqn(1);

  // read out dummy bytes in ssp fifo
  while(~LPC_SSP0->SR & SSPSR_RNE);

  while(LPC_SSP0->SR & SSPSR_RNE) {
    received_char = LPC_SSP0->DR;
    writeCharToBuf(received_char, &in);
  }

  writeCharToBuf(0xff, &out);
  writeCharToBuf(0xff, &out);

  // wait for RDYn to go low
  wait_for_rdy();
  // set REQn low
  reqn(0);
  write_ssp(out.data, 2);

  while(~LPC_SSP0->SR & SSPSR_RNE);

  while(LPC_SSP0->SR & SSPSR_RNE) {
    received_char = LPC_SSP0->DR;
    writeCharToBuf(received_char, &in);
  }
  reqn(1);
  flushNonProtocolBytes(&in);
  
  // read debug byte and length byte by feeding 0xff
  // for length bytes, read ssp byte

  rbuflen = getNumBytesToRead(&in);
  if (rbuflen > 0 ){
    write_uart("received some data on spi:");
    write_uart_len(in.data, rbuflen);
  } else {
    write_uart("no data on SPI");
  }
}