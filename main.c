/** \file main.c
 * \brief Sample LPC1114 project
 * \details This file holds a very basic code for LPC1114. This code configures
 * flash access time, enables the PLL and all required clocks and peripherals
 * to achieve the highest allowed frequency for LPC1114 (50MHz). Main code
 * block just blinks the LED. The LED port and pin are defined in config.h
 * file. Target core frequency and quartz crystal resonator frequency are
 * defined there as well.
 *
 * \author Freddie Chopin, http://www.freddiechopin.info/
 * \date 2012-01-08
 */

/******************************************************************************
* project: lpc1114_blink_led
* chip: LPC1114
* compiler: arm-none-eabi-gcc (Sourcery CodeBench Lite 2011.09-69) 4.6.1
*
* prefix: (none)
*
* available global functions:
* 	int main(void)
*
* available local functions:
* 	static void flash_access_time(uint32_t frequency)
* 	static uint32_t pll_start(uint32_t crystal, uint32_t frequency)
* 	static void init_system(void)
*
* available interrupt handlers:
******************************************************************************/

/*
+=============================================================================+
| includes
+=============================================================================+
*/

#include <stdint.h>

#include "inc/LPC11xx.h"
#include "inc/core_cm0.h"
#include "config.h"
#include "buffer.h"
#include "uart.h"
#include "hdr/hdr_syscon.h"

/*
+=============================================================================+
| module variables
+=============================================================================+
*/

/*
+=============================================================================+
| local functions' declarations
+=============================================================================+
*/

static void flash_access_time(uint32_t frequency);
static uint32_t pll_start(uint32_t crystal, uint32_t frequency);
static void init_system(void);
static void blink_led(void);
static void init_spi(void);
/*
+=============================================================================+
| global functions
+=============================================================================+
*/

#define RBRIE 	0x1
#define THREIE	0x2
#define RXLIE		0x4

#define IIR_RDA 0x2
#define IIR_RLS 0x3
#define IIR_CTI 0x6
#define FIFO_SIZE 16
uint8_t gotDataSerial = 0;


volatile Buffer rxBuf = {.end=0, .readStart=0};

/*------------------------------------------------------------------------*//**
* \brief main code block
* \details 
*//*-------------------------------------------------------------------------*/

// 1. update_display (if new data from serial/spi)
		// - static image
		// - binary clock
		// - animation
	// 2. drive_LED's
	// 		-- traverse through linked-list once
	// 3. update_time
	//		-- use an interrupt+timer

int main(void)
{

	pll_start(CRYSTAL, FREQUENCY);			// start the PLL
	init_system();							// initialize other necessary elements
	init_uart();
	init_spi();
	resetBuf(&rxBuf);
	unsigned int i;
	char x;
	
//	blink_led();
	while(1){                    //infinite loop

	//	blink_led();
		gotDataSerial = getNumBytesToRead(&rxBuf);
		// write_uart("bytes received:", 15);
		// x = '0' + gotDataSerial;
		// write_uart(&x, 1);
		if(gotDataSerial > 0){
			gotDataSerial = 0;
		//	blink_led();
			// write_uart("reading bytes", 13);
			int receivedBytes;
			receivedBytes = getNumBytesToRead(&rxBuf);
			// write_uart("received successfully!", 22);
			write_uart(rxBuf.data, receivedBytes);
			resetBuf(&rxBuf);
			LPC_SSP0->DR = 0x4741;

		}

	  for(i=0; i < 0xFFFFF; ++i);         // delay to make scope readings easier to read

  }

	return 0;
}

/*
+=============================================================================+
| local functions
+=============================================================================+
*/

static void blink_led(void) {
	volatile uint32_t count, count_max = 1000000;	// with core frequency ~50MHz this gives ~1.5Hz blinking frequency

	LED_GPIO->DIR |= LED;					// set the direction of the LED pin to output

	for (count = 0; count < count_max; count++);	// delay
	LED_gma = LED;						// instead of LED_GPIO->DATA |= LED;
	for (count = 0; count < count_max; count++);	// delay
	LED_gma = 0;						// instead of LED_GPIO->DATA &= ~LED;

}


static void init_spi(void) {
	//SET UP UART (sec. 13.2 in datasheet "BASIC CONFIGURATION")
  LPC_IOCON->PIO0_8         |= 0x01;      // MISO0
  LPC_IOCON->PIO0_9         |= 0x01;      // MOSI0
	LPC_IOCON->SWCLK_PIO0_10  |= 0x02;      // SCK0 *- IOCON_SCK0_LOC dependent(reset value selects PIO0_10)


  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<11) | (1<<18);    //enable clock to SPI0 & SPI1

  LPC_SYSCON->SSP0CLKDIV    |= 200;   // divide main clock by 100 (50MHz in this file) = 500kHz
  																			// max sck on the nrF8001 is 3MHz
	LPC_SYSCON->PRESETCTRL  	|= 1; 		//de-assert SPI reset
  	
	LPC_SSP0->CR0 						= 0xf; 	// DSS = 16-bit data transfer
	// LPC_SSP0->CR0 						= 0x0707; 	// DSS = 8-bit data transfer

	LPC_SSP0->CPSR					  |= 0x16;
	LPC_SSP0->CR1 						|= (1 << 1); // spi control enable
 
  // LPC_UART->IER = RBRIE | RXLIE; // enabling THREIE makes stuck in handler because we're sending...
  // NVIC_EnableIRQ(UART_IRQn); // enable UART interrupt

}


/*------------------------------------------------------------------------*//**
* \brief Configures flash access time.
* \details Configures flash access time which allows the chip to run at higher
* speeds.
*
* \param [in] frequency defines the target frequency of the core
*//*-------------------------------------------------------------------------*/

static void flash_access_time(uint32_t frequency)
{
	uint32_t access_time, flashcfg_register;

	if (frequency < 20000000ul)				// 1 system clock for core speed below 20MHz
		access_time = FLASHCFG_FLASHTIM_1CLK;
	else if (frequency < 40000000ul)		// 2 system clocks for core speed between 20MHz and 40MHz
		access_time = FLASHCFG_FLASHTIM_2CLK;
	else									// 3 system clocks for core speed over 40MHz
		access_time = FLASHCFG_FLASHTIM_3CLK;

	// do not modify reserved bits in FLASHCFG register
	flashcfg_register = FLASHCFG;			// read register
	flashcfg_register &= ~(FLASHCFG_FLASHTIM_mask << FLASHCFG_FLASHTIM_bit);	// mask the FLASHTIM field
	flashcfg_register |= access_time << FLASHCFG_FLASHTIM_bit;	// use new FLASHTIM value
	FLASHCFG = flashcfg_register;			// save the new value back to the register
}

/*------------------------------------------------------------------------*//**
* \brief Starts the PLL.
* \details Configure and enable PLL to achieve some frequency with some
* crystal. Before the speed change flash access time is configured via
* flash_access_time(). Main oscillator is configured and started. PLL
* parameters m and p are based on function parameters. The PLL is configured,
* started and selected as the main clock. AHB clock divider is set to 1.
*
* \param [in] crystal is the frequency of the crystal resonator connected to
* the LPC1114 chip.
* \param [in] frequency is the desired target frequency after enabling the PLL
*
* \return real frequency that was set
*//*-------------------------------------------------------------------------*/

static uint32_t pll_start(uint32_t crystal, uint32_t frequency)
{
	uint32_t m, p = 0, fcco;

	flash_access_time(frequency);			// configure flash access time first

	// SYSOSCCTRL_FREQRANGE should be 0 for crystals in range 1 - 20MHz
	// SYSOSCCTRL_FREQRANGE should be 1 for crystals in range 15 - 25MHz
	if (crystal < 17500000)					// divide the ranges on 17.5MHz then
		LPC_SYSCON->SYSOSCCTRL = 0;			// "lower speed" crystals
	else
		LPC_SYSCON->SYSOSCCTRL = SYSOSCCTRL_FREQRANGE;	// "higher speed" crystals

	LPC_SYSCON->PDRUNCFG &= ~PDRUNCFG_SYSOSC_PD;	// power-up main oscillator

	LPC_SYSCON->SYSPLLCLKSEL = SYSPLLCLKSEL_SEL_IRC;	// select main oscillator as the input clock for PLL
	LPC_SYSCON->SYSPLLCLKUEN = 0;			// confirm the change of PLL input clock by toggling the...
	LPC_SYSCON->SYSPLLCLKUEN = SYSPLLUEN_ENA;	// ...ENA bit in LPC_SYSCON->SYSPLLCLKUEN register

	// calculate PLL parameters
	m = frequency / crystal;				// M is the PLL multiplier
	fcco = m * crystal * 2;					// FCCO is the internal PLL frequency

	frequency = crystal * m;

	while (fcco < 156000000)
	{
		fcco *= 2;
		p++;								// find P which gives FCCO in the allowed range (over 156MHz)
	}

	LPC_SYSCON->SYSPLLCTRL = ((m - 1) << SYSPLLCTRL_MSEL_bit) | (p << SYSPLLCTRL_PSEL_bit);	// configure PLL
	LPC_SYSCON->PDRUNCFG &= ~PDRUNCFG_SYSPLL_PD; // power-up PLL

	while (!(LPC_SYSCON->SYSPLLSTAT & SYSPLLSTAT_LOCK));	// wait for PLL lock

	LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_SEL_PLLOUT;	// select PLL output as the main clock
	LPC_SYSCON->MAINCLKUEN = 0;				// confirm the change of main clock by toggling the...
	LPC_SYSCON->MAINCLKUEN = MAINCLKUEN_ENA;	// ...ENA bit in LPC_SYSCON->MAINCLKUEN register

	LPC_SYSCON->SYSAHBCLKDIV = 1;			// set AHB clock divider to 1

	return frequency;
}

/*------------------------------------------------------------------------*//**
* \brief Initializes system.
* \details Enables clock for IO configuration block.
*//*-------------------------------------------------------------------------*/

static void init_system(void)
{
	LPC_SYSCON->SYSAHBCLKCTRL |= SYSAHBCLKCTRL_IOCON;	// enable clock for IO configuration block
}

/*
+=============================================================================+
| ISRs
+=============================================================================+
*/

/******************************************************************************
* END OF FILE
******************************************************************************/
