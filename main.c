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
#include "ssp.h"
#include "gpio.h"
#include "hdr/hdr_syscon.h"

// porting ACI ble library
#include "ble/aci.h"		// enums and defines for nrf-specific config constants
#include "ble/aci_cmds.h" // command code enums
#include "ble/aci_evts.h" // event code enums
#include "ble/dtm.h" // BTLE -specific direct test mode constant definitions
#include "ble/services.h" // configuration ARRAY from Nordic's nRFGo studio program -- sent to radio at init
#include "ble/acilib.h" // defines for packing buffers with ACI message format
#include "ble/acilib_if.h" // function prototypes for above
#include "ble/aci_protocol_defines.h" // ACI offset definitions for acilib
#include "ble/hal_platform.h" // functions specific to Arduino & PIC *** TODO unsure if necessary for LPC1114 
#include "ble/hal_aci_tl.h" // (big refactoring) has the SPI data read interrupt and radio init function


// TODO: im(port) the following to project or determine if unnnecessary
// #include "ble/aci_queue.h" // a queue implementation.. ** TODO: analyze how this code is used with lib_aci and aci_tl
// #include "ble/common.h" // PIC-specific pin control defs. TODO: port from PIC to LPC
// #include "ble/lib_aci.h" // (big refactoring) library used by ble application (uart mode) ***lots of bools ***


static void flash_access_time(uint32_t frequency);
static uint32_t pll_start(uint32_t crystal, uint32_t frequency);
static void init_system(void);
static void blink_led(void);

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


static Buffer rxBuf = {.end=0, .start=0};

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

#define RDYnBIT (1<<11)

// #define GPORTN(x)  LPC_GPIO ## #x 
static volatile char rdy_int = 0;
void my_test_interrupt(void) {
	// int32_t ie = LPC_GPIO0->IE;
	rdy_int = 1;
	
	// LPC_GPIO0->IC |= 0xff;
	LPC_GPIO0->IC |= RDYnBIT;
	__NOP;
	__NOP;
}



void init_rdyn(void) {
  LPC_GPIO0->IE |= RDYnBIT; 


	// LPC_UART->IER = RBRIE | RXLIE; // enabling THREIE makes stuck in handler because we're sending...
  NVIC_EnableIRQ(EINT0_IRQn); // enable GPIO0 interrupt

  
}

void init_leds(void) 
{
	gpio_enable(1, 5, 1);
	gpio_enable(1, 8, 1);
	gpio_write(1, 5, 0);
	gpio_write(1, 8, 0);

}

static void delay_us(uint32_t duration) {
	volatile uint32_t count_max = FREQUENCY/10000;	// 1ms... approximately
	uint32_t i, us;
	
	for (us = 0; us < duration; us++) {
		for(i = 0; i < count_max; i++); // 1ms count
	}
}


static void delay_ms(uint32_t duration) {
	volatile uint32_t count_max = FREQUENCY/10000;	// 1ms... approximately
	uint32_t i, ms;
	
	for (ms = 0; ms < duration; ms++) {
		for(i = 0; i < count_max; i++); // 1ms count
	}
}


int main(void)
{

	pll_start(CRYSTAL, FREQUENCY);			// start the PLL
	init_system();							// initialize other necessary elements
	resetBuf(&rxBuf);

	init_uart(&rxBuf);
	// init_ssp();
	unsigned int i;
	for(i=0; i < 0xFFFFF; ++i);         // delay to make scope readings easier to read
	init_leds();
	// init_rdyn();
	uint8_t port = 0 , pin = 11, enable = 1;
	// gpio_setup_int(port, pin, enable, PIOINT0_IRQHandler);
	gpio_setup_int(0, 11, 1, my_test_interrupt);
	
	write_uart("initialized rdy interrupt");
	
	blink_led();
	int uartRXBytes = 0;
	char numbuf[2];
	numbuf[1] = '\0';
	while(1){                    //infinite loop
		// uartRXBytes = getNumBytesToRead(&rxBuf);
		// if(uartRXBytes > 0){
		// 	blink_led();

		// 	write_uart_len(rxBuf.data, uartRXBytes);
		// 	resetBuf(&rxBuf);

		// 	test_ssp();

		// }
		// write_uart("waiting for GPIO interrupt");
		// blink_led();
		if(rdy_int) {

			write_uart("gpio interrupt fired:");
			blink_led();
			rdy_int = 0;
		}

	  // for(i=0; i < 0xFFFFF; ++i);         // delay to make scope readings easier to read

  }

	return 0;
}

/*
+=============================================================================+
| local functions
+=============================================================================+
*/


static void blink_led(void) {
	
	// LED_GPIO->DIR |= LED;					// set the direction of the LED pin to output
	delay_ms(500);
	LED_gma = LED;						// instead of LED_GPIO->DATA |= LED;
	gpio_write(1, 5, 1);
	gpio_write(1, 8, 1);
	delay_ms(500);
	
	LED_gma = 0;					// instead of LED_GPIO->DATA &= ~LED;
	gpio_write(1, 5, 0);
	gpio_write(1, 8, 0);
	
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
