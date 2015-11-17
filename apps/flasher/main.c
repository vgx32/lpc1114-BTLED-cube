

#include <stdint.h>

#include "inc/LPC11xx.h"
#include "inc/core_cm0.h"
#include "config.h"
#include "buffer.h"
#include "uart.h"
#include "hdr/hdr_syscon.h"



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

void init_gpio(void) {
	// reset -- PIO0_8
	// flashmode -- PIO0_9
	LPC_GPIO0->DIR |= 0x1 << 8;
	LPC_GPIO0->DIR |= 0x1 << 9;

}

void reset_host(void){
	// reset low
	LPC_GPIO0->DATA &= ~(0x1 << 8);
	delay_ms(50);
	LPC_GPIO0->DATA |= (0x1 << 8);
	// reset high

}

void set_flashmode(unsigned char on) {
	if(on) { // drive CT32B0_MAT2 low
		LPC_GPIO0->DATA &= ~(0x1 << 9);
	} else  {  // drive CT32B0_MAT2 high
		LPC_GPIO0->DATA |= (0x1 << 9);
	}
	// LPC_GPIO0->DATA |= (0x1 << 9);
	reset_host();
}



int main(void)
{

	pll_start(CRYSTAL, FREQUENCY);			// start the PLL
	init_system();							
	resetBuf(&rxBuf);

	init_uart(&rxBuf);
	init_gpio();

	unsigned int i;

	blink_led();
	int uartRXBytes = 0;

	set_flashmode(0);
	while(1){                    
		uartRXBytes = getNumBytesToRead(&rxBuf);
		if(uartRXBytes > 0){
			blink_led();
			char command = readBuf(&rxBuf);
			switch (command){
				case 'f':
					write_uart("flashing");
					set_flashmode(1);
					break;
				case 'r':
					write_uart("running");
					set_flashmode(0);
					break;
				default:
					write_uart("unrecognized command");
					break;
			}
			write_uart_len(rxBuf.data, uartRXBytes);
			resetBuf(&rxBuf);

		}
		// write_uart("waiting for GPIO interrupt");

	  delay_ms(50);

  }

	return 0;
}

/*
+=============================================================================+
| local functions
+=============================================================================+
*/

// #define LED_GPIO							LPC_GPIO1	///< GPIO port to which the LED is connected
// #define LED_pin								8			///< pin number of the LED

// #define LED									(1 << LED_pin)


static void blink_led(void) {
	
	LED_GPIO->DIR |= LED;					// set the direction of the LED pin to output
	delay_ms(500);
	LED_gma = LED;						// instead of LED_GPIO->DATA |= LED;
	delay_ms(500);
	
	LED_gma = 0;						// instead of LED_GPIO->DATA &= ~LED;

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
