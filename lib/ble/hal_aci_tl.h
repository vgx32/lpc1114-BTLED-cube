/** @file
 * @brief Interface for hal_aci_tl.
 */
 
/** @defgroup hal_aci_tl hal_aci_tl
@{
@ingroup hal
 
@brief Module for the ACI Transport Layer interface
@details This module is responsible for sending and receiving messages over the ACI interface of the nRF8001 chip.
 The hal_aci_tl_send_cmd() can be called directly to send ACI commands.


The RDYN line is hooked to an interrupt on the MCU when the level is low.
The SPI master clocks in the interrupt context.
The ACI Command is taken from the head of the command queue is sent over the SPI
and the received ACI event is placed in the tail of the event queue.

*/
 
#ifndef HAL_ACI_TL_H__
#define HAL_ACI_TL_H__

#include "hal_platform.h"
#include "aci.h"

#ifndef HAL_ACI_MAX_LENGTH
#define HAL_ACI_MAX_LENGTH 31
#endif

#define bool uint8_t
#define true 1
#define false 0
/** Data type for ACI commands and events */
typedef struct {
  uint8_t status_byte;
  uint8_t buffer[HAL_ACI_MAX_LENGTH+1];
} _aci_packed_ hal_aci_data_t;

ACI_ASSERT_SIZE(hal_aci_data_t, HAL_ACI_MAX_LENGTH + 2);

uint8_t m_aci_spi_transfer(hal_aci_data_t * data_to_send, hal_aci_data_t * received_data);
uint8_t lib_aci_board_init(void);



#endif 

