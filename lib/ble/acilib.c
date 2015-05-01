/* Copyright (c) 2014, Nordic Semiconductor ASA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file
 *
 * @ingroup group_acilib
 *
 * @brief Implementation of the acilib module.
 */


#include "hal_platform.h"
#include "aci.h"
#include "aci_cmds.h"
#include "aci_evts.h"
#include "acilib.h"
#include "aci_protocol_defines.h"
#include "acilib_if.h"

void acil_encode_cmd_sleep(uint8_t *buffer)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = 1;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_SLEEP;
}

void acil_encode_cmd_get_device_version(uint8_t *buffer)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = 1;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_GET_DEVICE_VERSION;
}

void acil_encode_cmd_set_local_data(uint8_t *buffer, aci_cmd_params_set_local_data_t *p_aci_cmd_params_set_local_data, uint8_t data_size)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_SET_LOCAL_DATA_BASE_LEN + data_size;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_SET_LOCAL_DATA;
  *(buffer + OFFSET_ACI_CMD_T_SET_LOCAL_DATA + OFFSET_ACI_CMD_PARAMS_SEND_DATA_T_TX_DATA + OFFSET_ACI_TX_DATA_T_PIPE_NUMBER) = p_aci_cmd_params_set_local_data->tx_data.pipe_number;
  memcpy(buffer + OFFSET_ACI_CMD_T_SET_LOCAL_DATA + OFFSET_ACI_CMD_PARAMS_SEND_DATA_T_TX_DATA + OFFSET_ACI_TX_DATA_T_ACI_DATA,  &(p_aci_cmd_params_set_local_data->tx_data.aci_data[0]), data_size);
}

void acil_encode_cmd_connect(uint8_t *buffer, aci_cmd_params_connect_t *p_aci_cmd_params_connect)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_CONNECT_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_CONNECT;
  *(buffer + OFFSET_ACI_CMD_T_CONNECT + OFFSET_ACI_CMD_PARAMS_CONNECT_T_TIMEOUT_MSB) = (uint8_t)(p_aci_cmd_params_connect->timeout >> 8);
  *(buffer + OFFSET_ACI_CMD_T_CONNECT + OFFSET_ACI_CMD_PARAMS_CONNECT_T_TIMEOUT_LSB) = (uint8_t)(p_aci_cmd_params_connect->timeout);
  *(buffer + OFFSET_ACI_CMD_T_CONNECT + OFFSET_ACI_CMD_PARAMS_CONNECT_T_ADV_INTERVAL_MSB) = (uint8_t)(p_aci_cmd_params_connect->adv_interval >> 8);
  *(buffer + OFFSET_ACI_CMD_T_CONNECT + OFFSET_ACI_CMD_PARAMS_CONNECT_T_ADV_INTERVAL_LSB) = (uint8_t)(p_aci_cmd_params_connect->adv_interval);
}

void acil_encode_cmd_bond(uint8_t *buffer, aci_cmd_params_bond_t *p_aci_cmd_params_bond)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_BOND_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_BOND;
  *(buffer + OFFSET_ACI_CMD_T_BOND + OFFSET_ACI_CMD_PARAMS_BOND_T_TIMEOUT_MSB) = (uint8_t)(p_aci_cmd_params_bond->timeout >> 8);
  *(buffer + OFFSET_ACI_CMD_T_BOND + OFFSET_ACI_CMD_PARAMS_BOND_T_TIMEOUT_LSB) = (uint8_t)(p_aci_cmd_params_bond->timeout);
  *(buffer + OFFSET_ACI_CMD_T_BOND + OFFSET_ACI_CMD_PARAMS_BOND_T_ADV_INTERVAL_MSB) = (uint8_t)(p_aci_cmd_params_bond->adv_interval >> 8);
  *(buffer + OFFSET_ACI_CMD_T_BOND + OFFSET_ACI_CMD_PARAMS_BOND_T_ADV_INTERVAL_LSB) = (uint8_t)(p_aci_cmd_params_bond->adv_interval);
}

void acil_encode_cmd_disconnect(uint8_t *buffer, aci_cmd_params_disconnect_t *p_aci_cmd_params_disconnect)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_DISCONNECT_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_DISCONNECT;
  *(buffer + OFFSET_ACI_CMD_T_DISCONNECT + OFFSET_ACI_CMD_PARAMS_DISCONNECT_T_REASON) = (uint8_t)(p_aci_cmd_params_disconnect->reason);
}

void acil_encode_baseband_reset(uint8_t *buffer)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_BASEBAND_RESET_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_RADIO_RESET;
}

void acil_encode_direct_connect(uint8_t *buffer)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_DIRECT_CONNECT_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_CONNECT_DIRECT;
}

void acil_encode_cmd_wakeup(uint8_t *buffer)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_WAKEUP_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_WAKEUP;
}

void acil_encode_cmd_set_radio_tx_power(uint8_t *buffer, aci_cmd_params_set_tx_power_t *p_aci_cmd_params_set_tx_power)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_SET_RADIO_TX_POWER_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_SET_TX_POWER;
  *(buffer + OFFSET_ACI_CMD_T_SET_TX_POWER + OFFSET_ACI_CMD_PARAMS_SET_TX_POWER_T_DEVICE_POWER) = (uint8_t)p_aci_cmd_params_set_tx_power->device_power;
}

void acil_encode_cmd_get_address(uint8_t *buffer)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_GET_DEVICE_ADDR_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_GET_DEVICE_ADDRESS;
}

void acil_encode_cmd_send_data(uint8_t *buffer, aci_cmd_params_send_data_t *p_aci_cmd_params_send_data_t, uint8_t data_size)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_SEND_DATA_BASE_LEN + data_size;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_SEND_DATA;
  *(buffer + OFFSET_ACI_CMD_T_SEND_DATA + OFFSET_ACI_CMD_PARAMS_SEND_DATA_T_TX_DATA + OFFSET_ACI_TX_DATA_T_PIPE_NUMBER) = p_aci_cmd_params_send_data_t->tx_data.pipe_number;
  memcpy((buffer + OFFSET_ACI_CMD_T_SEND_DATA + OFFSET_ACI_CMD_PARAMS_SEND_DATA_T_TX_DATA + OFFSET_ACI_TX_DATA_T_ACI_DATA), &(p_aci_cmd_params_send_data_t->tx_data.aci_data[0]), data_size);
}

void acil_encode_cmd_request_data(uint8_t *buffer, aci_cmd_params_request_data_t *p_aci_cmd_params_request_data)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_DATA_REQUEST_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_REQUEST_DATA;
  *(buffer + OFFSET_ACI_CMD_T_REQUEST_DATA + OFFSET_ACI_CMD_PARAMS_REQUEST_DATA_T_PIPE_NUMBER) = p_aci_cmd_params_request_data->pipe_number;
}

void acil_encode_cmd_open_remote_pipe(uint8_t *buffer, aci_cmd_params_open_remote_pipe_t *p_aci_cmd_params_open_remote_pipe)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_OPEN_REMOTE_PIPE_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_OPEN_REMOTE_PIPE;
  *(buffer + OFFSET_ACI_CMD_T_OPEN_REMOTE_PIPE + OFFSET_ACI_CMD_PARAMS_OPEN_REMOTE_PIPE_T_PIPE_NUMBER) = p_aci_cmd_params_open_remote_pipe->pipe_number;
}

void acil_encode_cmd_close_remote_pipe(uint8_t *buffer, aci_cmd_params_close_remote_pipe_t *p_aci_cmd_params_close_remote_pipe)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_CLOSE_REMOTE_PIPE_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_CLOSE_REMOTE_PIPE;
  *(buffer + OFFSET_ACI_CMD_T_CLOSE_REMOTE_PIPE + OFFSET_ACI_CMD_PARAMS_CLOSE_REMOTE_PIPE_T_PIPE_NUMBER) = p_aci_cmd_params_close_remote_pipe->pipe_number;
}

void acil_encode_cmd_echo_msg(uint8_t *buffer, aci_cmd_params_echo_t *p_cmd_params_echo, uint8_t msg_size)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_ECHO_MSG_CMD_BASE_LEN + msg_size;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_ECHO;
  memcpy((buffer + OFFSET_ACI_CMD_T_ECHO + OFFSET_ACI_CMD_PARAMS_ECHO_T_ECHO_DATA), &(p_cmd_params_echo->echo_data[0]), msg_size);
}

void acil_encode_cmd_battery_level(uint8_t *buffer)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = 1;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_GET_BATTERY_LEVEL;
}

void acil_encode_cmd_temparature(uint8_t *buffer)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = 1;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_GET_TEMPERATURE;
}

void acil_encode_cmd_read_dynamic_data(uint8_t *buffer)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = 1;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_READ_DYNAMIC_DATA;
}

void acil_encode_cmd_write_dynamic_data(uint8_t *buffer, uint8_t seq_no, uint8_t* dynamic_data, uint8_t dynamic_data_size)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_WRITE_DYNAMIC_DATA_BASE_LEN + dynamic_data_size;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_WRITE_DYNAMIC_DATA;
  *(buffer + OFFSET_ACI_CMD_T_WRITE_DYNAMIC_DATA + OFFSET_ACI_CMD_PARAMS_WRITE_DYNAMIC_DATA_T_SEQ_NO) = seq_no;
  memcpy((buffer + OFFSET_ACI_CMD_T_WRITE_DYNAMIC_DATA + OFFSET_ACI_CMD_PARAMS_WRITE_DYNAMIC_DATA_T_DYNAMIC_DATA), dynamic_data, dynamic_data_size);
}

void acil_encode_cmd_change_timing_req(uint8_t *buffer, aci_cmd_params_change_timing_t *p_aci_cmd_params_change_timing)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_CHANGE_TIMING_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_CHANGE_TIMING;
  *(buffer + OFFSET_ACI_CMD_T_CHANGE_TIMING + OFFSET_ACI_CMD_PARAMS_CHANGE_TIMING_T_CONN_PARAMS + OFFSET_ACI_LL_CONN_PARAMS_T_MIN_CONN_INTERVAL_MSB) = (uint8_t)(p_aci_cmd_params_change_timing->conn_params.min_conn_interval >> 8);
  *(buffer + OFFSET_ACI_CMD_T_CHANGE_TIMING + OFFSET_ACI_CMD_PARAMS_CHANGE_TIMING_T_CONN_PARAMS + OFFSET_ACI_LL_CONN_PARAMS_T_MIN_CONN_INTERVAL_LSB) = (uint8_t)(p_aci_cmd_params_change_timing->conn_params.min_conn_interval);
  *(buffer + OFFSET_ACI_CMD_T_CHANGE_TIMING + OFFSET_ACI_CMD_PARAMS_CHANGE_TIMING_T_CONN_PARAMS + OFFSET_ACI_LL_CONN_PARAMS_T_MAX_CONN_INTERVAL_MSB) = (uint8_t)(p_aci_cmd_params_change_timing->conn_params.max_conn_interval >> 8);
  *(buffer + OFFSET_ACI_CMD_T_CHANGE_TIMING + OFFSET_ACI_CMD_PARAMS_CHANGE_TIMING_T_CONN_PARAMS + OFFSET_ACI_LL_CONN_PARAMS_T_MAX_CONN_INTERVAL_LSB) = (uint8_t)(p_aci_cmd_params_change_timing->conn_params.max_conn_interval);
  *(buffer + OFFSET_ACI_CMD_T_CHANGE_TIMING + OFFSET_ACI_CMD_PARAMS_CHANGE_TIMING_T_CONN_PARAMS + OFFSET_ACI_LL_CONN_PARAMS_T_SLAVE_LATENCY_MSB    ) = (uint8_t)(p_aci_cmd_params_change_timing->conn_params.slave_latency >> 8);
  *(buffer + OFFSET_ACI_CMD_T_CHANGE_TIMING + OFFSET_ACI_CMD_PARAMS_CHANGE_TIMING_T_CONN_PARAMS + OFFSET_ACI_LL_CONN_PARAMS_T_SLAVE_LATENCY_LSB    ) = (uint8_t)(p_aci_cmd_params_change_timing->conn_params.slave_latency);
  *(buffer + OFFSET_ACI_CMD_T_CHANGE_TIMING + OFFSET_ACI_CMD_PARAMS_CHANGE_TIMING_T_CONN_PARAMS + OFFSET_ACI_LL_CONN_PARAMS_T_TIMEOUT_MULT_MSB     ) = (uint8_t)(p_aci_cmd_params_change_timing->conn_params.timeout_mult >> 8);
  *(buffer + OFFSET_ACI_CMD_T_CHANGE_TIMING + OFFSET_ACI_CMD_PARAMS_CHANGE_TIMING_T_CONN_PARAMS + OFFSET_ACI_LL_CONN_PARAMS_T_TIMEOUT_MULT_LSB     ) = (uint8_t)(p_aci_cmd_params_change_timing->conn_params.timeout_mult);
}

void acil_encode_cmd_set_app_latency(uint8_t *buffer, aci_cmd_params_set_app_latency_t *p_aci_cmd_params_set_app_latency)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_SET_APP_LATENCY_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_SET_APP_LATENCY;
  *(buffer + OFFSET_ACI_CMD_T_SET_APP_LATENCY + OFFSET_ACI_CMD_PARAMS_SET_APP_LATENCY_T_MODE) = (uint8_t)( p_aci_cmd_params_set_app_latency->mode);
  *(buffer + OFFSET_ACI_CMD_T_SET_APP_LATENCY + OFFSET_ACI_CMD_PARAMS_SET_APP_LATENCY_T_LATENCY_MSB) = (uint8_t)( p_aci_cmd_params_set_app_latency->latency>>8);
  *(buffer + OFFSET_ACI_CMD_T_SET_APP_LATENCY + OFFSET_ACI_CMD_PARAMS_SET_APP_LATENCY_T_LATENCY_LSB) = (uint8_t)( p_aci_cmd_params_set_app_latency->latency);
}

void acil_encode_cmd_change_timing_req_GAP_PPCP(uint8_t *buffer)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_CHANGE_TIMING_LEN_GAP_PPCP;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_CHANGE_TIMING;
}


void acil_encode_cmd_setup(uint8_t *buffer, aci_cmd_params_setup_t *p_aci_cmd_params_setup, uint8_t setup_data_size)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = setup_data_size + MSG_SETUP_CMD_BASE_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_SETUP;
  memcpy((buffer + OFFSET_ACI_CMD_T_SETUP), &(p_aci_cmd_params_setup->setup_data[0]), setup_data_size);
}

void acil_encode_cmd_dtm_cmd(uint8_t *buffer, aci_cmd_params_dtm_cmd_t *p_aci_cmd_params_dtm_cmd)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_DTM_CMD;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_DTM_CMD;
  *(buffer + OFFSET_ACI_CMD_T_DTM_CMD) = p_aci_cmd_params_dtm_cmd->cmd_msb;
  *(buffer + OFFSET_ACI_CMD_T_DTM_CMD + 1) = p_aci_cmd_params_dtm_cmd->cmd_lsb;
}

void acil_encode_cmd_send_data_ack(uint8_t *buffer, const uint8_t pipe_number )
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_ACK_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_SEND_DATA_ACK;
  *(buffer + OFFSET_ACI_CMD_T_SEND_DATA_ACK + OFFSET_ACI_CMD_PARAMS_SEND_DATA_ACK_T_PIPE_NUMBER) = pipe_number;
}

void acil_encode_cmd_send_data_nack(uint8_t *buffer, const uint8_t pipe_number, const uint8_t err_code )
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_NACK_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_SEND_DATA_NACK;
  *(buffer + OFFSET_ACI_CMD_T_SEND_DATA_NACK + OFFSET_ACI_CMD_PARAMS_SEND_DATA_NACK_T_PIPE_NUMBER) = pipe_number;
  *(buffer + OFFSET_ACI_CMD_T_SEND_DATA_NACK + OFFSET_ACI_CMD_PARAMS_SEND_DATA_NACK_T_ERROR_CODE) = err_code;
}

void acil_encode_cmd_bond_security_request(uint8_t *buffer)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = 1;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_BOND_SECURITY_REQUEST;
}

void acil_encode_cmd_broadcast(uint8_t *buffer, aci_cmd_params_broadcast_t * p_aci_cmd_params_broadcast)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_BROADCAST_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_BROADCAST;
  *(buffer + OFFSET_ACI_CMD_T_BROADCAST + OFFSET_ACI_CMD_PARAMS_BROADCAST_T_TIMEOUT_LSB) = (p_aci_cmd_params_broadcast->timeout & 0xff);
  *(buffer + OFFSET_ACI_CMD_T_BROADCAST + OFFSET_ACI_CMD_PARAMS_BROADCAST_T_TIMEOUT_MSB) = (uint8_t)(p_aci_cmd_params_broadcast->timeout >> 8);
  *(buffer + OFFSET_ACI_CMD_T_BROADCAST + OFFSET_ACI_CMD_PARAMS_BROADCAST_T_ADV_INTERVAL_LSB) = (p_aci_cmd_params_broadcast->adv_interval & 0xff);
  *(buffer + OFFSET_ACI_CMD_T_BROADCAST + OFFSET_ACI_CMD_PARAMS_BROADCAST_T_ADV_INTERVAL_MSB) = (uint8_t)(p_aci_cmd_params_broadcast->adv_interval >> 8);
}
  
void acil_encode_cmd_open_adv_pipes(uint8_t *buffer, aci_cmd_params_open_adv_pipe_t * p_aci_cmd_params_open_adv_pipe)
{
  *(buffer + OFFSET_ACI_CMD_T_LEN) = MSG_OPEN_ADV_PIPES_LEN;
  *(buffer + OFFSET_ACI_CMD_T_CMD_OPCODE) = ACI_CMD_OPEN_ADV_PIPE;
  memcpy(buffer + OFFSET_ACI_CMD_T_OPEN_ADV_PIPE + OFFSET_ACI_CMD_PARAMS_OPEN_ADV_PIPE_T_PIPES, p_aci_cmd_params_open_adv_pipe->pipes, 8);
}

void acil_decode_evt_device_started(uint8_t *buffer_in, aci_evt_params_device_started_t *p_evt_params_device_started)
{
  p_evt_params_device_started->device_mode = (aci_device_operation_mode_t) *(buffer_in + OFFSET_ACI_EVT_T_DEVICE_STARTED+OFFSET_ACI_EVT_PARAMS_DEVICE_STARTED_T_DEVICE_MODE);
  p_evt_params_device_started->hw_error = (aci_hw_error_t) *(buffer_in + OFFSET_ACI_EVT_T_DEVICE_STARTED+OFFSET_ACI_EVT_PARAMS_DEVICE_STARTED_T_HW_ERROR);
  p_evt_params_device_started->credit_available = *(buffer_in + OFFSET_ACI_EVT_T_DEVICE_STARTED+OFFSET_ACI_EVT_PARAMS_DEVICE_STARTED_T_CREDIT_AVAILABLE); 
}

void acil_decode_evt_pipe_status(uint8_t *buffer_in, aci_evt_params_pipe_status_t *p_aci_evt_params_pipe_status)
{
  memcpy((uint8_t *)p_aci_evt_params_pipe_status->pipes_open_bitmap, (buffer_in + OFFSET_ACI_EVT_T_PIPE_STATUS + OFFSET_ACI_EVT_PARAMS_PIPE_STATUS_T_PIPES_OPEN_BITMAP), 8);
  memcpy((uint8_t *)p_aci_evt_params_pipe_status->pipes_closed_bitmap, (buffer_in + OFFSET_ACI_EVT_T_PIPE_STATUS + OFFSET_ACI_EVT_PARAMS_PIPE_STATUS_T_PIPES_CLOSED_BITMAP), 8);
}

void acil_decode_evt_disconnected(uint8_t *buffer_in, aci_evt_params_disconnected_t *p_aci_evt_params_disconnected)
{
  p_aci_evt_params_disconnected->aci_status = (aci_status_code_t)*(buffer_in + OFFSET_ACI_EVT_T_DISCONNECTED + OFFSET_ACI_EVT_PARAMS_DISCONNECTED_T_ACI_STATUS);
  p_aci_evt_params_disconnected->btle_status = *(buffer_in + OFFSET_ACI_EVT_T_DISCONNECTED + OFFSET_ACI_EVT_PARAMS_DISCONNECTED_T_BTLE_STATUS);
}

void acil_decode_evt_bond_status(uint8_t *buffer_in, aci_evt_params_bond_status_t *p_aci_evt_params_bond_status)
{
  p_aci_evt_params_bond_status->status_code = (aci_bond_status_code_t)*(buffer_in + OFFSET_ACI_EVT_T_BOND_STATUS + OFFSET_ACI_EVT_PARAMS_BOND_STATUS_T_STATUS_CODE);
  p_aci_evt_params_bond_status->status_source = (aci_bond_status_source_t)*(buffer_in + OFFSET_ACI_EVT_T_BOND_STATUS + OFFSET_ACI_EVT_PARAMS_BOND_STATUS_T_STATUS_SOURCE);
  p_aci_evt_params_bond_status->secmode1_bitmap = *(buffer_in + OFFSET_ACI_EVT_T_BOND_STATUS + OFFSET_ACI_EVT_PARAMS_BOND_STATUS_T_SECMODE1_BITMAP);
  p_aci_evt_params_bond_status->secmode2_bitmap = *(buffer_in + OFFSET_ACI_EVT_T_BOND_STATUS + OFFSET_ACI_EVT_PARAMS_BOND_STATUS_T_SECMODE2_BITMAP);
  p_aci_evt_params_bond_status->keys_exchanged_slave = *(buffer_in + OFFSET_ACI_EVT_T_BOND_STATUS + OFFSET_ACI_EVT_PARAMS_BOND_STATUS_T_KEYS_EXCHANGED_SLAVE);
  p_aci_evt_params_bond_status->keys_exchanged_master = *(buffer_in + OFFSET_ACI_EVT_T_BOND_STATUS + OFFSET_ACI_EVT_PARAMS_BOND_STATUS_T_KEYS_EXCHANGED_MASTER);
}

uint8_t acil_decode_evt_data_received(uint8_t *buffer_in, aci_evt_params_data_received_t *p_evt_params_data_received)
{
  uint8_t size = *( buffer_in + OFFSET_ACI_EVT_T_LEN) - (OFFSET_ACI_EVT_T_DATA_RECEIVED + OFFSET_ACI_RX_DATA_T_ACI_DATA) + 1 ;
  p_evt_params_data_received->rx_data.pipe_number = *(buffer_in + OFFSET_ACI_EVT_T_DATA_RECEIVED + OFFSET_ACI_RX_DATA_T_PIPE_NUMBER);
  memcpy((uint8_t *)p_evt_params_data_received->rx_data.aci_data, (buffer_in + OFFSET_ACI_EVT_T_DATA_RECEIVED + OFFSET_ACI_RX_DATA_T_ACI_DATA), size);
  return size;
}

void acil_decode_evt_data_ack(uint8_t *buffer_in, aci_evt_params_data_ack_t *p_evt_params_data_ack)
{
  p_evt_params_data_ack->pipe_number = *(buffer_in + OFFSET_ACI_EVT_T_DATA_ACK + OFFSET_ACI_EVT_PARAMS_DATA_ACK_T_PIPE_NUMBER);
}

uint8_t acil_decode_evt_hw_error(uint8_t *buffer_in, aci_evt_params_hw_error_t *p_aci_evt_params_hw_error)
{
  uint8_t size = *(buffer_in + OFFSET_ACI_EVT_T_LEN) - (OFFSET_ACI_EVT_T_HW_ERROR + OFFSET_ACI_EVT_PARAMS_HW_ERROR_T_FILE_NAME) + 1;
  p_aci_evt_params_hw_error->line_num = (uint16_t)(*(buffer_in + OFFSET_ACI_EVT_T_HW_ERROR + OFFSET_ACI_EVT_PARAMS_HW_ERROR_T_LINE_NUM_MSB)) << 8;
  p_aci_evt_params_hw_error->line_num |= (uint16_t)*(buffer_in + OFFSET_ACI_EVT_T_HW_ERROR + OFFSET_ACI_EVT_PARAMS_HW_ERROR_T_LINE_NUM_LSB);
  memcpy((uint8_t *)p_aci_evt_params_hw_error->file_name, (buffer_in + OFFSET_ACI_EVT_T_HW_ERROR + OFFSET_ACI_EVT_PARAMS_HW_ERROR_T_FILE_NAME), size);
  return size;
}

void acil_decode_evt_credit(uint8_t *buffer_in, aci_evt_params_data_credit_t *p_evt_params_data_credit)
{
  p_evt_params_data_credit->credit = *(buffer_in + OFFSET_ACI_EVT_T_DATA_CREDIT + OFFSET_ACI_EVT_PARAMS_DATA_CREDIT_T_CREDIT);
}

void acil_decode_evt_connected(uint8_t *buffer_in, aci_evt_params_connected_t *p_aci_evt_params_connected)
{
  p_aci_evt_params_connected->dev_addr_type = (aci_bd_addr_type_t)*(buffer_in + OFFSET_ACI_EVT_T_CONNECTED + OFFSET_ACI_EVT_PARAMS_CONNECTED_T_DEV_ADDR_TYPE);
  memcpy(&(p_aci_evt_params_connected->dev_addr[0]), (buffer_in + OFFSET_ACI_EVT_T_CONNECTED + OFFSET_ACI_EVT_PARAMS_CONNECTED_T_DEV_ADDR), BTLE_DEVICE_ADDRESS_SIZE);
  p_aci_evt_params_connected->conn_rf_interval       = (uint16_t)*(buffer_in + OFFSET_ACI_EVT_T_CONNECTED + OFFSET_ACI_EVT_PARAMS_CONNECTED_T_CONN_RF_INTERVAL_MSB) << 8;
  p_aci_evt_params_connected->conn_rf_interval      |= (uint16_t)*(buffer_in + OFFSET_ACI_EVT_T_CONNECTED + OFFSET_ACI_EVT_PARAMS_CONNECTED_T_CONN_RF_INTERVAL_LSB);
  p_aci_evt_params_connected->conn_slave_rf_latency  = (uint16_t)*(buffer_in + OFFSET_ACI_EVT_T_CONNECTED + OFFSET_ACI_EVT_PARAMS_CONNECTED_T_CONN_SLAVE_RF_LATENCY_MSB) << 8;
  p_aci_evt_params_connected->conn_slave_rf_latency |= (uint16_t)*(buffer_in + OFFSET_ACI_EVT_T_CONNECTED + OFFSET_ACI_EVT_PARAMS_CONNECTED_T_CONN_SLAVE_RF_LATENCY_LSB);
  p_aci_evt_params_connected->conn_rf_timeout        = (uint16_t)*(buffer_in + OFFSET_ACI_EVT_T_CONNECTED + OFFSET_ACI_EVT_PARAMS_CONNECTED_T_CONN_RF_TIMEOUT_MSB) << 8;
  p_aci_evt_params_connected->conn_rf_timeout       |= (uint16_t)*(buffer_in + OFFSET_ACI_EVT_T_CONNECTED + OFFSET_ACI_EVT_PARAMS_CONNECTED_T_CONN_RF_TIMEOUT_LSB);
  p_aci_evt_params_connected->master_clock_accuracy  = (aci_clock_accuracy_t)*(buffer_in + OFFSET_ACI_EVT_T_CONNECTED + OFFSET_ACI_EVT_PARAMS_CONNECTED_T_MASTER_CLOCK_ACCURACY);

}

void acil_decode_evt_timing(uint8_t *buffer_in, aci_evt_params_timing_t *p_evt_params_timing)
{
  p_evt_params_timing->conn_rf_interval       = *(buffer_in + OFFSET_ACI_EVT_T_TIMING + OFFSET_ACI_EVT_PARAMS_TIMING_T_CONN_RF_INTERVAL_MSB) << 8;
  p_evt_params_timing->conn_rf_interval      |= (uint16_t)*(buffer_in + OFFSET_ACI_EVT_T_TIMING + OFFSET_ACI_EVT_PARAMS_TIMING_T_CONN_RF_INTERVAL_LSB);
  p_evt_params_timing->conn_slave_rf_latency  = (uint16_t)*(buffer_in + OFFSET_ACI_EVT_T_TIMING + OFFSET_ACI_EVT_PARAMS_TIMING_T_CONN_SLAVE_RF_LATENCY_MSB) << 8;
  p_evt_params_timing->conn_slave_rf_latency |= (uint16_t)*(buffer_in + OFFSET_ACI_EVT_T_TIMING + OFFSET_ACI_EVT_PARAMS_TIMING_T_CONN_SLAVE_RF_LATENCY_LSB);
  p_evt_params_timing->conn_rf_timeout        = (uint16_t)*(buffer_in + OFFSET_ACI_EVT_T_TIMING + OFFSET_ACI_EVT_PARAMS_TIMING_T_CONN_RF_TIMEOUT_MSB) << 8;
  p_evt_params_timing->conn_rf_timeout       |= *(buffer_in + OFFSET_ACI_EVT_T_TIMING + OFFSET_ACI_EVT_PARAMS_TIMING_T_CONN_RF_TIMEOUT_LSB);
}

