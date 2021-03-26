/*
 * STM32 driver library for TI CC1100 Low-Power Sub-1 GHz RF Transceiver
 *
 * Based on https://github.com/loboris/ESP32_CC1101
 *
 * Modified and adapted for STM32 by: https://github.com/llunohodl  03/2021
 *
 */

#ifndef _CC1101_H
#define _CC1101_H

#include <stdint.h>

#define BROADCAST_ADDRESS         0x00  // broadcast address

// Start the operation
//  return 1 if CC1101 found or 0 on fail
uint8_t cc1101_init(uint8_t addr, uint8_t channel, int8_t power);

// Finish the operation
//      after this cc1101_init must performed for contionue operation
void cc1101_deinit();

// Reset function 
//      after this cc1101_init must performed for contionue operation
void cc1101_reset(void);

// Put into SLEEP MODE
void cc1101_powerdown(void);

// WakeUp from SLEEP MODE
void cc1101_wakeup(void);

// Send data
//      to_addr - destanation address (BROADCAST_ADDRESS for broadcast)
//      txbuffer - pointer for data buffer (length <61)
//      len - data bufer length (<61)
//      broadcast - pointer to broadcast flag,  if it not nedded, it may set to NULL
//      return 1 if tx succedded or 0 on fail
uint8_t cc1101_write(uint8_t to_addr, uint8_t *txbuffer, uint8_t len);

// Receive data
//      from_addr - pointer to store addres of sender, if it not nedded, it may set to NULL
//      rxbuffer - pointer for data buffer (length <61)
//      lenmax - data bufer length (<61)
//      waitms - [ms] time for vait paket (task delay used), for simply check without delay set to 0
//      broadcast - pointer to broadcast flag,  if it not nedded, it may set to NULL
//      return 0 if no data present or data length (<61)
uint8_t cc1101_read(uint8_t* from_addr, uint8_t *rxbuffer, uint8_t lenmax, uint32_t waitms, uint8_t* broadcast);

// Set address
void cc1101_set_addr(uint8_t addr);

// Set channel
void cc1101_set_channel(uint8_t channel);

// Set output power +10dBm max
void cc1101_set_power(int8_t dBm);

// WARNING Below functions return correct values only after cc1101 read() receive data

// Get rssi of last RX packet
int8_t cc1101_rssi();

// Get lqi of last RX packet
// The Link Quality Indicator estimates how easily a received signal can be
// demodulated. Calculated over the 64 symbols following the sync word 
uint8_t cc1101_lqi();

// Get The last CRC comparison matched. 
uint8_t cc1101_is_crc_ok();


#endif /* _CC1101_H */
