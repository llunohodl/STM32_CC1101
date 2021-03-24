/*
 * STM32 driver library for TI CC1100 Low-Power Sub-1 GHz RF Transceiver
 *
 * Based on https://github.com/loboris/ESP32_CC1101
 *
 * Modified and adapted for STM32 by: https://github.com/llunohodl  03/2021
 *
 */

#include "cmsis_os.h"
#include "main.h" /* CubeMX defines */

#include "cc1101_private.h"

// Not optimized RAM function
#pragma optimize=none
void __ramfunc cc1101_delayMicroseconds(uint32_t us){
  //Calibrated on 100us pulse for Cortex-M0 FCPU 48 000 000
  __disable_irq();
  for(uint32_t i=0;i<us;i++) {
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
  }
  __enable_irq();
}



// Milli second delay
//---------------------------------
void cc1101_delay(int ms)
{
	osDelay(ms);
}

void cc1101_ss_pin_set(){
  RF_nCS_GPIO_Port->BSRR = (uint32_t)RF_nCS_Pin;
}

void cc1101_ss_pin_reset(){
  RF_nCS_GPIO_Port->BRR = (uint32_t)RF_nCS_Pin;
}

// Run this function in any TaskLoop and abjust number of call asm("nop"); in 
// cc1101_delayMicroseconds for get 100us pulse on nSS pin
void delay_calibration_loop_100us(){
  cc1101_ss_pin_set();
  cc1101_delayMicroseconds(100);
  cc1101_ss_pin_reset();
  osDelay(2);
}


// =================== SPI Functions ===================

//==== SPI Initialization for CC1100 ====
uint8_t cc1101_spi_begin(void){
  /* All settings done in CubeMX */
  cc1101_ss_pin_set();
  return 0;
}

//==== De initialize the SPI interface ===
//-----------------------
void cc1101_spi_end(void){
	/* Is it need ? */
}

/* Pin and Port names from CubeMX
  RF_nCS_Pin
  RF_nCS_GPIO_Port
  RF_MISO_Pin
  RF_MISO_GPIO_Port
*/

extern SPI_HandleTypeDef hspi2; /* CubeMX defines */

//==== SPI Transmission ====
//---------------------------------------------------------
void cc1101_spi_send(uint8_t *data, uint32_t len)
{
    cc1101_ss_pin_reset();
    /*
      When the header byte, data byte, or command	strobe is sent on the SPI interface,
      the chip status byte is sent by the CC1101 on the SO pin.
      The status byte contains key status signals, useful for the MCU.
      The first bit, s7, is the CHIP_RDYn signal and this signal must go low
      before the first positive edge of SCLK.
      The	CHIP_RDYn signal indicates that the crystal is running.
	*/
    while(RF_MISO_GPIO_Port->IDR&RF_MISO_Pin){
      //TODO: add timeout here
    }
    
    HAL_SPI_TransmitReceive(&hspi2,data,data,len,30);
	cc1101_ss_pin_set();
}


// Write strobe command
//-------------------------------------------------------
void cc1101_spi_write_strobe(uint8_t spi_instr)
{
	uint8_t instr = spi_instr;
    cc1101_spi_send(&instr, 1);
}

// Read one Register
//-----------------------------------------------------------
uint8_t cc1101_spi_read_register(uint8_t spi_instr)
{
	uint8_t buf[2];
	buf[0] = spi_instr | READ_SINGLE_BYTE;
	buf[1] = 0xFF;

	cc1101_spi_send(buf, 2);
    return buf[1];
}

// Write one Register
//------------------------------------------------------------------------
void cc1101_spi_write_register(uint8_t spi_instr, uint8_t value)
{
	uint8_t buf[2];
	buf[0] = spi_instr | WRITE_SINGLE_BYTE;
	buf[1] = value;

	cc1101_spi_send(buf, 2);
}

// Write several consecutive registers at once
//-----------------------------------------------------------------------------------------------+------
void cc1101_spi_write_burst(uint8_t spi_instr, uint8_t *pArr, uint8_t length, uint8_t *rdback)
{
    if(length>128) length = 128;
	uint8_t buf[129];
	buf[0] = spi_instr | WRITE_BURST;
	memcpy(buf+1, pArr, length);

    cc1101_spi_send(buf, length+1);
    if (rdback) {
    	memcpy(rdback, buf+1, length);
    }
}

/* Pin and Port names from CubeMX
    RF_GIO2_Pin
    RF_GIO2_GPIO_Port
*/

// Check if Packet is received
//========================
uint8_t cc1101_packet_available()
{
  #ifdef RF_GIO2_Pin  //Interrupt Driven Solution: check GPIO pin
    return (RF_GIO2_GPIO_Port->IDR&RF_GIO2_Pin)!=0;
  #else  //SPI Polling: check GPIO pin in  PKTSTATUS
    return (cc1101_spi_read_register(PKTSTATUS)&(1<<2))!=0;
  #endif
}


