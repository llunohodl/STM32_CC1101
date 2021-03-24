// Address Config = No address check 
// Base Frequency = 433.149780 
// CRC Autoflush = false 
// CRC Enable = true 
// Carrier Frequency = 433.149780 
// Channel Number = 0 
// Channel Spacing = 199.951172 
// Data Format = Normal mode 
// Data Rate = 9.9926 
// Deviation = 19.042969 
// Device Address = 0 
// Manchester Enable = true 
// Modulated = true 
// Modulation Format = GFSK 
// PA Ramping = true 
// Packet Length = 255 
// Packet Length Mode = Variable packet length mode. Packet length configured by the first byte after sync word 
// Preamble Count = 4 
// RX Filter BW = 101.562500 
// Sync Word Qualifier Mode = 30/32 sync word bits detected 
// TX Power = 10 
// Whitening = false 
// PA table 
#define PA_TABLE {0x00,0x12,0x0e,0x34,0x60,0xc5,0xc1,0xc0}

#include "cc1101_private.h"
#include "stdint.h"
#include "stddef.h"

uint8_t Patable[] = PA_TABLE;

void cc1101_configure(){
    cc1101_spi_write_register(IOCFG0,0x06);  //GDO0 Output Pin Configuration
    cc1101_spi_write_register(FIFOTHR,0x47); //RX FIFO and TX FIFO Thresholds
    cc1101_spi_write_register(PKTCTRL0,0x05);//Packet Automation Control
    cc1101_spi_write_register(FSCTRL1,0x06); //Frequency Synthesizer Control
    cc1101_spi_write_register(FREQ2,0x10);   //Frequency Control Word, High Byte
    cc1101_spi_write_register(FREQ1,0xA8);   //Frequency Control Word, Middle Byte
    cc1101_spi_write_register(FREQ0,0xDC);   //Frequency Control Word, Low Byte
    cc1101_spi_write_register(MDMCFG4,0xC8); //Modem Configuration
    cc1101_spi_write_register(MDMCFG3,0x93); //Modem Configuration
    cc1101_spi_write_register(MDMCFG2,0x1B); //Modem Configuration
    cc1101_spi_write_register(DEVIATN,0x34); //Modem Deviation Setting
    cc1101_spi_write_register(MCSM0,0x18);   //Main Radio Control State Machine Configuration
    cc1101_spi_write_register(FOCCFG,0x16);  //Frequency Offset Compensation Configuration
    cc1101_spi_write_register(AGCCTRL2,0x43);//AGC Control
    cc1101_spi_write_register(WORCTRL,0xFB); //Wake On Radio Control
    cc1101_spi_write_register(FREND0,0x17);  //Front End TX Configuration
    cc1101_spi_write_register(FSCAL3,0xE9);  //Frequency Synthesizer Calibration
    cc1101_spi_write_register(FSCAL2,0x2A);  //Frequency Synthesizer Calibration
    cc1101_spi_write_register(FSCAL1,0x00);  //Frequency Synthesizer Calibration
    cc1101_spi_write_register(FSCAL0,0x1F);  //Frequency Synthesizer Calibration
    cc1101_spi_write_register(TEST2,0x81);   //Various Test Settings
    cc1101_spi_write_register(TEST1,0x35);   //Various Test Settings
    cc1101_spi_write_register(TEST0,0x09);   //Various Test Settings


    cc1101_spi_write_burst(PATABLE_BURST,Patable,8, NULL);
}