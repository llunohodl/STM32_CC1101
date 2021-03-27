/*
 * STM32 driver library for TI CC1100 Low-Power Sub-1 GHz RF Transceiver
 *
 * Based on https://github.com/loboris/ESP32_CC1101
 *
 * Modified and adapted for STM32 by: https://github.com/llunohodl  03/2021
 *
 */

#include <string.h>
#include <stdint.h>
#include "cc1101.h"
#include "cc1101_private.h"

static int8_t last_rssi_dbm = 0;
static uint8_t last_lqi = 0;
static uint8_t last_crc = 0;
static uint8_t self_addr = 0;



// Set idle mode
//=================
static uint8_t cc1101_idle(void)
{
    uint8_t marcstate = (cc1101_spi_read_register(MARCSTATE) & 0x1F);
    if(marcstate == 0x01){
        return 1;
    }else if(marcstate == 0x11){ // RXFIFO_OVERFLOW  
        //  RXFIFO_OVERFLOW->IDLE
        cc1101_spi_write_strobe(SFRX); //Flash RX FIFO and go to IDLE
    }else if(marcstate == 0x16){ //TXFIFO_UNDERFLOW
        //  TXFIFO_UNDERFLOW->IDLE
        cc1101_spi_write_strobe(SFTX); //Flash TX FIFO and go to IDLE
    }else{
        //  any state->IDLE
        cc1101_spi_write_strobe(SIDLE);
    }
    uint8_t times = 0;
    do{
        cc1101_delay(1);
        marcstate = (cc1101_spi_read_register(MARCSTATE) & 0x1F);        
        times++;
        if((times&0x03) == 0){ 
          cc1101_spi_write_strobe(SIDLE);
          cc1101_delay(1);
        }
    }while(marcstate != 0x01); //0x01 = cc1101_idle
    return 1;
}

// Receive mode
//===================
static uint8_t cc1101_receive(void)
{
    uint8_t marcstate = (cc1101_spi_read_register(MARCSTATE) & 0x1F);
    if(marcstate == 0x0D){       // RX
        return 1;
    }else{
        //  any state->IDLE
        cc1101_idle();
    }
    //Start RX: IDLE->FS_WAKEUP->(CALIBRATE)->SETTLING->RX
    cc1101_spi_write_strobe(SRX);	
    uint8_t times = 0;
    do{
    	cc1101_delay(1);
        marcstate = (cc1101_spi_read_register(MARCSTATE) & 0x1F);
        times++;
        if((times&0x03) == 0){
          cc1101_spi_write_strobe(SRX);
          cc1101_delay(1);
        }
    }while (marcstate != 0x0D);	//0x0D = RX
    return 1;
}



// CC1101 reset function
// Reset as defined in cc1100 data sheet
//==============
void cc1101_reset(void)
{
    cc1101_ss_pin_reset();
    cc1101_delayMicroseconds(10);
    cc1101_ss_pin_set();
    cc1101_delayMicroseconds(40);

    cc1101_spi_write_strobe(SRES);
    cc1101_delay(1);
}

// Set Power Down
// Put CC1101 into SLEEP MODE
//==================
void cc1101_powerdown(void){
    cc1101_idle();
    cc1101_spi_write_strobe(SPWD); // CC1100 Power Down
}

// WakeUp from SLEEP MODE
//===============
void cc1101_wakeup(void){
    cc1101_ss_pin_reset();
    cc1101_delayMicroseconds(10);
    cc1101_ss_pin_set();
    cc1101_delayMicroseconds(10);
    cc1101_receive();  // go to RX Mode
}


// Start operation
uint8_t cc1101_init(uint8_t addr, uint8_t channel, int8_t power)
{
    //initialize SPI Interface
    if (cc1101_spi_begin() != 0) {
    	return 0;
    }

    // CC1100 initial reset
    cc1101_reset();

    cc1101_spi_write_strobe(SFTX); 
    cc1101_delayMicroseconds(100); //flush the TX_fifo content
    cc1101_spi_write_strobe(SFRX); 
    cc1101_delayMicroseconds(100); //flush the RX_fifo content

    uint8_t partnum = cc1101_spi_read_register(PARTNUM); //reads CC1100 part number
    uint8_t version = cc1101_spi_read_register(VERSION); //reads CC1100 version number

    //Checks if valid Chip ID is found. Usually 0x03 or 0x14. if not -> abort
    if (version == 0x00 || version == 0xFF){
            // no CC11xx found!
            cc1101_deinit();
            return 0;
    }

    cc1101_configure();
    
    //Set setings for correct library work 
    //      potential rewriing default settings from SmartRF Studio
    
    //GDO2 Asserts (goes to high) when: 
    //      a packet has been received with CRC OK. De-asserts when the first byte is read from the RX FIFO
    cc1101_spi_write_register(IOCFG2,0x07);
    
    //Packet Length
    //       PKTLEN.PACKET_LENGTH=64 (maximum)
    cc1101_spi_write_register(PKTLEN,64);
    
    // Packet Automation Control 1
    //      (0<<5) PKTCTRL1.PQT[2:0]  -> A sync word is always accepted.
    //      (1<<3) PKTCTRL1.CRC_AUTOFLUSH  -> 
    //            Automatic flush of RX FIFO when CRC is not OK. This requires that
    //            only one packet is in the RXIFIFO and that packet length is limited to the
    //            RX FIFO size.
    //      (0<<2) PKTCTRL1.APPEND_STATUS  -> Disable append status to end of FIFO
    //      (2<<0) PKTCTRL1.LENGTH_CONFIG[1:0] -> Address check and 0 (0x00) broadcast
    cc1101_spi_write_register(PKTCTRL1,0x0A);
    
    // Packet Automation Control 0
    //      (1<<6) PKTCTRL0.WHITE_DATA -> Whitening on
    //      (0<<4) PKTCTRL0.PKT_FORMAT[1:0] -> Normal mode, use FIFOs for RX and TX
    //      (1<<2) PKTCTRL0.CRC_EN -> 
    //            CRC calculation in TX and CRC check in RX enabled
    //      (1<<0) PKTCTRL0.LENGTH_CONFIG[1:0] -> 
    //            Variable packet length mode. Packet length configured by the first byte after sync word
    cc1101_spi_write_register(PKTCTRL0,0x45);
    
    //Main Radio Control State Machine Configuration 1
    //      what should happen when a packet has been received: (0x00<<2) IDLE
    //      what should happen when a packet has been sent (TX): (0x03<<0) RX
    cc1101_spi_write_register(MCSM1,0x03);   

    //set channel
    cc1101_set_channel(channel);

    //set output power amplifier
    cc1101_set_power(power);

    //set my receiver address 'My_Addr' from NVS to global variable
    cc1101_set_addr(addr);
    self_addr = addr;

    cc1101_idle();                          //set to ILDE first
	cc1101_set_power(0);        //set PA level in dbm


    
    //set to RECEIVE mode
    cc1101_receive();

    return 1;
}


// Finish the CC1100 operation
//===============
void cc1101_deinit()
{
    cc1101_powerdown();	//power down CC1100
    cc1101_spi_end();		//disable SPI Interface
}



// Send packet
uint8_t cc1101_write(uint8_t to_addr, uint8_t *txbuffer, uint8_t len)
{
    uint8_t buff[65];
    if(len>64-3) len = 64-3;
    buff[0]=TXFIFO_BURST;
    buff[1]=len+2;        //Length of 2 adress + payload
    buff[2]=to_addr;      //To address
    buff[3]=self_addr;    //From address
    memcpy(buff+4,txbuffer,len);
    cc1101_spi_send(buff, len+4);
    //send data over air
    uint8_t marcstate = 0xFF;
    cc1101_spi_write_strobe(STX);		//sends the data over air
    uint8_t timeout = 88; //110bytes on 10kBod 
    while(1){
    	cc1101_delay(1);
    	marcstate = (cc1101_spi_read_register(MARCSTATE) & 0x1F);
        if (marcstate == 0x16 || timeout == 0) { //0x16 = TXFIFO_UNDERFLOW
        	//flush the TX_fifo content
        	cc1101_spi_write_strobe(SFTX);
            cc1101_delayMicroseconds(100);
            cc1101_receive(); //receive mode
            return 0; 
        }
        if(marcstate == 0x0D){ // 0x0D = RX (MCSM1&0x03 == 0x03)
            //Tx complete
            return 1;
        }
        timeout--;
    }
    //cc1101_receive(); //receive mode
    return 0; 
}

static void cc1101_get_rx_info()
{
    last_rssi_dbm = 0;
    int16_t Rssi_dec = cc1101_spi_read_register(RSSI);
    if (Rssi_dec >= 128) last_rssi_dbm=((Rssi_dec-256)/2)-RSSI_OFFSET;
    else if (Rssi_dec<128) last_rssi_dbm=((Rssi_dec)/2)-RSSI_OFFSET;

    uint8_t lqi = cc1101_spi_read_register(LQI);
    last_lqi = lqi&0x7F;
    last_crc = lqi&0x80 ? 1 : 0;
}

// Receive data

uint8_t cc1101_read(uint8_t* from_addr, uint8_t *rxbuffer, uint8_t lenmax, uint32_t waitms, uint8_t* broadcast)
{
    cc1101_receive(); //Swith to RX mode (only if needed)
    uint8_t len = 0;
    do{
        if (cc1101_packet_available() == 1)                        //if RF package received check package acknowledge
        {
            len = cc1101_spi_read_register(RXBYTES);
            if(len&0x80){ //RXFIFO_OVERFLOW
                cc1101_spi_write_strobe(SFRX);
                cc1101_delayMicroseconds(100);
                cc1101_receive();
            }else if ((len&0x7F)!=0){
                if(len>64) len=64;
                uint8_t buff[65];
                buff[0]=RXFIFO_BURST;
                memset(buff+1,0xFF,len);
                cc1101_spi_send(buff,len+1);
                //cc1101_get_rx_info();
                cc1101_receive();
                if(buff[1]!=len-1){ //Wrong Len
                  len=0;
                  continue;
                }
                if(broadcast){ *broadcast = buff[2]!=self_addr; }
                if(from_addr){ *from_addr = buff[3]; } 
                len=buff[1]-2;
                if(len>lenmax+3) len=lenmax+3;
                memcpy(rxbuffer,buff+4,len);
                return len;
            }   
        }
        if(waitms>=10){
            cc1101_delay(10);
            waitms-=10;
        }else{
            cc1101_delay(waitms);
            waitms=0;
        }
    } while (waitms);               
    return 0;  //No data                                                 
}

// Set address
void cc1101_set_addr(uint8_t addr)
{
    //store Address in the CC1100
    cc1101_spi_write_register(ADDR,addr);
}

// Set channel
void cc1101_set_channel(uint8_t channel)
{
	//store the new channel # 
    cc1101_spi_write_register(CHANNR,channel);
}

// Set output power
extern const uint8_t Patable[];
void cc1101_set_power(int8_t dBm)
{
    //Save FREND0.LODIV_BUF_CURRENT_TX set from SmartRF Studio
    //PA for -30dBm or lower
    uint8_t pa = cc1101_spi_read_register(FREND0)&0xF8; 
    
    if (dBm <= -20)      pa |= 0x01;
    else if (dBm <= -15) pa |= 0x02;
    else if (dBm <= -10) pa |= 0x03;
    else if (dBm <= 0)   pa |= 0x04;
    else if (dBm <= 5)   pa |= 0x05;
    else if (dBm <= 7)   pa |= 0x06;
    else                 pa |= 0x07;

    cc1101_spi_write_register(FREND0,pa);
}


// Get rssi of last RX packet
int8_t cc1101_rssi(){
    return last_rssi_dbm;
}

// Get lqi of last RX packet
// The Link Quality Indicator estimates how easily a received signal can be
// demodulated. Calculated over the 64 symbols following the sync word 
uint8_t cc1101_lqi(){
    return last_lqi;
}

// Get The last CRC comparison matched. 
uint8_t cc1101_is_crc_ok(){
    return last_crc;
}


