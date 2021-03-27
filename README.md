### STM32 (with CubeMX) driver library for TI CC1100 Low-Power Sub-1 GHz RF Transceiver

Based on [loboris/ESP32_CC1101 by Boris Lovosevic](https://github.com/loboris/ESP32_CC1101)

### Build RF settings with SmartRF Studio

This library use [SmartRF Studio](https://www.ti.com/tool/SMARTRFTM-STUDIO) for configure CC1101 registers

**All RF settings must be set in SmartRF Studio**

1. Run SmartRF Studio
2. On start select CC1101 -> Open RF device in offline mode
3. Then press F6 for show Expert Mode and  press F7 for show "Register view"
4. In "Typical settings" tab select what you prefer and customize it in "RF Parameters" tab
> I select: Generic 868MHz -> Data rate: 10kBaund, Dev.: 25.4kHz, Mod.: GFSK, RX BW: 100kHz, Optimized for sensitivity
5. Change Base Frequency and other parameters
6. Then Press to "Register Export" and customize "RF settings HAL":
    - `Parameter Summary` and `PA Table` checked
    - "Comment" set to `//`
    - "Registers" set to `cc1101_spi_write_register(@RN@,0x@VH@);@<<@//@Rd@`
    - "Header" set to 
    ```
    #include "cc1101_private.h"
    #include "stdint.h"

    const uint8_t Patable[] = PA_TABLE;

    void cc1101_configure(){
    ```
    - "Footer" set to
    ```
    
        cc1101_spi_write_burst(PATABLE_BURST,Patable,8);
    }
    ```
7. After thet your see in content for your [`cc1101_rf_config.c`](cc1101_rf_config.c) in "Registers" tab

> Registers `IOCFG2`, `PKTLEN`, `PKTCTRL1`, `PKTCTRL0`, `MCSM1` force set in `cc1101_init()` for correct library work (state machine behavior, CRC append and check, variable paket length and maximum length paket) 



## Sample FreeRTOS test task
```
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cc1101.h"


uint16_t rf_tx_count = 0;
uint16_t rf_rx_count = 0;

/*
    Maximum application data length 
    (64 ( FIFO)- 1 (length) - 1 (to address) - 1 (from address)) = 61
*/
#define MAX_PACK_LEN 61 

uint8_t rf_rx[MAX_PACK_LEN];

void rfTskFunc(void const * argument){
  
  while(cc1101_init(0x00,0,10)==0){
  }
  
  uint8_t rf_tx[MAX_PACK_LEN];
  for(uint8_t i=0;i<MAX_PACK_LEN;i++){
    rf_tx[i]=(i+128-(MAX_PACK_LEN/2))^0xAA;
  }

  while(1){

#if 0 //0 on RX side | 1 on TX side
    rf_tx[0]=rf_tx_count>>8;
    rf_tx[1]=rf_tx_count&0xFF;
    rf_tx_count+=cc1101_write(BROADCAST_ADDRESS,rf_tx,16);
    osDelay(50);
#else
    memset(rf_rx,0x00,MAX_PACK_LEN);
    uint8_t len = cc1101_read(NULL,rf_rx,61,50,NULL);
    if(len == 0){
      continue;
    }
    rf_rx_count++;
    rf_tx_count=rf_rx[0]; rf_tx_count<<=8;
    rf_tx_count+=rf_rx[1];
#endif
  }
}
```