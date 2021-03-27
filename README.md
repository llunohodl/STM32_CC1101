### STM32 (with CubeMX) driver library for TI CC1100 Low-Power Sub-1 GHz RF Transceiver

Based on [loboris/ESP32_CC1101 by Boris Lovosevic](https://github.com/loboris/ESP32_CC1101)

### Build RF settings with SmartRF Studio

This library use [SmartRF Studio](https://www.ti.com/tool/SMARTRFTM-STUDIO) for configure CC1101 registers

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
> IOCFG0-IOCFG2 force set in `cc1101_init()`