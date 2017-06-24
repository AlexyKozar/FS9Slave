#ifndef DEVICE_H
    #define DEVICE_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include "cmd/cmdlist.h"
    //-----------------------------------------------
    /** @brief This function for set a device address
        @param address device
        @return void
     */
    void    dev_set_addr(uint8_t addr);
    /** @brief This function for get a device address
        @param void
        @return address a device (default value 0xFF)
     */
    uint8_t dev_get_addr(void);
    /** @brief This function for get a cmd size packet
        @param cmd id
        @return a size packet
     */
    uint8_t dev_get_size_packet(uint8_t cmd);
#endif
