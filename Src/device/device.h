#ifndef DEVICE_H
    #define DEVICE_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include "stm32f0xx_hal.h"
    #include "cmd/cmdlist.h"
    //----------------------------
    #define MAX_SIZE_PACKET     20
    #define MAX_SIZE_PARAM_LIST 20
    #define MAX_SIZE_OUTPUT     8
    //-------------
    struct packet_t
    {
        uint8_t array[MAX_SIZE_PACKET]; // a packet bytes
        uint8_t size; // a packet size
    };
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
    /** @brief This function create response for master
        @param packet from master
        @return void
     */
    void dev_get_packet(struct packet_t* pack);
    /** @brief This function initialize output chanels
        @param an array with outputs
        @param the size array
        @return void
     */
    void dev_set_output(uint16_t* outputs, uint8_t count);
#endif
