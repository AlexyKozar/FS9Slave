#ifndef BUFFER_H
    #define BUFFER_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    //---------------------
    #define MAX_BUF_SIZE 20
    //--------------------------
    #define BUF_TX (uint8_t)0x00
    #define BUF_RX (uint8_t)0x01
    //-----------------------------------
    /** @bief  This structure ring buffer
        @field buffer This field buffer array
        @field head This field begin data in buffer
        @field tail This field end data in buffer
        @field count This field number bytes in buffer
     */
    struct ring_buffer_t
    {
        uint8_t buffer[MAX_BUF_SIZE];
        uint8_t head;
        uint8_t tail;
        uint8_t count;
    };
    //-----------------------------------------------
    /** @bief This function for push a byte in buffer
        @param This byte for push in buffer
        @param This bool value - choice buffer (RX - true or TX)
        @return Return true if push byte in buffer else false
     */
    bool buf_push(uint8_t byte, bool dir);
    //--------------------------------------------------------------
    /** @bief This function for get byte and remove byte from buffer
        @param This bool value - choice buffer (RX - true or TX)
        @return Return byte from buffer (0xFF if buffer is empty)
     */
    uint8_t buf_pop(bool dir);
    //---------------------------------------
    /** @brief This function return last byte
        @param This bool value - choice buffer (RX - true or TX)
        @return Return last byte (0xFF if buffer is empty)
     */
    uint8_t buf_end_byte(bool dir);
    //----------------------------------------
    /** @brief This function return first byte
        @param This bool value - choice buffer (RX - true or TX)
        @return Return firs byte (0xFF if buffer is empty)
     */
    uint8_t buf_front_byte(bool dir);
    //-----------------------------------------------------
    /** @brief This function determin buffer is empty or no
        @param This bool value - choice buffer (RX - true or TX)
        @return Return true if buffer is empty
     */
    bool buf_is_empty(bool dir);
    //----------------------------------------------------
    /** @brief This function determin buffer is full or no
        @param This bool value - choice buffer (RX - true or TX)
        @return Return true if buffer is full
     */
    bool buf_is_full(bool dir);
    //-----------------------------------
    /** @brief This function clear buffer
        @param This bool value - choice buffer (RX - true or TX)
     */
    void buf_clear(bool dir);
    //-------------------------------------------
    /** @brief This function determin size buffer
        @param This bool value - choice buffer (RX - true or TX)
        @return Return size buffer
     */
    uint8_t buf_size(bool dir);
#endif
