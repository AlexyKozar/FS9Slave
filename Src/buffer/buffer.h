#ifndef BUFFER_H
    #define BUFFER_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    //------------------------
    #define MAX_BUF_SIZE_RX 20
    
    /** @brief This function append byte to ring buffer RX
        @param Byte for append to ring buffer RX
        @return Returned true if append byte to buffer successfully
     */
    bool rx_buf_push(uint8_t byte);
    /** @brief This function returned byte from buffer with removing
        @return Returned byte
     */
    uint8_t rx_buf_pop(void);
    /** @brief This function returned size the ring buffer rx
        @return Returned size
     */
    uint8_t rx_buf_size(void);
    /** @brief This function returned front byte from ring buffer rx without removing
        @return Returned byte
     */
    uint8_t rx_buf_front(void);
    /** @brief This function returned end byte from ring buffer rx without removing
        @return Returned byte
     */
    uint8_t rx_buf_end(void);
#endif
