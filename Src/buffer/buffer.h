#ifndef BUFFER_H
    #define BUFFER_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    //------------------------
    #define MAX_BUF_SIZE_RX 20
    #define MAX_BUF_SIZE_TX 20
    //----------------------------------------------------
    /** @brief This function append byte to ring buffer RX
        @param Byte for append to ring buffer RX
        @return Returned true if append byte to buffer successfully
     */
    bool rx_buf_push(uint8_t byte);
    /** @brief This function returned byte from ring buffer RX with removing
        @return Returned byte
     */
    uint8_t rx_buf_pop(void);
    /** @brief This function returned size the ring buffer RX
        @return Returned size
     */
    uint8_t rx_buf_size(void);
    /** @brief This function returned front byte from ring buffer RX without removing
        @return Returned byte
     */
    uint8_t rx_buf_front(void);
    /** @brief This function returned end byte from ring buffer RX without removing
        @return Returned byte
     */
    uint8_t rx_buf_end(void);
    /** @brief This function returned true if the ring buffer RX is empty
        @return Returned true if the ring buffer RX is empty
     */
    bool rx_buf_is_empty(void);
    /** @brief This function returned true if the ring buffer RX is full
        @return Returned true if the ring buffer RX is full
     */
    bool rx_buf_is_full(void);
    //----------------------------------------------------
    /** @brief This function append byte to ring buffer TX
        @param Byte for append to ring buffer RX
        @return Returned true if append byte to buffer successfully
     */
    bool tx_buf_push(uint8_t byte);
    /** @brief This function returned byte from ring buffer TX with removing
        @return Returned byte
     */
    uint8_t tx_buf_pop(void);
    /** @brief This function returned size the ring buffer TX
        @return Returned size
     */
    uint8_t tx_buf_size(void);
    /** @brief This function returned front byte from ring buffer TX without removing
        @return Returned byte
     */
    uint8_t tx_buf_front(void);
    /** @brief This function returned end byte from ring buffer TX without removing
        @return Returned byte
     */
    uint8_t tx_buf_end(void);
    /** @brief This function returned true if the ring buffer TX is empty
        @return Returned true if the ring buffer TX is empty
     */
    bool tx_buf_is_empty(void);
    /** @brief This function returned true if the ring buffer TX is full
        @return Returned true if the ring buffer TX is full
     */
    bool tx_buf_is_full(void);
#endif
