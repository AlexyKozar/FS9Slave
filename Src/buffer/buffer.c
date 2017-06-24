#include "buffer.h"
//---------------------------------
uint8_t rx_buffer[MAX_BUF_SIZE_RX]; // buffer rx
uint8_t rx_count = 0; // the byte counter
uint8_t rx_head =  0; // the head buffer
uint8_t rx_tail =  0; // the tail buffer
//----------------------------
bool rx_buf_push(uint8_t byte)
{
    if(rx_count != MAX_BUF_SIZE_RX)
    {
        rx_buffer[rx_tail++] = byte;
        
        if(rx_tail == MAX_BUF_SIZE_RX)
            rx_tail = 0;
        
        rx_count++;
    }
    else
        return false;
    
    return true;
}
//----------------------
uint8_t rx_buf_pop(void)
{
    uint8_t byte = 0;
    
    if(rx_count > 0)
    {
        byte = rx_buffer[rx_head++];
        
        if(rx_head == MAX_BUF_SIZE_RX)
            rx_head = 0;
        
        rx_count--;
    }
    
    return byte;
}
//-----------------------
uint8_t rx_buf_size(void)
{
    return rx_count;
}
//------------------------
uint8_t rx_buf_front(void)
{
    uint8_t byte = 0;
    
    if(rx_count > 0)
        byte = rx_buffer[rx_head];
    
    return byte;
}
//----------------------
uint8_t rx_buf_end(void)
{
    uint8_t byte = 0;
    
    if(rx_count > 0)
        byte = rx_buffer[rx_tail - 1];
    
    return byte;
}
