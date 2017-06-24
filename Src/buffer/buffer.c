#include "buffer.h"
//---------------------------------
uint8_t rx_buffer[MAX_BUF_SIZE_RX]; // buffer rx
uint8_t tx_buffer[MAX_BUF_SIZE_TX]; // buffer tx
uint8_t rx_count = 0; // the byte counter rx
uint8_t rx_head =  0; // the head buffer rx
uint8_t rx_tail =  0; // the tail buffer rx
uint8_t tx_count = 0; // the byte counter tx
uint8_t tx_head =  0; // the head buffer tx
uint8_t tx_tail =  0; // the tail buffer tx
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
//------------------------
bool rx_buf_is_empty(void)
{
    if(rx_count == 0)
        return true;
    
    return false;
}
//-----------------------
bool rx_buf_is_full(void)
{
    if(rx_count == MAX_BUF_SIZE_RX)
        return true;
    
    return false;
}
//----------------------------
bool tx_buf_push(uint8_t byte)
{
    if(tx_count != MAX_BUF_SIZE_TX)
    {
        tx_buffer[tx_tail++] = byte;
        
        if(tx_tail == MAX_BUF_SIZE_TX)
            tx_tail = 0;
        
        tx_count++;
    }
    else
        return false;
    
    return true;
}
//----------------------
uint8_t tx_buf_pop(void)
{
    uint8_t byte = 0;
    
    if(tx_count > 0)
    {
        byte = tx_buffer[tx_head++];
        
        if(tx_head == MAX_BUF_SIZE_TX)
            tx_head = 0;
        
        tx_count--;
    }
    
    return byte;
}
//-----------------------
uint8_t tx_buf_size(void)
{
    return tx_count;
}
//------------------------
uint8_t tx_buf_front(void)
{
    uint8_t byte = 0;
    
    if(tx_count > 0)
        byte = tx_buffer[tx_head];
    
    return byte;
}
//----------------------
uint8_t tx_buf_end(void)
{
    uint8_t byte = 0;
    
    if(tx_count > 0)
        byte = tx_buffer[tx_tail - 1];
    
    return byte;
}
//------------------------
bool tx_buf_is_empty(void)
{
    if(tx_count == 0)
        return true;
    
    return false;
}
//-----------------------
bool tx_buf_is_full(void)
{
    if(tx_count == MAX_BUF_SIZE_TX)
        return true;
    
    return false;
}
