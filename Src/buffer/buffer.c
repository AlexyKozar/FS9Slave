#include "buffer.h"
//-----------------------------
struct ring_buffer_t buffer_rx = { 0, 0, 0, 0 };
struct ring_buffer_t buffer_tx = { 0, 0, 0, 0 };
//-----------------------------------
bool buf_push(uint8_t byte, bool dir)
{
    struct ring_buffer_t* buffer;
    
    if(dir)
        buffer = &buffer_rx;
    else
        buffer = &buffer_tx;
    
    if(buffer->count != MAX_BUF_SIZE)
    {
        buffer->buffer[buffer->tail++] = byte;
        
        if(buffer->tail == MAX_BUF_SIZE)
            buffer->tail = 0;
        
        buffer->count++;
    }
    else
        return false;
    
    return true;
}
//-----------------------
uint8_t buf_pop(bool dir)
{
    struct ring_buffer_t* buffer;
    uint8_t byte = 0xFF;
    
    if(dir)
        buffer = &buffer_rx;
    else
        buffer = &buffer_tx;
    
    if(buffer->count > 0)
    {
        byte = buffer->buffer[buffer->head++];
        
        if(buffer->head == MAX_BUF_SIZE)
            buffer->head = 0;
        
        buffer->count--;
    }
    
    return byte;
}
//----------------------------
uint8_t buf_end_byte(bool dir)
{
    struct ring_buffer_t* buffer;
    uint8_t byte = 0xFF;
    
    if(dir)
        buffer = &buffer_rx;
    else
        buffer = &buffer_tx;
    
    if(buffer->count > 0)
    {
        byte = buffer->buffer[buffer->tail - 1];
    }
    
    return byte;
}
//------------------------------
uint8_t buf_front_byte(bool dir)
{
    struct ring_buffer_t* buffer;
    uint8_t byte = 0xFF;
    
    if(dir)
        buffer = &buffer_rx;
    else
        buffer = &buffer_tx;
    
    if(buffer->count > 0)
    {
        byte = buffer->buffer[buffer->head];
    }
    
    return byte;
}
//-------------------------
bool buf_is_empty(bool dir)
{
    struct ring_buffer_t* buffer;
    
    if(dir)
        buffer = &buffer_rx;
    else
        buffer = &buffer_tx;
    
    if(buffer->count == 0)
        return true;
    
    return false;
}
//------------------------
bool buf_is_full(bool dir)
{
    struct ring_buffer_t* buffer;
    
    if(dir)
        buffer = &buffer_rx;
    else
        buffer = &buffer_tx;
    
    if(buffer->count == MAX_BUF_SIZE)
        return true;
    
    return false;
}
//----------------------
void buf_clear(bool dir)
{
    struct ring_buffer_t* buffer;
    
    if(dir)
        buffer = &buffer_rx;
    else
        buffer = &buffer_tx;
    
    buffer->head = buffer->tail = buffer->count = 0;
}
//------------------------
uint8_t buf_size(bool dir)
{
    struct ring_buffer_t* buffer;
    
    if(dir)
        buffer = &buffer_rx;
    else
        buffer = &buffer_tx;
    
    return buffer->count;
}
