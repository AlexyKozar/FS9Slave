#include "fs9slave.h"
//-------------------
uint8_t rx_pop(void);
uint8_t tx_pop(void);
bool    rx_push(uint8_t byte);
bool    tx_push(uint8_t byte);
uint8_t rx_size(void);
uint8_t tx_size(void);
bool    rx_is_empty(void);
bool    tx_is_empty(void);
//---------------------------------
uint8_t rx_buffer[MAX_SIZE_BUF_RX];
uint8_t tx_buffer[MAX_SIZE_BUF_TX];
//----------------------------
volatile uint8_t rx_count = 0;
volatile uint8_t tx_count = 0;
volatile uint8_t rx_head  = 0;
volatile uint8_t tx_head  = 0;
volatile uint8_t rx_tail  = 0;
volatile uint8_t tx_tail  = 0;
volatile uint8_t rx_bytes = 0; // how much will be read bytes
volatile bool    is_cmd   = false;
//-----------------------------------
volatile bool FS9_Data_Ready = false;
//------------------
uint8_t rx_pop(void)
{
    uint8_t byte = 0;
    
    if(rx_count > 0)
    {
        byte = rx_buffer[rx_head++];
        
        if(rx_head == MAX_SIZE_BUF_RX)
            rx_head = 0;
        
        rx_count--;
    }
    
    return byte;
}
//------------------
uint8_t tx_pop(void)
{
    uint8_t byte = 0;
    
    if(tx_count > 0)
    {
        byte = tx_buffer[tx_head++];
        
        if(tx_head == MAX_SIZE_BUF_TX)
            tx_head = 0;
        
        tx_count--;
    }
    
    return byte;
}
//------------------------
bool rx_push(uint8_t byte)
{
    if(rx_count == MAX_SIZE_BUF_RX)
        return false;
    
    rx_buffer[rx_tail++] = byte;
    
    if(rx_tail == MAX_SIZE_BUF_RX)
        rx_tail = 0;
    
    rx_count++;
    
    return true;
}
//------------------------
bool tx_push(uint8_t byte)
{
    if(tx_count == MAX_SIZE_BUF_TX)
        return false;
    
    tx_buffer[tx_tail++] = byte;
    
    if(tx_tail == MAX_SIZE_BUF_TX)
        tx_tail = 0;
    
    tx_count++;
    
    return true;
}
//-------------------
uint8_t rx_size(void)
{
    return rx_count;
}
//-------------------
uint8_t tx_size(void)
{
    return tx_count;
}
//--------------------
bool rx_is_empty(void)
{
    return (rx_count == 0);
}
//--------------------
bool tx_is_empty(void)
{
    return (tx_count == 0);
}
//--------------------------------
bool FS9_read(FS9Packet_t* packet)
{
    if(!rx_is_empty())
    {
        packet->size = rx_size();
        
        for(uint8_t i = 0; i < packet->size; ++i)
        {
            packet->buffer[i] = rx_pop();
        }
    }
    else
        return false;
    
    FS9_Data_Ready = false;
    
    return true;
}
//---------------------------------
bool FS9_write(FS9Packet_t* packet)
{
    for(uint8_t i = 0; i < packet->size; ++i)
    {
        if(!tx_push(packet->buffer[i]))
            return false;
    }
    
    FS9_UART->CR1 |= USART_CR1_TE | USART_CR1_TXEIE;
    
    return true;
}
//---------------------
bool FS9_Is_Ready(void)
{
    return FS9_Data_Ready;
}
//-----------------------
void FS9_IRQHandler(void)
{
    uint32_t status = USART1->ISR;
    
    if(status & USART_ISR_RXNE)
    {
        uint16_t byte = USART1->RDR;

        if(!is_cmd) // is command
        {
            if((byte & CMD_MASK) == CMD_MASK)
            {
                cmd_t cmd = CMD_get(byte&CMD_CODE_MASK);
                
                rx_bytes = cmd.n; // get size packet
                
                if(rx_bytes > 0)
                {   
                    is_cmd = true;
                    rx_push((uint8_t)(byte&0xFF));
                }
            }
        }
        else
        {
            rx_push((uint8_t)(byte&0xFF));
            
            if(rx_size() == rx_bytes) // packet receiver
            {
                FS9_Data_Ready = true;
                is_cmd         = false;
            }
        }
        
        USART1->RQR |= USART_RQR_RXFRQ;
    }
    
    if(status & USART_ISR_TXE)
    {
       if(!tx_is_empty())
        {
            USART1->TDR = tx_pop();
        }
        else
        {
            USART1->CR1 &= ~(USART_CR1_TE | USART_CR1_TXEIE);
            USART1->ISR &= ~USART_ISR_TXE;
        }
    }
    
    if(status & USART_ISR_ORE)
    {
        USART1->ICR |= USART_ICR_ORECF;
    }
}
