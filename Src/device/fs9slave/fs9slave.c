#include "fs9slave.h"
//------------------------------------------
volatile FS9Buffer_t _rx_buffer     = { 0 }; // receive buffer
volatile FS9Buffer_t _tx_buffer     = { 0 }; // transmission buffer
volatile bool        _is_cmd        = false; // is command or not
volatile bool        _is_data_ready = false;
volatile uint8_t     _address       = 0xFF; // default device address
//---------------------------------
void FS9Slave_Init(uint8_t address)
{
    _address = address;
}
//--------------------------
void USART1_IRQHandler(void)
{
    if((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) // read data
    {
        uint16_t byte = USART1->RDR;
        
        if(_address != 0xFF) // device address is valid
        {
            if(!_is_cmd && (byte&CMD_MASK) == CMD_MASK) // command yet is not finded and byte contains is command bit
            {
                uint8_t address = (uint8_t)((byte&DEV_ADDR_MASK) >> 6);
                
                if(_address == address) // received address is device address
                {
                    uint8_t cmd_code = ((uint8_t)(byte&0x00FF))&CMD_CODE_MASK; // get command code from received byte
                    cmd_t   cmd      = CMD_get(cmd_code); // get command data from list command valid
                    
                    if(cmd.n > 0)
                    {
                        _is_cmd             = true;
                        _rx_buffer.cmd_code = cmd_code; // save command code
                        _rx_buffer.cmd      = cmd; // save command data
                        _rx_buffer.size     = cmd.n; // save command size
                        _rx_buffer.index    = 0; // index clear
                        _rx_buffer.data[_rx_buffer.index++] = ((uint8_t)(byte&0x00FF)); // save first byte in receiver buffer
                        
//                        USART1->RTOR = 2;
//                        USART1->CR2 |= USART_CR2_RTOEN; // enable receive timeout
//                        USART1->CR1 |= USART_CR1_RTOIE; // enable timeout interrupt
                    }
                    else
                        ERROR_command_inc();
                }
            }
            else if(_is_cmd) // flag command is set
            {
                _rx_buffer.data[_rx_buffer.index++] = ((uint8_t)(byte&0x00FF)); // save next byte in receiver buffer
                
                if(_rx_buffer.size == _rx_buffer.index) // received all data
                {
                    _is_cmd        = false;
                    _is_data_ready = true;
                    
//                    USART1->CR2 &= ~USART_CR2_RTOEN; // disable receive timeout
//                    USART1->CR1 &= ~USART_CR1_RTOIE; // disable timeout interrupt
                }
            }
        }
        
        USART1->RQR |= USART_RQR_RXFRQ; // flag clear
    }
    
    if((USART1->ISR & USART_ISR_TXE) == USART_ISR_TXE)
    {
        if(_tx_buffer.index < _tx_buffer.size)
        {
            USART1->TDR = _tx_buffer.data[_tx_buffer.index++];
        }
        else
        {
            USART1->CR1 &= ~(USART_CR1_TE | USART_CR1_TXEIE);
        }
        
        USART1->ISR &= !USART_ISR_TXE;
    }
    
//    if((USART1->ISR & USART_ISR_RTOF) == USART_ISR_RTOF)
//    {   
//        _is_cmd = false;
//        
//        ERROR_timeout_inc(); // timeout error counter increment
//        
//        USART1->ICR |= USART_ICR_RTOCF; // clear flag
//    }
    
    if((USART1->ISR & USART_ISR_ORE) == USART_ISR_ORE)
    {
        _is_cmd = false;
        
        ERROR_overrun_inc(); // overrun error counter increment
        
        USART1->ICR |= USART_ICR_ORECF; // clear flag
    }
}
//-------------------------
bool FS9Slave_IsReady(void)
{
    return _is_data_ready;
}
//-----------------------------------
bool FS9Slave_Read(FS9Buffer_t* dest)
{
    if(_rx_buffer.size > 0)
    {
        dest->size     = _rx_buffer.size;
        dest->cmd_code = _rx_buffer.cmd_code;
        dest->cmd      = _rx_buffer.cmd;
        _is_data_ready = false;
        
        memcpy(&dest->data[0], (const uint8_t*)&_rx_buffer.data[0], _rx_buffer.size);
        
        return true;
    }
    
    return false;
}
//--------------------------------------
void FS9Slave_Write(FS9Buffer_t* packet)
{
    _tx_buffer.size  = packet->size;
    _tx_buffer.index = 0;
    
    memcpy((uint8_t*)&_tx_buffer.data[0], (const uint8_t*)&packet->data[0], _tx_buffer.size);
    
    USART1->CR1 |= USART_CR1_TE | USART_CR1_TXEIE;
    
    USART1->TDR = _tx_buffer.data[_tx_buffer.index++];
}
