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
    
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    
    DMA1_Channel2->CPAR  = (uint32_t)(&(USART1->TDR));
    DMA1_Channel2->CMAR  = (uint32_t)(&(_tx_buffer.data[0]));
    DMA1_Channel2->CCR  &= ~(DMA_CCR_MSIZE | DMA_CCR_PSIZE | DMA_CCR_MINC | DMA_CCR_PINC | DMA_CCR_CIRC | DMA_CCR_DIR);
    DMA1_Channel2->CCR   = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_PL | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;
    
    USART1->CR3 |= USART_CR3_DMAT;
    
    NVIC_SetPriority(DMA1_Ch2_3_DMA2_Ch1_2_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Ch2_3_DMA2_Ch1_2_IRQn);
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
                        _rx_buffer.data[_rx_buffer.index++] = (byte&0x00FF); // save first byte in receiver buffer
                        
                        USART1->RTOR = cmd.n*8;
                        USART1->CR2 |= USART_CR2_RTOEN; // enable receive timeout
                        USART1->CR1 |= USART_CR1_RTOIE; // enable timeout interrupt
                    }
                    else
                        ERROR_command_inc();
                }
            }
            else if(_is_cmd) // flag command is set
            {
                _rx_buffer.data[_rx_buffer.index++] = (byte&0x00FF); // save next byte in receiver buffer
                
                if(_rx_buffer.size == _rx_buffer.index) // received all data
                {
                    _is_cmd        = false;
                    _is_data_ready = true;
                    
                    USART1->CR2 &= ~USART_CR2_RTOEN; // disable receive timeout
                    USART1->CR1 &= ~USART_CR1_RTOIE; // disable timeout interrupt
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
        
        USART1->ISR &= ~USART_ISR_TXE;
    }
    
    if((USART1->ISR & USART_ISR_RTOF) == USART_ISR_RTOF)
    {   
        _is_cmd = false;
        
        ERROR_timeout_inc(); // timeout error counter increment
        
        USART1->ICR |= USART_ICR_RTOCF; // clear flag
    }
    
    if((USART1->ISR & USART_ISR_ORE) == USART_ISR_ORE)
    {
        _is_cmd = false;
        
        ERROR_overrun_inc(); // overrun error counter increment
        
        USART1->ICR |= USART_ICR_ORECF; // clear flag
    }
}
//-----------------------------------------
void DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler(void)
{
    if((DMA1->ISR & DMA_ISR_GIF2) == DMA_ISR_GIF2)
    {
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;
        USART1->CR1        &= ~USART_CR1_TE;
        DMA1->IFCR         |= DMA_IFCR_CGIF2;
        
        _tx_buffer.size  = 0;
        _tx_buffer.index = 0;
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
        
        memcpy(&dest->data[0], (const uint16_t*)&_rx_buffer.data[0], sizeof(_rx_buffer.size)*_rx_buffer.size);
        
        return true;
    }
    
    return false;
}
//--------------------------------------
void FS9Slave_Write(FS9Buffer_t* packet)
{
    _tx_buffer.size  = packet->size;
    _tx_buffer.index = 0;
    
    memcpy((uint16_t*)&_tx_buffer.data[0], (const uint16_t*)&packet->data[0], sizeof(_tx_buffer.size)*_tx_buffer.size);
    
    DMA1_Channel2->CMAR   = (uint32_t)(&(_tx_buffer.data[0]));
    DMA1_Channel2->CNDTR  = _tx_buffer.size;
    DMA1_Channel2->CCR   |= DMA_CCR_EN;
    
    USART1->CR1 |= USART_CR1_TE;
}
