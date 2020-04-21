#include "io.h"
//-------------------------------------------------
uint32_t _io_samples[DINPUT_MAX_SIZE /*<< 1*/] = { 0 }; // samples buffer (double)
volatile uint8_t  _io_count = 0; // io count
volatile uint8_t  _io_sample_count = 0; // current sample
volatile bool     _io_data_is_ready = false; // flag data is ready
volatile uint8_t  _io_dev_addr = 0xFF; // device address
volatile uint16_t _io_sequence_count = DINPUT_SAMPLE_PERIOD; // sequence samples count
//----------------------------
void IO_TIM_Init(uint8_t addr)
{
    if(addr == 0x02) // device address is MIK-01
        return;
    
    _io_dev_addr = addr;
    
    if(_io_dev_addr == 0x00) // device address is MDVV-01
        _io_count = 12;
    else if(_io_dev_addr == 0x01) // device address is MDVV-02
        _io_count = 10;
    
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
    
    TIM16->PSC = F_CPU/1000000UL - 1;
    TIM16->ARR = 1000 - 1;
    TIM16->DIER &= ~TIM_DIER_UIE;
    TIM16->EGR |= TIM_EGR_UG;
    TIM16->SR &= ~TIM_SR_UIF;
    TIM16->DIER |= TIM_DIER_UIE;
    TIM16->CR1 |= TIM_CR1_CEN;
    
    NVIC_SetPriority(TIM16_IRQn, 1);
    NVIC_EnableIRQ(TIM16_IRQn);
}
//-------------------------
bool IO_SampleIsReady(void)
{
    return _io_data_is_ready;
}
//----------------------
void IO_ReadyReset(void)
{
	_io_data_is_ready = false;
}
//------------------------------------------------
uint32_t* IO_SampleCopy(void/*uint32_t *samples*/)
{
    if(!_io_data_is_ready)
    {
        return 0;
    }
    
    return _io_samples;
}
//--------------------------------------
void IO_SetSequenceCount(uint16_t count)
{
    _io_sequence_count = count;
}
//-----------------------
bool IO_SampleIsEnd(void)
{
    if(_io_sequence_count == 0)
        return true;
    
    return false;
}
//-------------------------
void TIM16_IRQHandler(void)
{
    if((TIM16->SR & TIM_SR_UIF) == TIM_SR_UIF)
    {
        TIM16->SR &= ~TIM_SR_UIF;

        if(_io_data_is_ready == true) // block data if old not reading
            return;

        uint16_t input_mask = 0;
        
        if(_io_dev_addr == 0) // device is MDVV-01
            input_mask = 0x0FFF;
        else if(_io_dev_addr == 1) // device is MDVV-02
            input_mask = 0x03FF;
        
        uint16_t inputs = (GPIOA->IDR&input_mask); // read inputs states
        
        for(uint8_t i = 0; i < _io_count; i++)
        {
            if(inputs & (1 << i))
                _io_samples[i /*+ _io_cur_buffer*/] |= (1 << _io_sample_count);
            else 
                _io_samples[i /*+ _io_cur_buffer*/] &= ~(1 << _io_sample_count);
        }
        
        _io_sample_count++;
        
        if(_io_sample_count >= DINPUT_SAMPLE_PERIOD)
        {
            _io_sample_count = 0;
            _io_data_is_ready = true;
            
            if(_io_sequence_count >= DINPUT_SAMPLE_PERIOD)
                _io_sequence_count -= DINPUT_SAMPLE_PERIOD;
        }
    }
}
