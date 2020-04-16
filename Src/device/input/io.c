#include "io.h"
//-------------------------------------------------
uint32_t _io_samples[DINPUT_MAX_SIZE /*<< 1*/] = { 0 }; // samples buffer (double)
volatile uint8_t  _io_count = 0;                             // io count
volatile uint8_t  _io_sample_count = 0;                      // current sample
volatile bool     _io_data_is_ready = false;                 // flag data is ready
volatile uint8_t  _io_cur_buffer = 0;                        // offset current begin buffer (first or seconst buffer)
//----------------------------
void IO_TIM_Init(uint8_t addr)
{
    if(addr == 0x02) // device address is MIK-01
        return;
    
    if(addr == 0x00) // device address is MDVV-01
        _io_count = 12;
    else if(addr == 0x01) // device address is MDVV-02
        _io_count = 10;
    
    _io_cur_buffer = 0; // first buffer
    
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
//--------------------------------------
void IO_ReadyReset(void)
{
	_io_data_is_ready = false;
}



uint32_t* IO_SampleCopy(void/*uint32_t *samples*/)
{
    if(!_io_data_is_ready)
    {
        return 0;
    }
    
    //uint8_t offset = (_io_cur_buffer == 0)?DINPUT_MAX_SIZE:0;
    
   // memcpy(samples, _io_samples /*+ offset*/, _io_count);
	//	memset(_io_samples /*+ offset*/, 0, 20);
    //_io_data_is_ready = false;
		//samples = _io_samples;
    
   // return _io_count;
		return _io_samples;
}
//-------------------------
void TIM16_IRQHandler(void)
{
    if((TIM16->SR & TIM_SR_UIF) == TIM_SR_UIF)
    {
        TIM16->SR &= ~TIM_SR_UIF;
        if(_io_data_is_ready == true) return;
        uint16_t inputs = GPIOA->IDR;
        
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
            /*if(_io_cur_buffer == 0) // if first buffer
                _io_cur_buffer = DINPUT_MAX_SIZE;
            else if(_io_cur_buffer == DINPUT_MAX_SIZE)
                _io_cur_buffer = 0;
            */
            _io_sample_count = 0;
            _io_data_is_ready = true;
        }
    }
}
