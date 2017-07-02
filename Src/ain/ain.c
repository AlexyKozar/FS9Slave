#include "ain.h"
//-------------------------------------
uint16_t AIN_channels[MAX_NUM_CHANNEL];
uint16_t AIN_buffer[MAX_NUM_CHANNEL] = { 0 };
uint8_t  AIN_number_con = 0; // number conversions
bool     AIN_Data_Ready = false; // data is ready
//-----------------
void AIN_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_DMA1EN;
    
    GPIOB->MODER |= GPIO_MODER_MODER8 | GPIO_MODER_MODER9; // input analog
    GPIOB->PUPDR &= (GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);
    
    ADC1->CR &= ~ADC_CR_ADEN; // disable adc for calibration
    ADC1->CR |= ADC_CR_ADCAL; // start calibration adc
    
    while((ADC1->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL); // wait end calibration adc
    
    ADC1->CFGR1 &= ~ADC_CFGR1_RES; // resolution 12 bit
    ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN; // right-aligned
    ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG;
    DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));
    DMA1_Channel1->CMAR = (uint32_t)(AIN_channels);
    DMA1_Channel1->CNDTR = 3;
    DMA1_Channel1->CCR |= DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_TEIE | DMA_CCR_CIRC;
    DMA1_Channel1->CCR |= DMA_CCR_EN; // enable channel
    DMA1_Channel1->CCR |= DMA_CCR_TCIE; // enable interrupt
    ADC1->CR |= ADC_CR_ADEN; // adc enable
    
    while((ADC1->ISR & ADC_ISR_ADRDY) != ADC_ISR_ADRDY); // wait ready adc
    
    ADC1->CHSELR |= ADC_CHSELR_CHSEL8 | ADC_CHSELR_CHSEL9 | ADC_CHSELR_CHSEL16/* | ADC_CHSELR_CHSEL17*/; // selection temperature sensor channel
    ADC1->SMPR   |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; // set sampling time 239.5 ADC clock cycles
    ADC1->CFGR1  |= ADC_CFGR1_CONT; // continuous conversion mode
    ADC->CCR     |= ADC_CCR_TSEN; // enable temperature sensor
    //ADC->CCR     |= ADC_CCR_VREFEN;
    
    NVIC_EnableIRQ(DMA1_Ch1_IRQn);
    NVIC_EnableIRQ(ADC1_IRQn);
}
//---------------------
bool AIN_Is_Ready(void)
{
    return AIN_Data_Ready;
}
//--------------------------
void AIN_Read(uint16_t* buf)
{
    for(uint8_t i = 0; i < MAX_NUM_CHANNEL; ++i)
    {
        buf[i] = AIN_buffer[i]/MAX_NUM_CONVERSION;
    }
}
//-----------------------
void AIN_IRQHandler(void)
{
    if((DMA1->ISR & DMA_ISR_TCIF1) == DMA_ISR_TCIF1)
    {
        for(uint8_t i = 0; i < MAX_NUM_CHANNEL; ++i)
        {
            if(AIN_number_con == 0)
                AIN_buffer[i] = 0;
            
            AIN_buffer[i] += AIN_channels[i];
        }
        
        AIN_number_con++;
        
        if(AIN_number_con == MAX_NUM_CONVERSION)
        {
            AIN_Data_Ready = true;
            AIN_number_con = 0;
        }
        
        DMA1->IFCR &= ~DMA_IFCR_CTCIF1; // clear flag
    }
}
