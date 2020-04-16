#ifndef _IO_H_
    #define _IO_H_
    //--------------------
    #include "stm32f0xx.h"
    //-----------------------------
    #define DINPUT_MAX_SIZE      20 // max size buffer samples
    #define DINPUT_SAMPLE_PERIOD 20 // samples count on one period
    //--------------------------------
    void    IO_TIM_Init(uint8_t addr);
    bool    IO_SampleIsReady(void);
    //uint8_t IO_SampleCopy(uint32_t *samples);
		uint32_t* IO_SampleCopy(void);
		void IO_ReadyReset(void);
#endif
