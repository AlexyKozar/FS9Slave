#ifndef EVENT_H
    #define EVENT_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include "stm32f0xx_hal.h"
    #include "stm32f0xx_hal_gpio.h"
    //-----------------------
    #define EVENT_MAX_SIZE 16
    //----------------------------
    typedef void (*event_t)(void);
    //-----------------------
    void    EVENT_Init(void);
    bool    EVENT_Create(uint16_t time, event_t ev);
    event_t EVENT_Execute(void);
#endif
