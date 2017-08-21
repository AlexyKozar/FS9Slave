#ifndef EVENT_H
    #define EVENT_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include "stm32f0xx_hal.h"
    #include "stm32f0xx_hal_gpio.h"
    //-----------------------
    #define EVENT_MAX_SIZE 16
    //--------------------------
    typedef void (*Event)(void);
    //------------
    struct event_t
    { 
        Event    event; // функция обработки события
        bool     autorepeat; // автоповтор события
        uint16_t time; // время задержки
        uint8_t  id; // id события
    };
    //--------------------
    void EVENT_Init(void);
    bool EVENT_Create(uint16_t time, bool autorepeat, Event f);
    void EVENT_Execute(void);
#endif
