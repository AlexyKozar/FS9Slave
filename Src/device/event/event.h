#ifndef EVENT_H
    #define EVENT_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include "stm32f0xx_hal.h"
    #include "stm32f0xx_hal_gpio.h"
    //-----------------------
    #define EVENT_MAX_SIZE 16
    //------------------------------------------------------
    typedef void (*Event)(GPIO_TypeDef* gpio, uint16_t pin);
    //---------------------
    typedef struct _event_t
    { 
        Event         event; // функция обработки события
        bool          autorepeat; // автоповтор события
        uint16_t      time; // время задержки
        uint8_t       id; // id события
        GPIO_TypeDef* gpio;
        uint16_t      pin;
    } event_t;
    //-----------------------
    void    EVENT_Init(void);
    uint8_t EVENT_Create(uint16_t time, bool autorepeat, Event function, GPIO_TypeDef* gpio, uint16_t pin, uint8_t id);
    void    EVENT_Execute(void);
    void    EVENT_Kill(uint8_t id);
#endif
