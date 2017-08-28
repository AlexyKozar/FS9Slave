#include "event.h"
//---------------------------------
const uint16_t TIMER_IDLE = 0xFFFF;
//------------
struct timer_t
{
    uint16_t        timer;
    struct event_t  event;
};
//-----------------------------------
struct event_t event[EVENT_MAX_SIZE]; // очередь событий
struct timer_t timer[EVENT_MAX_SIZE]; // очередь таймеров
//----------------------
uint8_t event_count = 0;
uint8_t event_head  = 0;
uint8_t event_tail  = 0;
//------------------------------------
bool EVENT_Insert(struct event_t evt);
//-------------------
void EVENT_Init(void)
{
    for(uint8_t i = 0; i < EVENT_MAX_SIZE; ++i)
    {
        timer[i].timer            = TIMER_IDLE;
        timer[i].event.autorepeat = false;
        timer[i].event.event      = NULL;
        timer[i].event.id         = i; // id события создается при инициализации и не меняется
    }
    
    uint32_t systick = 48000 - 1;
    
    SysTick->LOAD = systick; // Загрузка значения
    SysTick->VAL  = systick; // Обнуляем таймеры и флаги записью 
    SysTick->CTRL =	SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}
//----------------------------------------------------------------------------------------------------------------
uint8_t EVENT_Create(uint16_t time, bool autorepeat, Event function, GPIO_TypeDef* gpio, uint16_t pin, uint8_t id)
{
    struct timer_t* tim = NULL;
    
    if(id != 0xFF) // задан определенный таймер
    {
        tim = &timer[id]; // получаем заданный таймер
    }
    else if(id == 0xFF) // таймер не задан
    {
        for(uint8_t i = 0; i < EVENT_MAX_SIZE; ++i)
        {
            if(timer[i].timer == TIMER_IDLE)
            {
                tim = &timer[i];
            }
        }
    }

    if(tim != NULL)
    {
        tim->timer            = time;
        tim->event.time       = time;
        tim->event.autorepeat = autorepeat;
        tim->event.event      = function;
        tim->event.gpio       = gpio;
        tim->event.pin        = pin;
        
        return tim->event.id;
    }
    
    return 0xFF;
}
//----------------------
void EVENT_Execute(void)
{
    if(event_count == 0)
        return;
    
    struct event_t evt;
    
    evt = event[event_head++];
    
    if(event_head == EVENT_MAX_SIZE)
        event_head = 0;
    
    event_count--;
    
    if(evt.event != NULL)
    {
        evt.event(evt.gpio, evt.pin); // вызов функции обработки события
    }
    
    if(evt.autorepeat == true) // включен автоповтор события
    {
        EVENT_Create(evt.time, evt.autorepeat, evt.event, evt.gpio, evt.pin, evt.id);
    }
}
//-----------------------------------
bool EVENT_Insert(struct event_t evt)
{
    if(event_count == EVENT_MAX_SIZE)
        return false;
    
    event[event_tail++] = evt;
    
    if(event_tail == EVENT_MAX_SIZE)
        event_tail = 0;
    
    event_count++;
    
    return true;
}
//-------------------------
void EVENT_Kill(uint8_t id)
{
    timer[id].timer = TIMER_IDLE;
}
//------------------------
void SysTick_Handler(void)
{
    for(uint8_t i = 0; i < EVENT_MAX_SIZE; ++i)
    {
        if(timer[i].timer != TIMER_IDLE)
        {
            timer[i].timer--;
            
            if(timer[i].timer == 0)
            {
                EVENT_Insert(timer[i].event);
                
                timer[i].timer = TIMER_IDLE;
            }
        }
    }
}
