#ifndef DEVICE_H
    #define DEVICE_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include "stm32f0xx_hal.h"
    #include "stm32f0xx_hal_gpio.h"
    #include "fs9slave/fs9slave.h"
    #include "math.h"
    #include "ain/ain.h"
    #include "event/event.h"
    #include "ds18b20/ds18b20.h"
    //----------------------
    #define F_CPU 48000000UL
    //---------------------------
    #define MAX_SIZE_DS_INPUT  12
    #define MAX_SIZE_DS_OUTPUT 12
    //------------------------
    #define DEV_IO_INPUT  0x00
    #define DEV_IO_OUTPUT 0x01
    //-------------------------
    #define IN_DIR_INVERSE 0x00
    #define IN_DIR_DIRECT  0x01
    //---------------------
    #define IN_MODE_AC 0x00
    #define IN_MODE_DC 0x01
    //------------------------------------------
    #define OUTPUT_STATE_OFF      (uint8_t) 0x00 // выход выключен
    #define OUTPUT_STATE_ON       (uint8_t) 0x01 // выход включен (постоянно)
    #define OUTPUT_STATE_FREQ_2HZ (uint8_t) 0x02 // выход включен (мигает с частотой 2Гц)
    #define OUTPUT_STATE_RESERVE  (uint8_t) 0x03 // резервное состояние (мигает с частотой 2Гц)
    //--------------------------
    #define GPIO_INT       GPIOB
    #define GPIO_INT_PIN   GPIO_PIN_5
    #define GPIO_INT_SET   GPIO_BSRR_BS_5
    #define GPIO_INT_RESET GPIO_BSRR_BR_5
    //--------------------------
    #define GPIO_PWROK     GPIOA
    #define GPIO_PWROK_PIN GPIO_PIN_12
    //-------------------------------------
    #define DSDIN_TRIGGER_ON_0         0xA1 // срабатывание зафиксировано, установлен уровень «0»
    #define DSDIN_TRIGGER_ON_1         0xF5 // срабатывание зафиксировано, установлен уровень «1»
    #define DSDIN_TRIGGER_OFF          0xB8 // срабатывание не зафиксировано
    #define DSDIN_FUNCTION_NOT_SUPPORT 0xEA // функция не поддерживается
    //--------------------------
    #define KEY_MODE_NONE   0x01
    #define KEY_MODE_SCAN_1 0x02
    #define KEY_MODE_SCAN_2 0x04
    //--------------------------
    #define MAX_SIZE_AIN_TEMP 19 // максимальный размер массива для калибровочной таблицы температуры
    //---------------------------
    #define MAX_SIZE_QUEUE_OUT 12 // максимальный размер массива очереди входов в режиме blink (для МИК-01)
    //-------------------------------------------
    #define KEY_EMPTY_MASK ((uint32_t)0x000FFFFF) // маска клавиатуры - неактивное состояние
    //--------------------------------------
    typedef struct _FS9Packet_t FS9Packet_t;
    //-------------------------
    typedef struct _input_set_t
    {
        uint8_t Nperiod; // количество периодов накопления информации
        uint8_t Ndiscret; // количество выборок на период
        uint8_t SGac; // длина валидного импульса
        uint8_t P0dc; // процент нулей фильтра для постоянного сигнала
        uint8_t P1dc; // процент единиц фильтра для постоянного сигнала
    } input_set_t;
    //----------------------------
    typedef struct _input_filter_t
    {
        uint8_t c_clock; // такты после захвата входа
        uint8_t c_error; // счетчик ошибок
        bool    is_capture; // флаг захвата входа
        uint8_t c_lev_0; // количество импульсов лог. "0"
        uint8_t c_lev_1; // количество импульсов лог. "1"
        uint8_t c_period; // количество периодов
        uint8_t c_state; // количество валидных состояний
    } input_filter_t;
    //------------------
    typedef struct _io_t
    {
        GPIO_TypeDef* gpio; // порт канала
        uint32_t      io; // пин канала
        uint8_t       num; // номер канала
    } io_t;
    //---------------------
    typedef struct _input_t
    {
        io_t           pin;    // вход (номер входа и порт)
        bool           state;  // состояние входа (вкл или выкл)
        bool           error;  // ошибка канала
        input_filter_t filter; // параметры фильтрации входа
        uint8_t        fault;  // погрешность длительности периода в %
        uint8_t        mode;   // режим входа (AC/DC)
        uint8_t        dir;    // направление (прямой/инверсный)
        uint16_t       duration; // длительность периода
    } input_t;
    //----------------------
    typedef struct _output_t
    {
        io_t    pin;
        uint8_t state; // состояние канала
        uint8_t param; // произвольный параметр
        bool    level; // активный уровень (прямой - лог "1" и инверсный - лог "0")
    } output_t;
    //-----------------------------
    typedef struct _PORT_Input_Type
    {
        input_t     list[MAX_SIZE_DS_INPUT];
        input_set_t set;
        uint8_t     size;
    } PORT_Input_Type;
    //------------------------------
    typedef struct _PORT_Output_Type
    {
        output_t list[MAX_SIZE_DS_OUTPUT];
        uint8_t  size;
    } PORT_Output_Type;
    //------------------------
    typedef struct _PWROK_Type
    {
        bool     is_pwrok; // присутствие/отсутствие сигнала PWR_OK
        bool     is_dsdin; // режим "отключение питания" при отсутствии сигнала PWR_OK
        uint16_t dsdin_time; // время от включения режима "отключения питания" до изменения уровня на входе DSDIN
        bool     dsdin_level; // уровень сигнала на входе DSDIN
        bool     dsdin_lev_changed; // уровень сигнала изменился
    } PWROK_Type;
    //---------------------
    typedef struct _error_t // структура ошибкок приема данных по протоколу
    {
        uint16_t address; // ошибка адресации
        uint16_t command; // ошибка команды
        uint16_t checksum; // ошибка контрольной суммы
    } error_t;
    //-------------------
    typedef struct _key_t
    {
        uint32_t last_state;
        uint32_t cur_state;
        uint32_t temp;
        uint8_t  mode;
        bool     is_bounce;
    } key_t;
    //---------------------------
    typedef struct _Blink_queue_t
    {
        uint8_t queue[MAX_SIZE_QUEUE_OUT];
        bool    state;
        uint8_t count;
    } Blink_queue_t;
    //-----------
    union float_t
    {
        uint8_t byte[4];
        float   number;
    };
    //---------------------------------------------------------
    void    DEV_Create(GPIO_TypeDef* gpio, uint16_t addr_pins);
    void    DEV_Init(PORT_Input_Type* inputs, PORT_Output_Type* outputs);
    uint8_t DEV_Address(void);
    bool    DEV_Request(FS9Packet_t* source, FS9Packet_t* dest);
    bool    DEV_Driver(uint8_t cmd, FS9Packet_t* data, FS9Packet_t* packet);
    uint8_t DEV_Checksum(FS9Packet_t* packet, uint8_t size);
    void    DEV_Input_Scan(void);
    void    DEV_Input_Set_Default(void);
    void    DEV_Input_Filter(uint8_t index);
    void    DEV_Keyboard_Scan(void* data);
    bool    DEV_Input_Changed_Channel(void);
    void    DEV_Crash_Init(void);
    void    DEV_PWROK_Init(void);
    void    DEV_Out_Set(output_t* out);
    void    DEV_Out_Reset(output_t* out);
    void    DEV_Out_Toggle(output_t* out);
    bool    DEV_Is_Out(output_t* out);
#endif
