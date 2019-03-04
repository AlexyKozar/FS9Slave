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
    #include "i2c/i2c.h"
    #include "flash/flash.h"
    #include "error.h"
    #include "../RTT/SEGGER_RTT.h"
    #include "../RTT/SEGGER_RTT_Conf.h"
    //----------------------
    #define F_CPU 48000000UL
    //-------------------------
    #define DEVICE_MDVV_01 0x00
    #define DEVICE_MDVV_02 0x01
    #define DEVICE_MIK_01  0x02
    //---------------------------------
    #define DEV_ADDR_MASK (uint8_t)0xC0
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
    //-------------------------------------
    #define DSDIN_TRIGGER_ON_0         0xA1 // срабатывание зафиксировано, установлен уровень «0»
    #define DSDIN_TRIGGER_ON_1         0xF5 // срабатывание зафиксировано, установлен уровень «1»
    #define DSDIN_TRIGGER_OFF          0xB8 // срабатывание не зафиксировано
    #define DSDIN_FUNCTION_NOT_SUPPORT 0xEA // функция не поддерживается
    //--------------------------
    #define KEY_MODE_NONE   0x01
    #define KEY_MODE_SCAN_1 0x02
    #define KEY_MODE_SCAN_2 0x04
    /*!
     *   Описание режимов настройки искробезопасных входов DI01-DI04
     */
    #define SPARK_SECURITY_MODE_NONE (uint8_t)0x00 // искробезопасных входов нет
    #define SPARK_SECURITY_MODE_1    (uint8_t)0x01 // искробезопасные входы настроены на режим 1
    #define SPARK_SECURITY_MODE_2    (uint8_t)0x02 // искробезопасные входы настроены на режим 2
    #define SPARK_SECURITY_MODE_3    (uint8_t)0x03 // искробезопасные входы настроены на режим 3
    //--------------------------
    #define MAX_SIZE_AIN_TEMP 19 // максимальный размер массива для калибровочной таблицы температуры
    //---------------------------
    #define MAX_SIZE_QUEUE_OUT 12 // максимальный размер массива очереди входов в режиме blink (для МИК-01)
    //-------------------------------------------
    #define KEY_EMPTY_MASK ((uint32_t)0x000FFFFF) // маска клавиатуры - неактивное состояние
    //--------------
    // a device data
    #define DEVICE_NUMBER           (uint16_t)0x0001 // номер устройства
    #define DEVICE_LOT              (uint8_t)0x01 // номер партии
    #define DEVICE_FIRMWARE_VARIANT (uint8_t)0x00 // вариант прошивки
    //----------------------
    #define PWROK_INPUT 0x04
    #define PWROK_TIME  2000 // scan time for pwrok
    //----------------------------
    #define INT_STATE_IDLE    0x00 // состояние сигнала INT - свободен
    #define INT_STATE_TIMEOUT 0x01 // состояние сигнала INT - ожидание таймаута, перед отправкой следующего сигнала INT
    #define INT_STATE_RESET   0x02 // состояние сигнала INT - ожидание сброса (если нет чтения в течении 100мс, 
                                   // вход выставляется в единицу и снова прижимается к земле)
    //--------------------------------------
    typedef struct _FS9Buffer_t FS9Buffer_t;
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
        uint8_t        spark_security; // настройка искробезопасных входов (только для МДВВ-01, входы DI01-DI04)
        uint8_t        frequency; // предыдущая частота сигнала
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
    //---------------------
    typedef struct _pwrok_t
    {
        bool     is_ok; // сторожевой флаг (сброс на каждый восходящий фронт)
		bool     state; // состояние аварийной ситуации (true - пропадание питания)
		bool     is_crash; // была зафиксирована авария - отключение питания
		bool     IN_state; // состояние входа во время аварии
		bool     IN_change; // состояние входа во время аварии изменилось
		uint16_t IN_time; // время через которое поменялось состояние входа во время аварии
    } pwrok_t;
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
    //-------------------------
    typedef struct _int_state_t // структура управления выходом INT (событие обновления состояния входов)
    {
        uint8_t mode;  // режим выхода INT
        uint8_t state[3]; // состояние входов (снимок последнего состояния - обновляется только после прочтения предыдущего состояния)
    } int_state_t;
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
    bool    DEV_Request(FS9Buffer_t* source, FS9Buffer_t* dest);
    bool    DEV_Driver(FS9Buffer_t* source, FS9Buffer_t* dest);
    uint8_t DEV_Checksum(FS9Buffer_t* packet, uint8_t size);
    void    DEV_InputScan(void);
    void    DEV_InputSetDefault(void);
    void    DEV_InputFilter(uint8_t index);
    void    DEV_KeyboardScan(void* data);
    bool    DEV_InputChangedChannel(void);
    void    DEV_CrashInit(void);
    void    DEV_PWROKInit(void);
    void    DEV_OutSet(output_t* out);
    void    DEV_OutReset(output_t* out);
    void    DEV_OutToggle(output_t* out);
    bool    DEV_IsOut(output_t* out);
#endif
