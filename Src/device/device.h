#ifndef DEVICE_H
    #define DEVICE_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include "stm32f0xx_hal.h"
    #include "stm32f0xx_hal_gpio.h"
    #include "math.h"
    #include "fs9slave/fs9slave.h"
    #include "registers/registers.h"
    #include "ain/ain.h"
    //----------------------
    #define F_CPU 48000000UL
    //---------------------------
    #define MAX_SIZE_DS_INPUT  12
    #define MAX_SIZE_DS_OUTPUT 8
    //------------------------
    #define DEV_IO_INPUT  0x00
    #define DEV_IO_OUTPUT 0x01
    //-------------------------
    #define IN_DIR_INVERSE 0x00
    #define IN_DIR_DIRECT  0x01
    //---------------------
    #define IN_MODE_AC 0x00
    #define IN_MODE_DC 0x01
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
    #define MAX_SIZE_AIN_TEMP 19 // максимальный размер массива для калибровочной таблицы температуры
    //-------------------
    struct INPUT_Set_Type
    {
        uint8_t Nac;
        uint8_t Dac;
        float   SGac;
        uint8_t Sdur;   // длительность сигнала в мс
        float   NSac;
        uint8_t Ndc;
        uint8_t Ddc;
        uint8_t P0dc;
        uint8_t P1dc;
    };
    //----------------------
    struct INPUT_Filter_Type
    {
        uint8_t c_clock;    // счетчик тактов
        uint8_t c_period;   // счетчик периодов для фильтрации
        uint8_t c_high_lev; // счетчик импульсов лог. "1"
        uint8_t c_low_lev;  // счетчик импульсов лог. "0"
        uint8_t c_state;    // счетчик состояний входа
        bool    с_error;    // счетчик ошибок канала
        bool    is_capture; // флаг захвата входа
    };
    //---------------
    struct INPUT_Type
    {
        uint16_t                 pin;    // номер входа
        bool                     state;  // состояние входа (вкл или выкл)
        bool                     error;  // ошибка канала
        struct INPUT_Filter_Type filter; // параметры входа
        uint8_t                  fault;  // погрешность в %
        uint8_t                  mode;   // режим входа (AC/DC)
        uint8_t                  dir;    // направление (прямой/инверсный)
    };
    //--------------------
    struct PORT_Input_Type
    {
        GPIO_TypeDef*         gpio;
        struct INPUT_Type     in_arr[MAX_SIZE_DS_INPUT];
        struct INPUT_Set_Type in_set;
        uint8_t               size;
    };
    //---------------------
    struct PORT_Output_Type
    {
        GPIO_TypeDef* gpio;
        uint16_t      out_arr[MAX_SIZE_DS_OUTPUT];
        uint8_t       size;
    };
    //---------------
    struct PWROK_Type
    {
        bool     is_pwrok; // присутствие/отсутствие сигнала PWR_OK
        bool     is_dsdin; // режим "отключение питания" при отсутствии сигнала PWR_OK
        uint16_t dsdin_time; // время от включения режима "отключения питания" до изменения уровня на входе DSDIN
        bool     dsdin_level; // уровень сигнала на входе DSDIN
        bool     dsdin_lev_changed; // уровень сигнала изменился
    };
    //-----------
    union float_t
    {
        uint8_t byte[4];
        float   number;
    };
    //---------------------------------------------------------
    void    DEV_Create(GPIO_TypeDef* gpio, uint16_t addr_pins);
    void    DEV_Init(struct PORT_Input_Type* inputs, struct PORT_Output_Type* outputs);
    uint8_t DEV_Address(void);
    bool    DEV_Request(struct FS9Packet_t* source, struct FS9Packet_t* dest);
    bool    DEV_Driver(uint8_t cmd, struct FS9Packet_t* data, struct FS9Packet_t* packet);
    uint8_t DEV_Checksum(struct FS9Packet_t* packet, uint8_t size);
    void    DEV_Input_Scan(void);
    void    DEV_Input_Set_Default(void);
    void    DEV_Input_Filter(uint8_t index);
    bool    DEV_Input_Changed_Channel(void);
#endif
