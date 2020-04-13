#include "device.h"
//------------------------------------------
void    IO_Clock_Enable(GPIO_TypeDef* gpio);
void    IO_Init(io_t io, uint8_t io_dir);
void    CHANNEL_Out_Set(uint8_t index);
void    CHANNEL_Out_Reset(uint8_t index);
void    TIM_Scan_Init(void);
void    TIM_Scan_Update(void);
void    TIM_INT_Init(void);
void    TIM_INT_Start(void);
void    TIM_Backlight_Update(void);
float   Get_Temp(uint16_t val, uint8_t in_num);
float   UAIN_to_TResistance(uint16_t val, uint8_t in_num); // преобразование напряжения в сопротивление температуры
void    blink2Hz(void* output); // мигание с частотой 2Гц (для МИК-01)
void    crash(void* output); // для обработки аварийной ситуации (нет запросов от ЦП 5 сек)
void    int_timeout(void* param); // для обработки таймаута отправки данных по изменению состояний входов
void    int_watchdog(void *param); // для обработки зависания сигнала INT
void    queue_init(void);
void    insert_task(uint8_t id); // вставка задачи в очередь для мигания (МИК-01)
void    kill_task(uint8_t id); // убить задачу в очереди для мигания (МИК-01)
void    getDateBCD(uint8_t* date); // получение текущей даты
uint8_t convertByteToBCD(int value); // конвертирование числа в код BCD
void    convertInputState(uint8_t* data); // конвертирование состояний входов (data - массив из трех байт)
bool    isEqualIputState(uint8_t* data); // проверка на эквивалентность состояний входов
void    inputStateUpdate(void);
void    inputSettings(uint8_t number, uint8_t mode, uint8_t duration, uint8_t fault);
void    Backlight_Init(void); // инициализация ноги подсветки дисплея
void    TIM_Init_Crash(void); // инициализация таймера обрыва связи
//----------------------
PORT_Input_Type*  io_in;
PORT_Output_Type* io_out;
//---------------------
uint8_t devAddr = 0xFF;
uint8_t devID   = 0xFF;
//--------------------------
uint8_t int_reset_id = 0xFF; // id задачи СБРОСА линии INT по таймауту чтения
//-------------------------
bool InputStateChanged = false;
//-----------------------------
bool        is_crash   = false; // авария - включение выхода (происходит в случае отсутствия запросов от ЦП 5 сек)
output_t*   out_crash  = NULL; // выход аварийной сигнализации
input_t*    io_inOff   = 0;
input_t*    io_inOn    = 0;
input_t*    io_inPhase = NULL;
//--------------------
int_state_t int_state; // буфер для храненя состояния входов (один снимок)
//--------------------------
uint8_t deviceSN[8] = { 0 }; // серийный номер устройства
//--------------------------------------------------------------------------------
key_t keys = { 0x00000000, KEY_EMPTY_MASK, KEY_EMPTY_MASK, KEY_MODE_NONE, false };
//-----------------------------
error_t error = { 0, 0, 0, 0 };
//----------------------------
Blink_queue_t out_queue_blink;
//-----------------------------------------------------------------------
volatile pwrok_t _pwr_ok = { false, false, false, false, false, 0x0000 };
//---------------------------------------
uint16_t AIN_TEMP[MAX_SIZE_AIN_TEMP][3] = 
{
    { 0, 13, 18 }, // сопротивление термодатчика, напряжение AIN1, напряжение AIN2 (напряжения умножены на 10000)
    { 1, 170, 170 },
    { 2, 330, 340 },
    { 10, 1650, 1650 },
    { 20, 3280, 3270 },
    { 37, 6080, 6070 },
    { 48, 8010, 7980 },
    { 66, 10800, 10780 },
    { 80, 13230, 13190 },
    { 92, 15080, 15040 },
    { 100, 16470, 16420 },
    { 115, 18850, 18790 },
    { 127, 20800, 20700 },
    { 166, 27200, 27100 },
    { 178, 29200, 29100 },
    { 187, 30800, 30700 },
    { 189, 31000, 30900 },
    { 195, 32200, 32100 },
    { 200, 32900, 32900 }
};
//--------------------
uint32_t key_read = 0;
uint32_t key_receive = 0;
//---------------------------
uint32_t int_reset_count = 0; // переменная подсчета сбросов сигнала INT (если было 50, то устройство занято и сброс отключается)
uint32_t int_watchdog_count = 0; // переменная подсчета активности сигнала INT (активным является низкий уровень)
//------------------------------------------------
uint16_t backlight_mode = MODE_BACKLIGHT_PAUSE_0N; // режим инициализации подсветки дисплея
//-----------------------------------------------------
void DEV_Create(GPIO_TypeDef* gpio, uint16_t addr_pins)
{
    io_t pin = { gpio, addr_pins };
    
    IO_Init(pin, DEV_IO_INPUT);
    
    devAddr = (uint8_t)((gpio->IDR & addr_pins) >> 14); // set the address device
    
    if(devAddr == DEVICE_MDVV_01) // a device is MDVV-01
    {
        devID = 0x48;
    }
    else if(devAddr == DEVICE_MDVV_02) // a device is MDVV-02
    {
        devID = 0x49;
    }
    else if(devAddr == DEVICE_MIK_01) // a device is MIK-01
    {
        devID = 0x50;
    }
    
    //devAddr = 0x02; // для теста МИК
}
//---------------------------------------------------------------
void DEV_Init(PORT_Input_Type* inputs, PORT_Output_Type* outputs)
{
    io_in  = inputs;
    io_out = outputs;
    
    for(uint8_t i = 0; i < io_in->size; ++i)
    {
        if((devAddr == DEVICE_MIK_01 && i == 10) || 
           (devAddr == DEVICE_MIK_01 && i == 11))
        {
            // устройство МИК-01 и это 10 или 11 канал
            IO_Init(io_in->list[i].pin, DEV_IO_OUTPUT); // настраиваем как выход (сканирование)
            
            // устанавливаем на выходе лог "1"
            io_in->list[i].pin.gpio->ODR |= io_in->list[i].pin.io;
        }
        else
        {
            IO_Init(io_in->list[i].pin, DEV_IO_INPUT); // настраиваем как вход
            io_in->list[i].frequency = 0; // обнуляем переменную, которая сохраняет предыдущую частоту
        }
        
        io_in->list[i].pin.num = i;
    }
    
    for(uint8_t i = 0; i < io_out->size; ++i)
    {
        IO_Init(io_out->list[i].pin, DEV_IO_OUTPUT);
        io_out->list[i].pin.num = i;
        io_out->list[i].state   = OUTPUT_STATE_OFF;

				HAL_GPIO_WritePin(io_out->list[i].pin.gpio, io_out->list[i].pin.io, GPIO_PIN_RESET); // выключить выход - состояние по умолчанию
        //DEV_OutReset(&io_out->list[i]); // выключить выход - состояние по умолчанию
    }
    
    io_t pin_int = { GPIO_INT, GPIO_INT_PIN };
    IO_Init(pin_int, DEV_IO_OUTPUT); // вывод INT как выход
    
    GPIO_INT->BSRR |= GPIO_INT_SET; // включить выход INT (default state)
    
    DEV_InputSetDefault();
    
    if(devAddr != DEVICE_MIK_01) // только для МДВВ
    {
        if(devAddr == DEVICE_MDVV_01) // только для МДВВ-01
        {
            io_inPhase = &io_in->list[0]; // искробезопасный вход DI_1 для определения фазы
            io_inOff   = &io_in->list[1]; // искробезопасный вход DI_2 - кнопка СТОП
            io_inOn    = &io_in->list[2]; // искробезопасный вход DI_3 - кнопка СТАРТ
        }
    }
    else
    {
        Backlight_Init(); // инициализация ноги подсветки дисплея
			
        EVENT_Create(5, false, DEV_KeyboardScan, NULL, 0xFF); // опрос клавиатуры
        EVENT_Create(250, false, int_watchdog, NULL, 0xFF); // создание задачи сторожевой собаки для линии линии INT (250мс)
        
        queue_init(); // инициализация очереди входов для работы в режиме мигания
        
        EVENT_Create(1000, true, blink2Hz, NULL, 0xFF); // создание задачи мигания
    }
    
    // TIM_INT_Init();
    
    // Формирование серийного номера устройства
    deviceSN[0] = devID; // код изделия
    deviceSN[1] = convertByteToBCD(DEVICE_NUMBER >> 8); // старший байт номера устройства
    deviceSN[2] = convertByteToBCD(DEVICE_NUMBER&0x00FF); // младший байт номера устройства
    deviceSN[3] = convertByteToBCD(DEVICE_LOT); // номер в партии
    deviceSN[4] = convertByteToBCD(DEVICE_FIRMWARE_VARIANT); // вариант прошивки
    
    uint8_t current_date[3] = { 0 };
    getDateBCD(current_date);
    
    deviceSN[5] = current_date[0]; // год прошивки
    deviceSN[6] = current_date[1]; // месяц прошивки
    deviceSN[7] = current_date[2]; // день прошивки

    if(FLASH_Unlock())
    {
        uint32_t serial_key = FLASH_Read(FLASH_SERIAL_ADDRESS);
        uint32_t data = 0;    
        
        if(serial_key != FLASH_CELL_EMPTY) // serial key is not empty
        {
            // read serial number
            data = FLASH_Read(FLASH_SERIAL_ADDRESS + 4);
            
            deviceSN[1] = (data >> 24)&0x000000FF;
            deviceSN[2] = (data >> 16)&0x000000FF;
            deviceSN[3] = (data >> 8)&0x000000FF;
            deviceSN[4] = data&0x000000FF;
        }
        else // write default serial key to flash
        {
            if(FLASH_Erase(FLASH_SERIAL_ADDRESS))
            {
                FLASH_Write(FLASH_SERIAL_ADDRESS, SERIAL_NUMBER_KEY);
                
                data = ((deviceSN[1] << 24) | (deviceSN[2] << 16) | (deviceSN[3] << 8) | deviceSN[4]);
                FLASH_Write(FLASH_SERIAL_ADDRESS + 4, data);
            }
        }
        			
        FLASH_Lock();
    }
    
    // начальная инициализация буфера хранения состояний входов
    int_state.mode = INT_STATE_IDLE;
    int_state.state[0] = 0x00;
    int_state.state[1] = 0x00;
    int_state.state[2] = 0x00;
		
    TIM_Scan_Init(); // запуск сканирования входов для плат МДВВ-01 и МДВВ-02 или инициализация подсветки для МИК-01
}
//-----------------------
void Backlight_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    
    /*Configure GPIO pins*/
    GPIO_InitStruct.Pin  = GPIO_BACKLIGHT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIO_BACKLIGHT, &GPIO_InitStruct);
	
    GPIO_BACKLIGHT->BSRR |= GPIO_BACKLIGHT_LOW;
}
/*!
 * date - буфер для хранения даты
 */
void getDateBCD(uint8_t* date)
{
    const char* months[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

    uint8_t tdate[] = __DATE__;
    
    if(strlen((const char*)tdate) == 11)
    {
        tdate[3] = 0;
        tdate[6] = 0;

        int year  = atoi((const char*)tdate + 9);
        int month = -1;
        int day   = atoi((const char*)tdate + 4);

        for(int i = 0; i < 12; i++)
        {
            if(strcmp((const char*)tdate, months[i]) == 0)
            {
                month = i + 1;
                break;
            }
        }

        if(month != -1)
        {
            date[0] = convertByteToBCD(year); // год в формате BCD
            date[1] = convertByteToBCD(month); // месяц в формате BCD
            date[2] = convertByteToBCD(day); // день в формате BCD
        }
    }
}
/*!
 * value - значение которое переводится в BCD
 */
uint8_t convertByteToBCD(int value)
{
    return (((value/10) << 4) | (value%10));
}
//--------------------------------------
void IO_Clock_Enable(GPIO_TypeDef* gpio)
{
    if(gpio == GPIOA)
        RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    else if(gpio == GPIOB)
        RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    else if(gpio == GPIOC)
        RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    else if(gpio == GPIOD)
        RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    else if(gpio == GPIOF)
        RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
}
//-----------------------------------
void convertInputState(uint8_t* data)
{
    uint8_t  bit_count  = 0x00; // счетчик битов
    uint8_t  byte_index = 0x00; // индекс текущего байта
    
    // обнуление массива состояний входов
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
        
    if(devAddr == DEVICE_MIK_01) // для модуля МИК-01 отдельный алгоритм конвертирования (матричная клавиатура)
    {
        data[0] = keys.last_state&0x000000FF;
        data[1] = (keys.last_state >> 8)&0x000000FF;
        data[2] = (keys.last_state >> 16)&0x0000000F;
    }
    else
    {
        for(uint8_t i = 0; i < io_in->size; ++i)
        {
            if(bit_count == 8)
            {
                bit_count = 0;
                byte_index++;
            }
            
            uint8_t  channel_state = 0x00;
            input_t* channel       = &io_in->list[i];
            
            if(channel->state == true && channel->error == false)
            {
                // состояние канала входа активно и ошибок в канале нет
                channel_state = 0x01; // сигнал на входе присутствует
            }
            else if(channel->error == true)
            {
                // зафиксирована ошибка канала входа
                channel_state = 0x02;
                
                if(channel->state == true) // если вход установлен, то добавляем активное состояние
                    channel_state |= 0x01;
                else if(channel_state == false)
                    channel_state |= 0x00;
                
                // сбрасываем ошибку канала входа при чтении
                if(channel_state & 0x02)
                {
                    channel->error = false;
                    channel->state = 0x00;
                }
            }
            
            data[byte_index] |= channel_state << bit_count;
            bit_count += 2;
        }
    }
}
//----------------------------------
bool isEqualIputState(uint8_t* data)
{
    if(data[0] == int_state.state[0] && 
       data[1] == int_state.state[1] && 
       data[2] == int_state.state[2])
    {
        return true;
    }
    
    return false;
}
//-------------------------
void inputStateUpdate(void)
{
    if(int_state.mode != INT_STATE_IDLE || (GPIO_INT->BSRR & GPIO_INT_RESET)) // вывод INT в таймауте или INT прижат
    {
        return; // выходим и не сохраняем состояние
    }

    uint8_t state[3];
    convertInputState(state); // создаем снимок текущего состояния входов
    if(!isEqualIputState(state)) // если текущее состоние не равно последнему снимку
    {
        int_state.state[0] = state[0]; // то меняем состояние снимка на текущее
        int_state.state[1] = state[1];
        int_state.state[2] = state[2];
			
        int_reset_count = 0; // сброс переменной хранящей количество сброса сигнала INT
        int_watchdog_count = 0;

        InputStateChanged = false; // очистка флага изменения состояния входов
        GPIO_INT->BSRR |= GPIO_INT_RESET; // прижимаем линию INT (сигнал для МЦП - состояния входов изменились)
        int_state.mode = INT_STATE_RESET; // выставляем состояние СБРОСА линии INT, если чтения нет в течении 100мс
        int_reset_id = EVENT_Create(100, false, int_timeout, NULL, 0xFF); // создание задачи СБРОСА на линии INT (100мс)
    }
}
//------------------------------------
void IO_Init(io_t pin, uint8_t io_dir)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    IO_Clock_Enable(pin.gpio);
    
    /*Configure GPIO pins*/
    GPIO_InitStruct.Pin  = pin.io;
    GPIO_InitStruct.Mode = (io_dir == 0x01)?GPIO_MODE_OUTPUT_PP:GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(pin.gpio, &GPIO_InitStruct);
}
//----------------------------
void DEV_OutSet(output_t* out)
{
    uint32_t io = ((uint32_t)out->pin.io);
    
    if(out->level == false) // включение выхода по лог "0"
    {
        io <<= 16; // смещаем влево на 2 байта для получения RESET (GPIO_BSRR_BRx)
    }
    
    out->pin.gpio->BSRR |= io;
}
//------------------------------
void DEV_OutReset(output_t* out)
{
    uint32_t io = ((uint32_t)out->pin.io);
    
    if(out->level == true) // включение выхода по лог "1"
    {
        io <<= 16; // смещаем влево на 2 байта для получения RESET (GPIO_BSRR_BRx)
    }
    
    out->pin.gpio->BSRR |= io;
}
//-------------------------------
void DEV_OutToggle(output_t* out)
{
    if(DEV_IsOut(out))
    {
        DEV_OutReset(out);
    }
    else
    {
        DEV_OutSet(out);
    }
}
//---------------------------
bool DEV_IsOut(output_t* out)
{
    bool state = (out->pin.gpio->ODR & out->pin.io);
    
    if(out->level == true) // выход управляется лог "1" (прямой)
    {
        return state;
    }
    
    return !state; // иначе управление инверсное, т.е. лог "0"
}
//---------------------------------
void CHANNEL_Out_Set(uint8_t index)
{
    if(index < io_out->size)
    {
        output_t* out = &io_out->list[index];

        DEV_OutSet(out);
    }
}
//-----------------------------------
void CHANNEL_Out_Reset(uint8_t index)
{
    if(index < io_out->size)
    {
        output_t* out = &io_out->list[index];
        DEV_OutReset(out);
    }
}
//-----------------------
void TIM_Init_Crash(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    
    TIM1->PSC = F_CPU/1000UL - 1;
    TIM1->ARR = 5000 - 1; // перезагрузка каждые 5сек
    TIM1->DIER &= ~TIM_DIER_UIE;
    TIM1->EGR |= TIM_EGR_UG;
    TIM1->SR &= ~TIM_SR_UIF;
    TIM1->DIER |= TIM_DIER_UIE;
    TIM1->CR1 |= TIM_CR1_OPM | TIM_CR1_CEN;
    
    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 1);
    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
}
//----------------------
void TIM_Scan_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
    
    TIM16->PSC = F_CPU/1000000UL - 1;
    TIM16->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM16_IRQn);
    
    if(devAddr != DEVICE_MIK_01) // если это не МИК-01, то инициализируем сканирование входов
    {
        TIM_Scan_Update();
        
        TIM16->CR1  |= TIM_CR1_ARPE;
        TIM16->CR1  |= TIM_CR1_CEN; 
    }
    else
    {
        TIM16->PSC = 48000 - 1;
        TIM16->ARR = 200 - 1; // пауза 1 секунда перед настройкой дисплея
        TIM_Backlight_Update();

        TIM16->CR1 |= TIM_CR1_OPM | TIM_CR1_CEN;
    }
}
//------------------------
void TIM_Scan_Update(void)
{
    TIM16->ARR   = 10000/io_in->set.Ndiscret - 1;
    TIM16->DIER &= ~TIM_DIER_UIE;
    TIM16->EGR  |= TIM_EGR_UG;
    TIM16->SR &= ~TIM_SR_UIF;
    TIM16->DIER |= TIM_DIER_UIE;
}
//-----------------------------
void TIM_Backlight_Update(void)
{
    TIM16->DIER &= ~TIM_DIER_UIE;
    TIM16->EGR  |= TIM_EGR_UG;
    TIM16->SR &= ~TIM_SR_UIF;
    TIM16->DIER |= TIM_DIER_UIE;
}
//---------------------
void TIM_INT_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
    
    TIM17->PSC   = F_CPU/1000000UL - 1;
    TIM17->ARR   = 5000; // 5 ms сигнал INT
    TIM17->DIER |= TIM_DIER_UIE;
    TIM17->CR1  |= TIM_CR1_OPM;
    
    NVIC_EnableIRQ(TIM17_IRQn);
}
//----------------------
void TIM_INT_Start(void)
{
    TIM17->CR1 |= TIM_CR1_CEN;
}
//----------------------
void DEV_PWROKInit(void)
{
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    
    GPIOA->MODER &= ~GPIO_MODER_MODER12;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR12;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR12_1; // pull down
    
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI12_PA;
    
    EXTI->IMR  |= EXTI_IMR_MR12;
    EXTI->RTSR |= EXTI_RTSR_TR12;
    
    EXTI->PR = EXTI_PR_PR12; // clear flag
	
    TIM14->PSC   = 48000 - 1; // 1ms
    TIM14->ARR   = 12 - 1;
    TIM14->CR1  |= TIM_CR1_ARPE | TIM_CR1_OPM;
    TIM14->DIER &= ~TIM_DIER_UIE;
    TIM14->EGR  |= TIM_EGR_UG;
    TIM14->SR   &= ~TIM_SR_UIF;
    TIM14->DIER |= TIM_DIER_UIE;
    
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    NVIC_SetPriority(EXTI4_15_IRQn, 0);
	
    NVIC_EnableIRQ(TIM14_IRQn);
    NVIC_SetPriority(TIM14_IRQn, 0);
    
    TIM14->CR1 |= TIM_CR1_CEN;
}
//----------------------
void DEV_CrashInit(void)
{
    // настройка аварийного выхода
    out_crash        = &io_out->list[2];
    HAL_GPIO_WritePin(out_crash->pin.gpio, out_crash->pin.io, GPIO_PIN_RESET); // выключаем аварийный выход при включении модуля (включается, если опроса не было 5 сек)
    TIM_Init_Crash(); // запуск таймера
}
//-----------------------
uint8_t DEV_Address(void)
{
    return devAddr;
}
//------------------------------------------------------
bool DEV_Request(FS9Buffer_t* source, FS9Buffer_t* dest)
{
    is_crash = true; // получили запрос от ЦП - сброс аварийной ситуации
   
    ERROR_request_inc(); // увеличиваем счетчик запросов, если адрес устройства верный
    
    uint8_t checksum = DEV_Checksum(source, source->size - 1);
    
    if(checksum != (uint8_t)source->data[source->size - 1]) // ошибка контрольной суммы
    {
        ERROR_checksum_inc(); // увеличиваем счетчик ошибок контрольной суммы
        
        return false;
    }
    
    FS9Buffer_t data = { 0 };
    
    data.cmd      = dest->cmd = source->cmd;
    data.cmd_code = dest->cmd_code = source->cmd_code;
    data.size     = source->size - 2;
    
    if(data.size > 0)
    {
        memcpy(&data.data[0], &source->data[1], sizeof(data.data[0])*data.size);
    }
    
    bool answer = DEV_Driver(&data, dest);
    
    if(!answer && dest->cmd_code != 0x19) // если получили ложь и код команды не равен 0x19 (искробезопасные входы)
        return false; // ответа нет
    
    if(dest->cmd.is_ack)
    {
        if(dest->cmd_code == 0x19 && !answer) // если команда (запись искробезопасных входов) и ответ ложь
            dest->data[dest->size++] = NAK; // отправляем отказ
        else
            dest->data[dest->size++] = ACK; // в другом любом случае отправляем подтверждение
    }
    
    // append checksum for packet
    checksum = DEV_Checksum(dest, dest->size);
    dest->data[dest->size++] = checksum;
    
    return true;
}
//-----------------------------------------------------
bool DEV_Driver(FS9Buffer_t* source, FS9Buffer_t* dest)
{
    uint8_t   bit_count = 0; // счетчик бит (позиция канала в байте)
    int32_t   temp      = 0x00;
    uint8_t   byte      = 0x00;
    uint8_t   state     = 0x00;
    uint8_t   n_out     = 0x00;
    output_t* out       = NULL;
    
    union
    {
        uint16_t count;
        uint8_t  byte[2];
    } utemp;
    
    utemp.count = 0x0000;
    
    switch(source->cmd_code)
    {
        case 0x00: // чтение дискретных каналов входов
            // чтения байт из снимка состояний входов
            dest->data[0] = int_state.state[0];
            dest->data[1] = int_state.state[1];
            dest->data[2] = int_state.state[2];
            dest->size = 3;

            GPIO_INT->BSRR |= GPIO_INT_SET; // поднимаем сигнал INT
				
						int_reset_count = 0; // сброс переменной хранящей количество сброса сигнала INT
						int_watchdog_count = 0;
            
            if(int_reset_id != 0xFF)
            {
                kill_task(int_reset_id);
                int_reset_id = 0xFF;
                int_state.mode = INT_STATE_IDLE;
            }

            if(int_state.mode == INT_STATE_IDLE)
            {
                int_state.mode = INT_STATE_TIMEOUT; // включение режима ожидания таймаута перед очередной отправкой
                EVENT_Create(5, false, int_timeout, NULL, 0xFF); // создание задачи ожидания таймаута (5мс)
            }
        break;
            
        case 0x01: // чтение дискретных каналов выходов
            dest->data[0] = 0x00;
        
            for(uint8_t i = 0; i < io_out->size; ++i)
            {
                io_t pin = io_out->list[i].pin;
                
                if((pin.gpio->ODR & pin.io) == pin.io)
                {
                    dest->data[0] |= 0x01 << i;
                }
            }
            
            dest->size = 1;
        break;
            
        case 0x02: // чтение аналоговых величин 1..4
            while(AIN_Is_Ready() == false); // ожидание готовности результатов
            
            temp = AIN_Get_Temperature();
            
            union float_t t;
            
            if(devAddr == 0x00)
            {
                uint16_t vdda = AIN_Get_VDDA();
                
                t.number = ((float)(AIN_Get_Channel_1()/4095.0f*vdda/1000.0f));
            }
            else if(devAddr == 0x01)
            {
                uint16_t temp = ((float)AIN_Get_Channel_1()/4095)*AIN_TEMP[MAX_SIZE_AIN_TEMP - 1][1];
                t.number = Get_Temp(temp, 1);
            }
            else if(devAddr == 0x02)
            {
                t.number = 0.0f;
            }
            
            dest->data[0] = t.byte[0];
            dest->data[1] = t.byte[1];
            dest->data[2] = t.byte[2];
            dest->data[3] = t.byte[3];
        
            if(devAddr == 0x00)
            {
                uint16_t vdda = AIN_Get_VDDA();
                t.number = ((float)AIN_Get_Channel_2()/4095)*vdda/1000.0f; // шунт 0.1 Ом, усиление ОУ 20, делитель 2
                //t.number /= 0.1f;                         // т.е. значение пропорционально току
            }
            else if(devAddr == 0x01)
            {
                uint16_t temp = ((float)AIN_Get_Channel_2()/4095)*AIN_TEMP[MAX_SIZE_AIN_TEMP - 1][2];
                t.number = Get_Temp(temp, 2);
            }
            else if(devAddr == 0x02)
            {
                t.number = 0.0f;
            }
            
            dest->data[4] = t.byte[0];
            dest->data[5] = t.byte[1];
            dest->data[6] = t.byte[2];
            dest->data[7] = t.byte[3];
        
            t.number = temp/1000.0f;
        
            dest->data[8]  = t.byte[0];
            dest->data[9]  = t.byte[1];
            dest->data[10] = t.byte[2];
            dest->data[11] = t.byte[3];

            if(devAddr == 0x00)
            {
                t.number = DS18B20_Temperature();
            }
            else
            {
                t.number = 0.0f;
            }
            
            dest->data[12] = t.byte[0];
            dest->data[13] = t.byte[1];
            dest->data[14] = t.byte[2];
            dest->data[15] = t.byte[3];
        
            dest->size = 16;
        break;
            
        case 0x03: // чтение регистра расширения дискретных каналов входов (кнопки МИК-01)
            // чтения байт из снимка состояний входов
            dest->data[0] = int_state.state[0];
            dest->data[1] = int_state.state[1];
            dest->data[2] = int_state.state[2];
            dest->size = 3;

            GPIO_INT->BSRR |= GPIO_INT_SET; // поднимаем сигнал INT
            
            if(int_reset_id != 0xFF)
            {
                kill_task(int_reset_id);
                int_reset_id = 0xFF;
                int_state.mode = INT_STATE_IDLE;
            }

            if(int_state.mode == INT_STATE_IDLE)
            {
                int_state.mode = INT_STATE_TIMEOUT; // включение режима ожидания таймаута перед очередной отправкой
                EVENT_Create(5, false, int_timeout, NULL, 0xFF); // создание задачи ожидания таймаута (5мс)
            }
        break;
            
        case 0x04: // чтение регистра расширения дискретных каналов выходов
            for(uint8_t i = 0; i < io_out->size; ++i)
            {
                out = &io_out->list[i];
                
                if(bit_count == 8)
                {
                    bit_count = 0;
                    dest->data[++dest->index] = 0x00;
                }
                
                uint8_t channel_state = 0x00;
                
                if(out->state == OUTPUT_STATE_ON)
                {
                    channel_state = 0x01;
                }
                else if(out->state == OUTPUT_STATE_FREQ_2HZ)
                {
                    channel_state = 0x02;
                }
                else if(out->state == OUTPUT_STATE_RESERVE)
                {
                    channel_state = 0x03;
                }
                
                dest->data[dest->index] |= channel_state << bit_count;
                
                bit_count += 2;
            }
                
            dest->size = 3;
        break;
            
        case 0x05: // запись регистра расширения дискретных каналов выходов
            for(uint8_t i = 0; i < source->size; ++i)
            {
                byte = source->data[i]; // текущий байт
                
                for(uint8_t j = 0; j < 8; j += 2) // 8 бит по 2 на описание каждого канала
                {
                    state = (byte >> j)&0x03; // состояние текущего канала
                    n_out = i*4 + j/2; // порядковый номер канала
                    out   = &io_out->list[n_out];
                    
                    switch(state)
                    {
                        case OUTPUT_STATE_OFF: // отключение выхода
                            if(out->state == OUTPUT_STATE_OFF)
                            {
                                break;
                            }
                            else if(out->state == OUTPUT_STATE_FREQ_2HZ || out->state == OUTPUT_STATE_RESERVE)
                            {
                                kill_task(n_out);
                            }
                            
                            out->state = OUTPUT_STATE_OFF;

                            DEV_OutReset(out);
                        break;
                        
                        case OUTPUT_STATE_ON: // включение выхода
                            if(out->state == OUTPUT_STATE_ON)
                            {
                                break;
                            }
                            else if(out->state == OUTPUT_STATE_FREQ_2HZ || out->state == OUTPUT_STATE_RESERVE)
                            {
                                kill_task(n_out);
                                DEV_OutReset(out);
                            }
                            
                            out->state = OUTPUT_STATE_ON;
                            
                            DEV_OutSet(out);
                        break;
                        
                        case OUTPUT_STATE_FREQ_2HZ: // включение выхода с альтернативной функцией
                        case OUTPUT_STATE_RESERVE:
                            if(out->state == OUTPUT_STATE_FREQ_2HZ || out->state == OUTPUT_STATE_RESERVE)
                            {
                                break;
                            }
                            else if(out->state == OUTPUT_STATE_ON)
                            {
                                DEV_OutReset(out);
                            }
                            
                            out->state = OUTPUT_STATE_FREQ_2HZ;
                            insert_task(n_out);
                        break;
                    }
                }
            }
        break;
        
        case 0x06: // установка значения 0 на выходе канала 0
            CHANNEL_Out_Reset(0);
        break;
            
        case 0x07: // установка значения 0 на выходе канала 1
            CHANNEL_Out_Reset(1);
        break;
            
        case 0x08: // установка значения 0 на выходе канала 2
            CHANNEL_Out_Reset(2);
        break;
            
        case 0x09: // установка значения 0 на выходе канала 3
            CHANNEL_Out_Reset(3);
        break;
            
        case 0x0A: // установка значения 0 на выходе канала 4
            CHANNEL_Out_Reset(4);
        break;
            
        case 0x0B: // установка значения 0 на выходе канала 5
            CHANNEL_Out_Reset(5);
        break;
            
        case 0x0C: // установка значения 0 на выходе канала 6
            CHANNEL_Out_Reset(6);
        break;
            
        case 0x0D: // установка значения 0 на выходе канала 7
            CHANNEL_Out_Reset(7);
        break;
            
        case 0x0E: // установка значения 1 на выходе канала 0
            CHANNEL_Out_Set(0);
        break;
            
        case 0x0F: // установка значения 1 на выходе канала 1
            CHANNEL_Out_Set(1);
        break;
            
        case 0x10: // установка значения 1 на выходе канала 2
            CHANNEL_Out_Set(2);
        break;
            
        case 0x11: // установка значения 1 на выходе канала 3
            CHANNEL_Out_Set(3);
        break;
            
        case 0x12: // установка значения 1 на выходе канала 4
            CHANNEL_Out_Set(4);
        break;
            
        case 0x13: // установка значения 1 на выходе канала 5
            CHANNEL_Out_Set(5);
        break;
            
        case 0x14: // установка значения 1 на выходе канала 6
            CHANNEL_Out_Set(6);
        break;
            
        case 0x15: // установка значения 1 на выходе канала 7
            CHANNEL_Out_Set(7);
        break;
        
        case 0x18: // чтение байта конфигурации искробезопасных входов
            if(devAddr != DEVICE_MDVV_01) // если устройство не МДВВ-01,
            {
                dest->data[0] = SPARK_SECURITY_MODE_NONE; // то возвращаем ответ - нет искробезопасных входов
            }
            else
            {
                dest->data[0] = io_in->list[0].spark_security; // то возвращаем текущую настройку первого входа
                                                                   // остальные настроены однотипно (блок)
            }
            
            dest->size = 1;
        break;
        
        case 0x19: // запись байта конфигурации искробезопасных входов
            if(devAddr == DEVICE_MDVV_01 && source->size == 1) // настройка искробезопасных входов только для МДВВ-01
            {
                if((int8_t)source->data[0] >= SPARK_SECURITY_MODE_NONE && (int8_t)source->data[0] <= SPARK_SECURITY_MODE_3)
                {
                    // присваиваем новое значение режима для четырех искробезопасных входов
                    io_in->list[0].spark_security = io_in->list[1].spark_security = io_in->list[2].spark_security =
                    io_in->list[3].spark_security = source->data[0];
                    
                    // в режимах №2 и №3 частота входного сигнала 50Гц
                    if(source->data[0] == SPARK_SECURITY_MODE_2 || source->data[0] == SPARK_SECURITY_MODE_3)
                        io_in->list[0].duration = io_in->list[1].duration = io_in->list[2].duration = io_in->list[3].duration = 20;
                    else
                        io_in->list[0].duration = io_in->list[1].duration = io_in->list[2].duration = io_in->list[3].duration = 10;
                }
                else
                    return false;
            }
            else
                return false;
        break;
            
        case 0x1B: // чтение общих настроек фильтра дискретных входов
            dest->data[0] = io_in->set.Nperiod; // количество периодов фильтрации
            dest->data[1] = io_in->set.Ndiscret; // дискретность (количество выборок на период)
            dest->data[2] = io_in->set.SGac; // длительность сигнала считаемая, что сигнал валидный
        
            dest->size = 3;
        break;
        
        case 0x1C: // чтение настроек дискретного входа
        {
            uint8_t input = source->data[0];
            if(input >= io_in->size)
                return false;
            
            uint8_t byte = 0;
            
            if(io_in->list[input].mode != IN_MODE_AC) // режим входа "постоянный"
            {
                byte = (1 << 7); // устанавливаем 8 бит в единицу (отвечает за режим работы входа - в данном случае DC)
            }
            
            byte |= (io_in->list[input].fault&0x7F); // остальные 7 битов содержат погрешность в %
            
            dest->data[0] = input; // возвращаем номер входа
            dest->data[1] = byte; // режим входа + его погрешность
            dest->data[2] = io_in->list[input].duration; // длительность периода
            
            dest->size = 3;
        }
        break;
            
        case 0x1D: // чтение отладочной информации (счетчиков ошибок)
            utemp.count = ERROR_request();; // чтение счетчика количества запросов
            
            dest->data[0] = utemp.byte[0];
            dest->data[1] = utemp.byte[1];
            
            utemp.count = ERROR_command(); // чтение счетчика ошибок команд
            
            dest->data[2] = utemp.byte[0];
            dest->data[3] = utemp.byte[1];
            
            utemp.count = ERROR_checksum(); // чтение счетчика ошибок контрольной суммы
            
            dest->data[4] = utemp.byte[0];
            dest->data[5] = utemp.byte[1];
        
            utemp.count = ERROR_no_process(); // чтение счетчика ошибок отсутствия обработчика команды
            
            dest->data[6] = utemp.byte[0];
            dest->data[7] = utemp.byte[1];
        
            utemp.count = ERROR_overrun(); // чтение счетчика ошибок переполнения
            
            dest->data[8] = utemp.byte[0];
            dest->data[9] = utemp.byte[1];
            
            utemp.count = ERROR_timeout(); // чтение счетчика ошибок таймаута
            
            dest->data[10] = utemp.byte[0];
            dest->data[11] = utemp.byte[1];
        
            // структура инициализируется при передаче нулями, поэтому нет смысла обнулять резервные ячейки
            
            dest->size = 16;
        break;
        
        case 0x1E:
            for(int i = 0; i < sizeof(deviceSN); i++)
            {
                dest->data[i] = deviceSN[i];
            }
        
            dest->size = sizeof(deviceSN);
        break;
            
        case 0x1F: // чтение времени срабатывания выделенного входного дискретного канала
        {
            uint16_t IN_time = 0x0000;
            uint8_t state = DSDIN_FUNCTION_NOT_SUPPORT;

            if(devAddr == DEVICE_MDVV_01)
            {
                state = DSDIN_TRIGGER_OFF;

                if(FLASH_Unlock())
                {
                    uint32_t dsdin = FLASH_Read(FLASH_BASE_ADDRESS);

                    if(dsdin != FLASH_CELL_EMPTY)
                    {
                        uint32_t IN_change = (dsdin&0x00100000) >> 20; // получаем флаг фиксации изменения состояния входа
                        uint32_t IN_state = (dsdin&0x00010000) >> 16; // получаем флаг состояния входа
                        
                        if(IN_change == 1)
                        {
                            state = (IN_state == 1)?DSDIN_TRIGGER_ON_1:DSDIN_TRIGGER_ON_0;
                            IN_time = dsdin&0x0000FFFF;
                        }
                        else
                            state = DSDIN_TRIGGER_OFF;

                        FLASH_Erase(FLASH_BASE_ADDRESS); // стирание данных со страницы
                    }
                }
                FLASH_Lock();
            }

            dest->data[0] = state;
            dest->data[1] = (uint8_t)(IN_time&0x00FF);
            dest->data[2] = (uint8_t)((IN_time&0xFF00) >> 8);
            dest->size    = 3;
        }
        break;
					
        case 0x3A: // запись серийного номера
            if(FLASH_Unlock())
            {
                key_read = FLASH_Read(FLASH_SERIAL_ADDRESS);
                key_receive = ((source->data[0] << 24) | (source->data[1] << 16) | (source->data[2] << 8) | source->data[3]);
                
                if(key_read != key_receive)
                {
                    FLASH_Lock();
                    return false;
                }
                
                if(FLASH_Erase(FLASH_SERIAL_ADDRESS))
                {
                    uint32_t key_new = ((source->data[4] << 24) | (source->data[5] << 16) | (source->data[6] << 8) | source->data[7]);
                    
                    if(key_new != 0x00000000)
                    {
                        FLASH_Write(FLASH_SERIAL_ADDRESS, key_new);
                    }
                    else
                        FLASH_Write(FLASH_SERIAL_ADDRESS, key_receive);
                }
                
                uint32_t data = 0;
                
                deviceSN[1] = source->data[8];
                deviceSN[2] = source->data[9];
                deviceSN[3] = source->data[10];
                deviceSN[4] = source->data[11];
                
                data = ((deviceSN[1] << 24) | (deviceSN[2] << 16) | (deviceSN[3] << 8) | deviceSN[4]);
                FLASH_Write(FLASH_SERIAL_ADDRESS + 4, data);
                
                FLASH_Lock();
            }
        break;
        
        case 0x3E: // изменение параметров фильтрации
            if(source->size == 3)
            {
                io_in->set.Nperiod  = source->data[0]; // количество периодов фильтрации
                io_in->set.Ndiscret = source->data[1]; // количество выборок на период
                io_in->set.SGac     = source->data[2]; // длительность сигнала считаемая, что сигнал валидный
                
                TIM_Scan_Update();
            }
        break;
            
        case 0x3F: // установка настроек входов
//          --------------------------------------------------------------------------------
//          | группа входов - 2 байта | режим (AC|DC) | длительность периода | погрешность |
//          --------------------------------------------------------------------------------
            if(source->size == 5)
            {
                uint16_t input = ((source->data[0] << 8) | source->data[1]);

                for(int i = 0; i < io_in->size; i++)
                {
                    if(input & (1 << i)) // вход выбран
                    {
                        inputSettings(i, source->data[2], source->data[3], source->data[4]);
                    }
                }
            }
        break;
        
        default:
        {
            ERROR_no_process_inc(); // увеличиваем счетчик - нет обработчика команды
            return false;
        }
    };
    
    return true;
}
//-------------------------------------------------------------------------------
void inputSettings(uint8_t number, uint8_t mode, uint8_t duration, uint8_t fault)
{
	if(duration == 0) // защита от дурака - если длительность равна 0, то отменяем настройку
		return;
	
    io_in->list[number].mode  = mode; // режим работы входа AC или DC
    io_in->list[number].duration = duration; // длительность периода

    if(mode == IN_MODE_AC)
    {
        io_in->list[number].fault = (fault > 100)?100:fault; // погрешность допускаемая за один период - в процентах
    }
    else
    {
        io_in->set.P0dc = io_in->set.P1dc = (fault > 100)?100:fault;
    }
}
//-----------------------------------------------------
uint8_t DEV_Checksum(FS9Buffer_t* packet, uint8_t size)
{
    uint8_t checksum = 0;
    
    for(uint8_t i = 0; i < size; ++i)
    {
        checksum += (uint8_t)(packet->data[i]&0x00FF);
    }
    
    checksum += size;
    checksum ^= 0xFF;
    
    return checksum;
}
//----------------------
void DEV_InputScan(void)
{    
    if(_pwr_ok.state == false)
    {
        for(uint8_t i = 0; i < io_in->size; ++i)
        {
            DEV_InputFilter(i);
        }
    }
    else
    {
        DEV_InputFilter(4);
    }
    
    if(InputStateChanged) // состояние входов изменилось
    {
        if(int_state.mode == INT_STATE_IDLE) // если данные уже были прочитаны МЦП
        {
            inputStateUpdate();
        }
    }
}
//---------------------------------
void DEV_InputFilter(uint8_t index)
{
    input_t* input = &io_in->list[index];
    
    bool in_state  = input->pin.gpio->IDR & input->pin.io;
    bool act_level = !input->state; // ожидаемый уровень (при выключенном входе - лог "1", при включенном лог "0")
    
    // если уровень на входе равен ожидаемому уровню (обратное значение от состояния входа)
    if(in_state == act_level && input->filter.is_capture == false)
    {
        input->filter.is_capture = true; // захватываем вход
        
        if(in_state == true)
            input->filter.c_lev_1++;
        else
            input->filter.c_lev_0++;
        
        input->filter.c_clock++;
    }
    else if(input->filter.is_capture == true) // если вход захвачен, то набираем данные для фильтрации
    {
        if(in_state == true)
            input->filter.c_lev_1++;
        else
            input->filter.c_lev_0++;
        
        input->filter.c_clock++;
        
        if(input->filter.c_clock >= input->duration) // набрали данные на очередной период
        {
            input->filter.c_period++;
            
            if(input->mode == IN_MODE_AC) // режим входа АС
            {
                // если ожидаемый уровень лог "1" и количество лог "0" равно нулю, то это либо постоянный
                // сигнал, либо сигнал с частотой меньше установленно, т.е. это ошибка
                if(act_level == true && input->filter.c_lev_0 == 0)
                {
                    input->filter.c_error++;

                    input->filter.c_clock = 0;
                    input->filter.c_lev_0 = 0;
                    input->filter.c_lev_1 = 0;

                    return;
                }
                
                // если ожидаемый уровень лог "1" и длительность сигнала меньше пороговой, то это ошибка
                if(act_level == true && input->filter.c_lev_1 < io_in->set.SGac)
                {
                    input->filter.c_error++;

                    input->filter.c_clock = 0;
                    input->filter.c_lev_0 = 0;
                    input->filter.c_lev_1 = 0;

                    return;
                }
                
                // если ожидаемый уровень лог "0" (снятие сигнала со входа) и длительность лог "1"
                // больше нуля, то значит это не снятие сигнала и нет смысла дальше анализировать
                if(act_level == false && input->filter.c_lev_1 > 0 && (input->spark_security != SPARK_SECURITY_MODE_2 || 
                                                                       input->spark_security != SPARK_SECURITY_MODE_3))
                {
                    input->filter.c_clock    = 0;
                    input->filter.c_error    = 0;
                    input->filter.c_lev_0    = 0;
                    input->filter.c_lev_1    = 0;
                    input->filter.c_period   = 0;
                    input->filter.c_state    = 0;
                    input->filter.is_capture = false;
                    
                    return;
                }
                
                uint16_t tdur = (act_level == true)?(input->filter.c_lev_0 + input->filter.c_lev_1):
                                                     input->filter.c_lev_0;
                
                // определяем количество тиков до перезагрузки таймера сканирования, т.е. чему равно одно прерывание
                // процессор настроен на 48 МГц, т.е. таймер сканирования считает 1 тик = 1 мкс
                uint16_t tick_count = 10000/io_in->set.Ndiscret; // количество мкс до перезагрузки таймера, н-р: дискретность равна 10, тогда 1000 = 1мс
                
                // расчет частоты входного сигнала исходя из счетчиков нулей и единиц (длительность сигнала tdur)
                uint16_t frequency = tick_count/tdur; // н-р: единиц и нулей по 5, тогда 1000/10 = 100Гц
                uint16_t fault     = (tick_count/io_in->set.Ndiscret)*input->fault/100; // погрешность частоты, н-р: частота 100Гц, 
                                                                                        // погрешность 10% - 100*10/100 = 10Гц
                
                input->frequency = frequency; // обновляем переменную предыдущего значения частоты сигнала
                
                if(input->spark_security == SPARK_SECURITY_MODE_NONE || input->spark_security == SPARK_SECURITY_MODE_1)
                {
                    // обрабатываются не искробезопасные входы, либо искробезопасные в режиме №1
                    if(frequency >= (100 - fault) && frequency <= (100 + fault)) // частота в пределах 100Гц
                        input->filter.c_state++;
                    else
                        input->filter.c_error++;
                }
                else if(input->spark_security == SPARK_SECURITY_MODE_2 || input->spark_security == SPARK_SECURITY_MODE_3)
                {
                    /*! обработка искробезопасных входов в режиме №2 и №3 (сигнал на входе 50Гц)
                     *  Алгоритм работает следующим образом:
                     *  Вход ON фиксирует входной сигнал только в том случае, если на входе DI_1 и DI_3 (ПУСК) присутсвтвует сигнал 50Гц в противофазе
                     *  Состояние входа DI_3 (ПУСК) считается ошибкой, если сигнал находится в фазе с сигналом на входе DI_1 (прямое включение диода)
                     *  Состосние входа DI_2 (СТОП) считается ошибкой, если сигнал 50Гц на входе DI_1 присутствует, а налинии DI_2 нет (обрыв линии)
                     *  Состояние на любом из искробезопасных входов DI_1 - DI_4 является ошибкой, если частота сигнала меньше или больше 50Гц
                     */
                    
                    // мгновенные значения состояний входов DI_1, DI_2 (СТОП) и DI_3 (ПУСК)
                    bool io_phaseState = io_inPhase->pin.gpio->IDR & io_inPhase->pin.io;
                    bool io_offState   = io_inOff->pin.gpio->IDR & io_inOff->pin.io;
                    bool io_onState    = io_inOn->pin.gpio->IDR & io_inOn->pin.io;
                    
                    if((frequency >= (50 - fault) && frequency <= (50 + fault)) == false) // частота меньше или больше 50Гц с учетом погрешности
                    {
                        input->filter.c_error++;
                    }
                    else
                    {
                        if(input == io_inOn)
                        {
                            if(!input->state && io_onState && (!io_phaseState && !io_offState) && (io_inPhase->state && io_inOff->state))
                                input->filter.c_state++;
                            else if(input->state && !io_onState)
                                input->filter.c_state++;
                            else if(io_onState && io_offState && io_phaseState && input->spark_security == SPARK_SECURITY_MODE_2)
                            {
                                // ошибка включения диода на линии ON (должен быть влкючен в обратном) - только для режим №2 (в режиме №3 норма)
                                input->filter.c_error++;
                            }
                        }
                        else
                            input->filter.c_state++;
                    }
                }
            }
            else if(input->mode == IN_MODE_DC) // режим входа DC
            {
                uint16_t tfault_lev = ((act_level == true)?input->duration*io_in->set.P1dc:
                                                           input->duration*io_in->set.P0dc)/100;
                uint16_t tdur = (act_level == true)?input->filter.c_lev_1:input->filter.c_lev_0;
                
                if(tdur >= tfault_lev)
                    input->filter.c_state++;
            }
            
            input->filter.c_clock = 0;
            input->filter.c_lev_0 = 0;
            input->filter.c_lev_1 = 0;
        }
        
        if(input->filter.c_period >= io_in->set.Nperiod) // конец фильтрации - принятие решения
        {
            if(input->filter.c_state >= (io_in->set.Nperiod - 1))
            {
                // обработка состояния ошибки для искробезопасного входа в режиме 2 и №3
                if(input == io_inOff && !act_level && io_inPhase->state && (input->spark_security == SPARK_SECURITY_MODE_2 || 
                                                                            input->spark_security == SPARK_SECURITY_MODE_3))
                {
                    input->error  = true;
                    input->state  = false;
                    InputStateChanged = true;
                }
                else
                {
                    input->error  = false;
                    input->state  = act_level;
                    InputStateChanged = true;
                    
                    if(_pwr_ok.state == true && index == PWROK_INPUT)
                    {
                        if(_pwr_ok.IN_change == false)
                        {
                            _pwr_ok.IN_state  = input->state;
                            _pwr_ok.IN_time   = TIM14->CNT;
                            _pwr_ok.IN_change = true;

                            // Сохраняем состояние и время во флеш в формате:
                            // 0xXXCSTTTT, где X - indefinite state, C - change, S - state, T - time (2 bytes)
                            if(FLASH_Unlock()) // перезаписываем данные на странице настроек
                            {
                                if(FLASH_Erase(FLASH_BASE_ADDRESS))
                                {
                                    uint32_t pwrok = _pwr_ok.IN_time;

                                    if(_pwr_ok.IN_change)
                                        pwrok |= 1 << 20;

                                    if(_pwr_ok.IN_state)
                                        pwrok |= 1 << 16;

                                    FLASH_Write(FLASH_BASE_ADDRESS, pwrok);
                                }
                            }
                            
                            FLASH_Lock();
                        }
                    }
                }
            }
            else if(input->filter.c_error >= io_in->set.Nperiod)
            {
                input->error  = true;
                InputStateChanged = true;
            }
            
            input->filter.c_clock    = 0;
            input->filter.c_error    = 0;
            input->filter.c_lev_0    = 0;
            input->filter.c_lev_1    = 0;
            input->filter.c_period   = 0;
            input->filter.c_state    = 0;
            input->filter.is_capture = false;
        }
    }
}
//-------------------------------
void DEV_KeyboardScan(void* data)
{
    io_t scan;
    
    switch(keys.mode)
    {
        case KEY_MODE_NONE:
            scan = io_in->list[10].pin; // первая сканлиния
            scan.gpio->BSRR |= scan.io << 16; // прижимаем первую сканлинию
        
            keys.mode = KEY_MODE_SCAN_1;
        
            EVENT_Create(5, false, DEV_KeyboardScan, NULL, 0xFF);
        break;
        
        case KEY_MODE_SCAN_1:
            scan  = io_in->list[10].pin;
            keys.temp = scan.gpio->IDR&0x000003FF; // считываем состояние кнопок
            scan.gpio->BSRR |= scan.io; // поднимаем первую сканлинию
        
            scan = io_in->list[11].pin; // вторая сканлиния
            scan.gpio->BSRR |= scan.io << 16; // прижимаем вторую сканлинию
        
            keys.mode = KEY_MODE_SCAN_2;
        
            EVENT_Create(5, false, DEV_KeyboardScan, NULL, 0xFF);
        break;
        
        case KEY_MODE_SCAN_2:
            scan = io_in->list[11].pin;
            keys.temp |= (scan.gpio->IDR&0x000003FF) << 10; // считываем состояние кнопок
            scan.gpio->BSRR |= scan.io; // поднимаем вторую сканлинию
        
            if(keys.is_bounce == false)
            {
                keys.is_bounce = true;
                keys.cur_state = keys.temp;
            }
            else
            {
                if(keys.cur_state == keys.temp)
                {
                    keys.temp ^= KEY_EMPTY_MASK; // инвертируем значение
                    keys.temp &= KEY_EMPTY_MASK; // обрезаем по маске (до 20-ти значащих бит)
                    
                    if(keys.temp != keys.last_state) // предыдущее состояние не равно текущему (произошли изменения)
                    {
                        keys.last_state = keys.temp; // сохраняем текущее состояние входов
                        InputStateChanged = true; // устанавливаем сигнал INT для оповещении ЦП об изменении состояния входов
                    }
                }
                
                keys.is_bounce = false;
                keys.cur_state = KEY_EMPTY_MASK;
            }
            
            keys.temp = KEY_EMPTY_MASK;
            keys.mode = KEY_MODE_NONE;
            
            EVENT_Create(5, false, DEV_KeyboardScan, NULL, 0xFF);
        break;
    }
    
    if(InputStateChanged)
    {
        if(int_state.mode == INT_STATE_IDLE) // если данные уже были прочитаны МЦП
        {
            inputStateUpdate();
        }
    }
}
//----------------------------
void DEV_InputSetDefault(void)
{
    io_in->set.Ndiscret = 10;
    io_in->set.Nperiod  = 3;
    io_in->set.SGac     = 5;
    io_in->set.P0dc     = 50;
    io_in->set.P1dc     = 50;
    
    for(uint8_t i = 0; i < io_in->size; ++i)
    {
        io_in->list[i].mode              = IN_MODE_AC;
        io_in->list[i].fault             = 10;
        io_in->list[i].state             = false;
        io_in->list[i].error             = false;
        io_in->list[i].filter.c_clock    = 0;
        io_in->list[i].filter.c_period   = 0;
        io_in->list[i].filter.c_state    = 0;
        io_in->list[i].filter.c_error    = 0;
        io_in->list[i].filter.is_capture = false;
        
        if(io_in->list[i].spark_security == SPARK_SECURITY_MODE_2 || io_in->list[i].spark_security == SPARK_SECURITY_MODE_3)
            io_in->list[i].duration = 20;
        else
            io_in->list[i].duration = 10;

        if(devAddr == DEVICE_MDVV_01 && i == 4) // пятый вход по умолчанию цифровой
        {
            io_in->list[i].mode = IN_MODE_DC;
        }
    }
}
//-------------------------------------
bool DEV_InputStateChangedChannel(void)
{
    return InputStateChanged;
}
//----------------------------
void EXTI4_15_IRQHandler(void)
{
    if((EXTI->PR & EXTI_PR_PR12) == EXTI_PR_PR12)
    {
        _pwr_ok.is_ok = true;
		
		if(_pwr_ok.state == true)
		{
			_pwr_ok.state = false;
			
			TIM14->DIER &= ~TIM_DIER_UIE;
			TIM14->ARR   = 12 - 1;
			TIM14->EGR  |= TIM_EGR_UG;
			TIM14->SR   &= ~TIM_SR_UIF;
			TIM14->DIER |= TIM_DIER_UIE;
			TIM14->CR1  |= TIM_CR1_CEN;
		}
               
        EXTI->PR = EXTI_PR_PR12; // clear flag
    }
}
//-------------------------
void TIM14_IRQHandler(void)
{
	if((TIM14->SR & TIM_SR_UIF) == TIM_SR_UIF)
	{
		TIM14->SR &= ~TIM_SR_UIF;
		
		if(_pwr_ok.state == false)
		{
			if(_pwr_ok.is_ok == false)
			{
				_pwr_ok.state = _pwr_ok.is_crash = true;
				
				TIM14->DIER &= ~TIM_DIER_UIE;
				TIM14->ARR   = PWROK_TIME - 1;
				TIM14->EGR  |= TIM_EGR_UG;
				TIM14->SR   &= ~TIM_SR_UIF;
				TIM14->DIER |= TIM_DIER_UIE;
				TIM14->CR1  |= TIM_CR1_CEN;
				
				TIM_INT_Start();
			}
			
			_pwr_ok.is_ok = false;
			
			TIM14->CR1 |= TIM_CR1_CEN;
		}
	}
}
//-------------------------
void TIM16_IRQHandler(void)
{
    if((TIM16->SR & TIM_SR_UIF) == TIM_SR_UIF)
    {
        if(devAddr != DEVICE_MIK_01) // если не устройство МИК-01, то запускаем сканирование входов
            DEV_InputScan();
        else
        {
            switch(backlight_mode)
            {
                case MODE_BACKLIGHT_PAUSE_0N:
                    // DEV_OutSet(&io_out->list[0]); // для отладки
                    TIM16->PSC = F_CPU/1000000UL - 1;
                    TIM16->ARR = 10 - 1; // пауза 1 микросекунда - подача импульса яркости
                    TIM_Backlight_Update();
                        
                    GPIO_BACKLIGHT->BSRR |= GPIO_BACKLIGHT_HIGH; // устанавливаем высокий уровень импульса
                    backlight_mode = MODE_BACKLIGHT_PULSE_ON;
                        
                    TIM16->CR1 |= TIM_CR1_OPM | TIM_CR1_CEN;
                break;
                
                case MODE_BACKLIGHT_PULSE_ON:
                    // DEV_OutSet(&io_out->list[1]);  // для отладки
                    TIM16->ARR = 10 - 1; // пауза 1 микросекунда - пауза после импульса яркости
                    TIM_Backlight_Update();
                        
                    GPIO_BACKLIGHT->BSRR |= GPIO_BACKLIGHT_LOW; // устанавливаем низкий уровень импульса
                    backlight_mode = MODE_BACKLIGHT_PULSE_OFF;
                        
                    TIM16->CR1 |= TIM_CR1_OPM | TIM_CR1_CEN;
                break;
                
                case MODE_BACKLIGHT_PULSE_OFF:
                    // DEV_OutSet(&io_out->list[2]);  // для отладки
                    GPIO_BACKLIGHT->BSRR |= GPIO_BACKLIGHT_HIGH; // устанавливаем высокий уровень - подсветка включена 
                                            // (через 500мкс яркость установится в максимальный уровень - по даташиту)
                break;
                
                default:
                    GPIO_BACKLIGHT->BSRR |= GPIO_BACKLIGHT_HIGH; // устанавливаем высокий уровень - подсветка включена 
                                            // (через 500мкс яркость установится в максимальный уровень - по даташиту)
                break;
            }
        }
        
        TIM16->SR &= ~TIM_SR_UIF;
    }
}
//-------------------------
void TIM17_IRQHandler(void)
{
    if((TIM17->SR & TIM_SR_UIF) == TIM_SR_UIF)
    {   
        TIM17->SR &= ~TIM_SR_UIF;
    }
}
//---------------------------------------
void TIM1_BRK_UP_TRG_COM_IRQHandler(void) // обработчик таймера обрыва связи
{
    if((TIM1->SR & TIM_SR_UIF))
    {
        TIM1->SR &= ~TIM_SR_UIF;
        
        if(!(TIM1->CR1 & TIM_CR1_OPM)) // флаг одиночного запуска не выставлен - рабочий режим
        {
            if(is_crash == true) // запрос пришел
            {
                is_crash = false;
            }
            else // запроса нет - включаем выход и держим включенным до тех пор пока ЦПУ не даст команду на отключение
            {
                HAL_GPIO_WritePin(out_crash->pin.gpio, out_crash->pin.io, GPIO_PIN_SET);
            }
        }
        else // режим 5ти секундной задержки перед стартом аварийного таймера
        {
            TIM1->CR1 = (TIM1->CR1 & ~TIM_CR1_OPM) | TIM_CR1_CEN; // запуск таймера в рабочем режиме
            HAL_GPIO_WritePin(out_crash->pin.gpio, out_crash->pin.io, GPIO_PIN_RESET); // снимаем сигнал с аварийного реле
        }
    }
}
//------------------------------------------
float Get_Temp(uint16_t val, uint8_t in_num)
{
    float Rt    = UAIN_to_TResistance(val, in_num);
    float Pt100 = 3383.8098f - 8658.0088f*sqrtf(0.1758481f - 0.000231f*Rt);
	
	if(Pt100 >= 250.0f) // превышение температуры свыше 250 градусов - ошибка (код +500)
		Pt100 = 500.0f;
	else if(Pt100 <= -50.0f) // превышение температуры ниже -50 градусов - ошибка (код -500)
		Pt100 = -500.0f;
    
    return Pt100;
}
//-----------------------------------------------------
float UAIN_to_TResistance(uint16_t val, uint8_t in_num)
{
    // значение val приходит умноженное на 10 (значение ацп целочисленное, т.е. ацп*1000), т.к. в таблице значения
    // хранятся умноженные на 10000 для приведения к целочисленному виду
    float res_beg = 0;
    float res_end = 0;
    float ain_beg = 0;
    float ain_end = 0;
    
    for(uint8_t i = 1; i < 19; ++i)
    {
        if(AIN_TEMP[i - 1][in_num] == val)
            return AIN_TEMP[i - 1][0];
        else if(val > AIN_TEMP[i - 1][in_num] && val < AIN_TEMP[i][in_num])
        {
            res_beg = AIN_TEMP[i - 1][0];
            res_end = AIN_TEMP[i][0];
            ain_beg = AIN_TEMP[i - 1][in_num];
            ain_end = AIN_TEMP[i][in_num];
            
            break;
        }
    }
    
    if(res_beg == 0 && res_end == 0) // not equals value
    {
        return AIN_TEMP[MAX_SIZE_AIN_TEMP - 1][0]; // return max value
    }
    
    float Rt = res_beg + ((val - ain_beg)/(ain_end - ain_beg))*(res_end - res_beg);
    
    return Rt;
}
//-------------------------
void blink2Hz(void* output)
{    
    if(out_queue_blink.count == 0)
        return;
    
    for(uint8_t i = 0; i < MAX_SIZE_QUEUE_OUT; ++i)
    {
        if(out_queue_blink.queue[i] != 0xFF)
        {
            output_t* out = &io_out->list[out_queue_blink.queue[i]];
            
            if(out_queue_blink.state == true)
            {
                DEV_OutSet(out);
            }
            else
            {
                DEV_OutReset(out);
            }
        }
    }
    
    out_queue_blink.state = !out_queue_blink.state;
}
//----------------------
void crash(void* output)
{
    output_t* out = ((output_t*)output);
    
    if(is_crash == true) // запрос пришел
    {
        is_crash = false;
    }
    else // запроса нет - включаем выход и держим включенным до тех пор пока ЦПУ не даст команду на отключение
    {
        HAL_GPIO_WritePin(out->pin.gpio, out->pin.io, GPIO_PIN_SET);
    }
}
//---------------------------
void int_timeout(void* param)
{
    GPIO_INT->BSRR |= GPIO_INT_SET; // поднимаем сигнал INT
	
    if(int_state.mode == INT_STATE_TIMEOUT)
    {
        int_state.mode = INT_STATE_IDLE;
    
        if(int_reset_id != 0xFF)
        {
            kill_task(int_reset_id);
            int_reset_id = 0xFF;
        }
    }
    else if(int_state.mode == INT_STATE_RESET) // обработка сброса на линии INT
    {
        int_reset_count++; // инкремент переменной хранящей количество сброса сигнала INT
    
        if(int_reset_count > 50) // количество сбросов больше 50
        {
            if(int_reset_id != 0xFF)
            {
                    kill_task(int_reset_id);
                    int_reset_id = 0xFF;
            }
            
            int_reset_count = 0; // сброс переменной хранящей количество сброса сигнала INT
            int_watchdog_count = 0;
            int_state.mode = INT_STATE_IDLE;
            
            return;
        }
        
        GPIO_INT->BSRR |= GPIO_INT_RESET; // прижимаем сигнал INT к нулю (режим .mode не меняем)
        int_reset_id = EVENT_Create(100, false, int_timeout, NULL, 0xFF); // создание задачи СБРОСА на линии INT (100мс)
    }
}
//----------------------------
void int_watchdog(void *param)
{
    if((GPIO_INT->BSRR & GPIO_INT_RESET)) // если линия INT прижата
    {
            int_watchdog_count++; // инкрементируем переменную подсчета активного уровня сигнала INT
    }
    
    if(int_watchdog_count > 10) // активным сигнал остается на протяжении 10 проверок, то снимаем сигнал (возможно зависание)
    {
            GPIO_INT->BSRR |= GPIO_INT_SET; // поднимаем сигнал INT
            int_watchdog_count = 0;
            int_state.mode = INT_STATE_IDLE;
        
            if(int_reset_id != 0xFF)
            {
                    kill_task(int_reset_id);
                    int_reset_id = 0xFF;
            }
    }
    
    EVENT_Create(250, false, int_watchdog, NULL, 0xFF); // создание задачи сторожевой собаки для линии линии INT (250мс)
}
//--------------------
void  queue_init(void)
{
    for(uint8_t i = 0; i < MAX_SIZE_QUEUE_OUT; ++i)
    {
        out_queue_blink.queue[i] = 0xFF;
    }

    out_queue_blink.state = false;
    out_queue_blink.count = 0;
}
//--------------------------
void insert_task(uint8_t id)
{
    if(out_queue_blink.count < MAX_SIZE_QUEUE_OUT)
    {
        out_queue_blink.queue[id] = id;
        out_queue_blink.count++;
    }
}
//------------------------
void kill_task(uint8_t id)
{
    if(out_queue_blink.count > 0 && id < MAX_SIZE_QUEUE_OUT)
    {
        out_queue_blink.queue[id] = 0xFF;
        out_queue_blink.count--;
    }
}
