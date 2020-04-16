#ifndef __DIG_INPUTS_FILTER_H
#define __DIG_INPUTS_FILTER_H

#include <stdint.h>


typedef struct _io_type
    {
        uint8_t type;        // режим работы входа AC/DC
        uint8_t mode;        // режим работы Ia mode = 1 / noIa mode = 0
        uint8_t fltDuratiod; // длительность фильтрации мс
    } io_TypeDef;

		
uint16_t InputFilter(uint32_t *data, io_TypeDef *io_settings,uint8_t sum_dInputs);


#endif /* __DIG_INPUTS_FILTER_H */
