#ifndef __DIG_INPUTS_FILTER_H
#define __DIG_INPUTS_FILTER_H

#include <stdint.h>


typedef struct _io_type
    {
        uint8_t type;        // ����� ������ ����� AC/DC
        uint8_t mode;        // ����� ������ Ia mode = 1 / noIa mode = 0
        uint8_t fltDuratiod; // ������������ ���������� ��
    } io_TypeDef;

		
uint16_t InputFilter(uint32_t *data, io_TypeDef *io_settings,uint8_t sum_dInputs);


#endif /* __DIG_INPUTS_FILTER_H */
