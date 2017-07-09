#ifndef REG_IO_CTRL_H
    #define REG_IO_CTRL_H
    //-----------------
    #include <stdint.h>
    //---------------------------------
    #define DIN_STP_NAC_0 (uint8_t)0x01
    #define DIN_STP_NAC_1 (uint8_t)0x02
    #define DIN_STP_NAC_2 (uint8_t)0x04
    #define DIN_STP_NAC_3 (uint8_t)0x08
    //---------------------------------
    #define DIN_STP_DAC_0 (uint8_t)0x10
    #define DIN_STP_DAC_1 (uint8_t)0x20
    #define DIN_STP_DAC_2 (uint8_t)0x40
    #define DIN_STP_DAC_3 (uint8_t)0x80
    //----------------------------------
    #define DIN_STP_SGAC_0 (uint8_t)0x01
    #define DIN_STP_SGAC_1 (uint8_t)0x02
    #define DIN_STP_SGAC_2 (uint8_t)0x04
    #define DIN_STP_SGAC_3 (uint8_t)0x08
    //----------------------------------
    #define DIN_STP_NSAC_0 (uint8_t)0x10
    #define DIN_STP_NSAC_1 (uint8_t)0x20
    #define DIN_STP_NSAC_2 (uint8_t)0x40
    #define DIN_STP_NSAC_3 (uint8_t)0x80
    //---------------------------------
    #define DIN_STP_NDS_0 (uint8_t)0x01
    #define DIN_STP_NDS_1 (uint8_t)0x02
    #define DIN_STP_NDS_2 (uint8_t)0x04
    #define DIN_STP_NDS_3 (uint8_t)0x08
    //---------------------------------
    #define DIN_STP_DDS_0 (uint8_t)0x10
    #define DIN_STP_DDS_1 (uint8_t)0x20
    #define DIN_STP_DDS_2 (uint8_t)0x40
    #define DIN_STP_DDS_3 (uint8_t)0x80
    //----------------------------------
    #define DIN_STP_P0DC_0 (uint8_t)0x01
    #define DIN_STP_P0DC_1 (uint8_t)0x02
    #define DIN_STP_P0DC_2 (uint8_t)0x04
    #define DIN_STP_P0DC_3 (uint8_t)0x08
    //----------------------------------
    #define DIN_STP_P1DC_0 (uint8_t)0x10
    #define DIN_STP_P1DC_1 (uint8_t)0x20
    #define DIN_STP_P1DC_2 (uint8_t)0x40
    #define DIN_STP_P1DC_3 (uint8_t)0x80
#endif
