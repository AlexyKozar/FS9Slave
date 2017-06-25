#include "device.h"
//----------------------
uint8_t  devAddr = 0xFF;
uint16_t devOutput[MAX_SIZE_OUTPUT]; // output chanels
uint8_t  count_chanel = 0; // counter output chanels
uint8_t  ack = 0x06; // responce
//-----------------------------
void dev_set_addr(uint8_t addr)
{
    devAddr = addr;
}
//------------------------
uint8_t dev_get_addr(void)
{
    return devAddr;
}
//--------------------------------------
uint8_t dev_get_size_packet(uint8_t cmd)
{
    uint8_t size = 0;
    
    struct cmd_t tcmd = CMD_get(cmd);
    
    if(tcmd.type != RESERVE)
        size = tcmd.n;
    
    return size;
}
//----------------------------------------
void dev_get_packet(struct packet_t* pack)
{   
    if(pack)
    {
        uint8_t count = pack->size;
        uint8_t chsum = (count - 1) + pack->array[0];
        
        for(uint8_t i = 1; i < count - 1; ++i)
        {
            uint8_t byte = pack->array[i];
            
            chsum += byte;
        }
        
        chsum ^= 0xFF;
        
        if(chsum != pack->array[count - 1]) // checsum is invalid
        {
            pack->size = 0; // error
            return;
        }
        
        struct cmd_t cmd = CMD_get((pack->array[0] & 0x3F)); // get cmd
        
        if(cmd.type == SET_LEVEL_LOW_DSOut_Ch0)
        {
            HAL_GPIO_WritePin(GPIOB, devOutput[0], GPIO_PIN_RESET);
        }
        else if(cmd.type == SET_LEVEL_LOW_DSOut_Ch1)
        {
            HAL_GPIO_WritePin(GPIOB, devOutput[1], GPIO_PIN_RESET);
        }
        else if(cmd.type == SET_LEVEL_LOW_DSOut_Ch2)
        {
            HAL_GPIO_WritePin(GPIOB, devOutput[2], GPIO_PIN_RESET);
        }
        else if(cmd.type == SET_LEVEL_LOW_DSOut_Ch3)
        {
            HAL_GPIO_WritePin(GPIOB, devOutput[3], GPIO_PIN_RESET);
        }
        else if(cmd.type == SET_LEVEL_LOW_DSOut_Ch4)
        {
            HAL_GPIO_WritePin(GPIOB, devOutput[4], GPIO_PIN_RESET);
        }
        else if(cmd.type == SET_LEVEL_LOW_DSOut_Ch5)
        {
            HAL_GPIO_WritePin(GPIOB, devOutput[5], GPIO_PIN_RESET);
        }
        else if(cmd.type == SET_LEVEL_LOW_DSOut_Ch6)
        {
            if(count_chanel >= 6)
            {
                HAL_GPIO_WritePin(GPIOB, devOutput[6], GPIO_PIN_RESET);
            }
        }
        else if(cmd.type == SET_LEVEL_LOW_DSOut_Ch7)
        {
            if(count_chanel == 7)
            {
                HAL_GPIO_WritePin(GPIOB, devOutput[7], GPIO_PIN_RESET);
            }
        }
        else if(cmd.type == SET_LEVEL_HIGH_DSOut_Ch0)
        {
            HAL_GPIO_WritePin(GPIOB, devOutput[0], GPIO_PIN_SET);
        }
        else if(cmd.type == SET_LEVEL_HIGH_DSOut_Ch1)
        {
            HAL_GPIO_WritePin(GPIOB, devOutput[1], GPIO_PIN_SET);
        }
        else if(cmd.type == SET_LEVEL_HIGH_DSOut_Ch2)
        {
            HAL_GPIO_WritePin(GPIOB, devOutput[2], GPIO_PIN_SET);
        }
        else if(cmd.type == SET_LEVEL_HIGH_DSOut_Ch3)
        {
            HAL_GPIO_WritePin(GPIOB, devOutput[3], GPIO_PIN_SET);
        }
        else if(cmd.type == SET_LEVEL_HIGH_DSOut_Ch4)
        {
            HAL_GPIO_WritePin(GPIOB, devOutput[4], GPIO_PIN_SET);
        }
        else if(cmd.type == SET_LEVEL_HIGH_DSOut_Ch5)
        {
            HAL_GPIO_WritePin(GPIOB, devOutput[5], GPIO_PIN_SET);
        }
        else if(cmd.type == SET_LEVEL_HIGH_DSOut_Ch6)
        {
            if(count_chanel >= 6)
            {
                HAL_GPIO_WritePin(GPIOB, devOutput[6], GPIO_PIN_SET);
            }
        }
        else if(cmd.type == SET_LEVEL_HIGH_DSOut_Ch7)
        {
            if(count_chanel == 7)
            {
                HAL_GPIO_WritePin(GPIOB, devOutput[7], GPIO_PIN_SET);
            }
        }
        
        pack->array[0] = ack;
        pack->array[1] = ((ack + 1)^0xFF);
        pack->size     = 2;
    }
}
//---------------------------------------------------
void dev_set_output(uint16_t* outputs, uint8_t count)
{
    count_chanel = count;
    
    uint16_t out = 0;
    
    for(uint8_t i = 0; i < count_chanel; ++i)
    {
        uint16_t pin = outputs[i];
        devOutput[i] = pin;
        out += pin;
    }
    
    GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIOB Port Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIOB pins */
  GPIO_InitStruct.Pin = out;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
