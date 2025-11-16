#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx.h"

void bl_button_init(void)
{
    // PE0
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStruct);
}

bool bl_button_pressed(void)
{
    return GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0) == Bit_RESET;
}
