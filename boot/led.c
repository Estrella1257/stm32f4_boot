#include <stdbool.h>
#include "stm32f4xx.h"
#include "led.h"

static bool led_state;

void bl_led_init(void)
{
    // PE5
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStruct);

    bl_led_off();
}

void bl_led_set(bool on)
{
    led_state = on;
    GPIO_WriteBit(GPIOE, GPIO_Pin_5, on ? Bit_RESET : Bit_SET);
}

void bl_led_on(void)
{
    led_state = true;
    bl_led_set(true);
}

void bl_led_off(void)
{
    led_state = false;
    bl_led_set(false);

}

void bl_led_toggle(void)
{
    led_state = !led_state;
    bl_led_set(led_state);
}
