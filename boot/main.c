#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "uart.h"
#include "button.h"
#include "led.h"

#define LOG_TAG "main"
#define LOG_LVL ELOG_LVL_INFO
#include "elog.h"


extern void bl_lowlevel_init(void);
extern void bootloader_main(uint32_t boot_delay);
extern bool verify_application(void);

//第一阶段: 检测按钮是否按下以进入bootloader模式
static bool button_trap_boot(void)
{
    if (bl_button_pressed())
    {
        bl_delay_ms(100);
        return bl_button_pressed();
    }

    return false;
}

//等待按钮释放
static void button_wait_release(void)
{
    while (bl_button_pressed())
    {
        bl_delay_ms(100);
    }
}

int main(void)
{
    bl_lowlevel_init();

#if DEBUG
    elog_init();
    //设置各日志级别的输出格式
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_TAG);
    elog_start();                                 //启动日志系统
#endif

    //第二阶段: 外设初始化
    bl_delay_init();
    bl_uart_init();
    bl_button_init();
    bl_led_init();

    log_d("button: %d", bl_button_pressed());

    //第三阶段: 启动模式判断
    bool trap_boot = false;

    //条件1：检测按钮是否按下
    if (button_trap_boot())
    {
        log_w("button pressed, trap into boot");
        trap_boot = true;
    }

    // 条件2：验证应用程序是否有效
    else if (!verify_application())
    {
        log_w("application verify failed, trap into boot");
        trap_boot = true;
    }

    //第四阶段：模式处理
    if (trap_boot)
    {
        bl_led_on();
        button_wait_release();
    }

    //第五阶段：启动引导加载器
    //如果trap_boot为true，延迟0秒立即进入bootloader
    //如果trap_boot为false，延迟3秒后进入应用程序
    bootloader_main(trap_boot ? 0 : 3);

    return 0;
}
