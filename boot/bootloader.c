#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "main.h"
#include "crc32.h"
#include "ringbuffer8.h"
#include "flash_layout.h"
#include "arginfo.h"
#include "stm32f4xx.h"
#include "uart.h"
#include "led.h"
#include "button.h"
#include "norflash.h"


#define LOG_LVL     ELOG_LVL_INFO
#define LOG_TAG     "boot"
#include "elog.h"


#define BOOTLOADER_VERSION_MAJOR    1            //主版本
#define BOOTLOADER_VERSION_MINOR    0            //次版本  

#define BL_TIMEOUT_MS               500ul        //超时时间500毫秒
#define BL_UART_BUFFER_SIZE         512ul        //UART缓冲区大小512字节
#define BL_PACKET_HEAD_SIZE         128ul        //数据包头大小128字节
#define BL_PACKET_PAYLOAD_SIZE      4096ul       //数据包有效负载大小4096字节
#define BL_PACKET_PARAM_SIZE        (BL_PACKET_HEAD_SIZE + BL_PACKET_PAYLOAD_SIZE)          //数据包参数大小4224字节

#define BL_EVT_RX                   (1 << 0)      //接收事件
#define BL_EVT_BOOT                 (1 << 10)     //启动事件
#define BL_EVT_POWEROFF             (1 << 11)     //断电事件

/* format
 *
 * | start | opcode | length | payload | crc32 |
 * | u8    | u8     | u16    | u8 * n  | u32   |
 *
 * start: 0xAA
 */

//状态机定义
typedef enum
{
    BL_SM_IDLE,         //空闲
    BL_SM_START,        //起始
    BL_SM_OPCODE,       //操作码
    BL_SM_LENGTH,       //长度
    BL_SM_PARAM,        //参数
    BL_SM_CRC,          //CRC校验
} bl_state_machine_t;

//操作码定义(枚举)
typedef enum
{
    BL_OP_NONE = 0x00,          //无操作
    BL_OP_INQUIRY = 0x10,       //查询
    BL_OP_BOOT = 0x11,          //启动
    BL_OP_RESET = 0x1F,         //重置
    BL_OP_ERASE = 0x20,         //擦除
    BL_OP_READ,                 //读（未实现）0x21
    BL_OP_WRITE,                //写 0x22
    BL_OP_VERIFY,               //验证 0x23
    BL_OP_END,                  //结束 0x24
} bl_op_t;

//查询子码定义(枚举)
typedef enum
{
    BL_INQUIRY_VERSION,         //版本    0
    BL_INQUIRY_BLOCK_SIZE,      //块大小  1
} bl_inquiry_t;

//错误码定义(枚举)
typedef enum
{
    BL_ERR_OK,                  //操作成功 0x00
    BL_ERR_OPCODE,              //操作码错误 0x01
    BL_ERR_OVERFLOW,            //缓冲区溢出 0x02
    BL_ERR_TIMEOUT,             //操作超时 0x03
    BL_ERR_FORMAT,              //数据格式错误 0x04
    BL_ERR_VERIFY,              //校验失败 0x05
    BL_ERR_PARAM,               //参数错误  0x06
    BL_ERR_UNKNOWN = 0xff,      //未知错误
} bl_err_t;

//数据包结构体定义
typedef struct
{
    bl_op_t  opcode;            //操作码
    uint16_t length;            //参数数据长度
    uint32_t crc;               //CRC校验
    uint8_t  param[BL_PACKET_PARAM_SIZE];      //参数数据缓冲区
    uint16_t index;             //当前参数数据索引   
} bl_pkt_t;

//接收缓冲区结构体定义
typedef struct
{
    uint8_t data[16];          //接收数据缓冲区（用于状态机解析）  
    uint16_t index;            //当前接收数据索引
} bl_rx_t;

//Bootloader控制块结构体定义
typedef struct
{
    bl_pkt_t pkt;               //当前处理的数据包
    bl_rx_t rx;                 //接收缓冲区（用于协议解析）
    bl_state_machine_t sm;      //当前状态机状态
} bl_ctrl_t;

//查询命令参数结构体定义
typedef struct
{
    uint8_t subcode;             //查询子命令码（使用bl_inquiry_t枚举值）
} bl_inquiry_param_t;

//擦除命令参数结构体定义
typedef struct
{
    uint32_t address;            // 要擦除的Flash起始地址
    uint32_t size;               // 要擦除的Flash大小（字节）   
} bl_erase_param_t;

//读取命令参数结构体定义
typedef struct
{
    uint32_t address;            //读取的Flash起始地址
    uint32_t size;               //读取的Flash大小（字节）
} bl_read_param_t;


//写入命令参数结构体定义
typedef struct
{
    uint32_t address;            //写入的Flash起始地址
    uint32_t size;               //写入的Flash大小（字节）
    uint8_t data[];              //写入的数据,长度为size
} bl_write_param_t;

//验证命令参数结构体定义
typedef struct
{
    uint32_t address;            //验证的Flash起始地址
    uint32_t size;               //验证的Flash大小（字节）
    uint32_t crc;                //验证的CRC值
} bl_verify_param_t;

// 全局变量定义
static ringbuffer8_t serial_rb;                                 //串口环形缓冲区对象
static uint8_t serial_rb_buffer[BL_UART_BUFFER_SIZE];           //串口环形缓冲区内存空间
static bl_ctrl_t bl_ctrl;                                       //Bootloader全局控制块   
static uint32_t last_pkt_time;                                  //最后一个数据包的接收时间（用于超时检测）


void boot_application(void);

//串口数据接收回调函数
static void serial_recv_callback(uint8_t *data, uint32_t len)
{
    //将接收到的数据存入环形缓冲区
    rb8_puts(serial_rb, data, len);
}

//重置Bootloader控制块状态
static void bl_reset(bl_ctrl_t *ctrl)
{
    ctrl->sm = BL_SM_IDLE;           //状态机重置为空闲状态
    ctrl->rx.index = 0;              //清空接收缓冲区索引
    ctrl->pkt.index = 0;             //清空数据包参数索引 
}
     
//发送响应数据包
static void bl_response(bl_op_t op, uint8_t *data, uint16_t length)
{
    const uint8_t head = 0xAA;

    //计算CRC32校验值（按照协议格式顺序计算）
    uint32_t crc = 0;
    crc = crc32_update(crc, (uint8_t *)&head, 1);
    crc = crc32_update(crc, (uint8_t *)&op, 1);
    crc = crc32_update(crc, (uint8_t *)&length, 2);
    crc = crc32_update(crc, data, length);

    //按照协议格式发送完整数据包
    bl_uart_write((uint8_t *)&head, 1);
    bl_uart_write((uint8_t *)&op, 1);
    bl_uart_write((uint8_t *)&length, 2);
    bl_uart_write(data, length);
    bl_uart_write((uint8_t *)&crc, 4);
}

//发送简单ACK响应（只包含错误码）
static void bl_response_ack(bl_op_t op, bl_err_t err)
{
    bl_response(op, (uint8_t *)&err, 1);        //错误码作为数据发送
}

//查询命令处理函数
static void bl_op_inquiry_handler(uint8_t *data, uint16_t length)
{
    log_i("inquery");

    bl_inquiry_param_t *inquiry = (void *)data;             //转换查询参数结构

    //参数长度检查
    if (length != sizeof(bl_inquiry_param_t))
    {
        log_w("length mismatch %d != %d", length, sizeof(bl_inquiry_param_t));
        bl_response_ack(BL_OP_INQUIRY, BL_ERR_PARAM);
        return;
    }

    log_i("subcode: %02X", inquiry->subcode);
    switch (inquiry->subcode)
    {
        case BL_INQUIRY_VERSION:            //查询版本信息
        {
            uint8_t version[] = { BOOTLOADER_VERSION_MAJOR, BOOTLOADER_VERSION_MINOR };
            bl_response(BL_OP_INQUIRY, version, sizeof(version));
            break;
        }
        case BL_INQUIRY_BLOCK_SIZE:         //查询块大小
        {
            uint16_t size = BL_PACKET_PAYLOAD_SIZE;
            bl_response(BL_OP_INQUIRY, (uint8_t *)&size, sizeof(size));
            break;
        }
        default:                            //未知子码
        {
            bl_response_ack(BL_OP_INQUIRY, BL_ERR_PARAM);
            break;
        }
    }
}

//启动应用程序命令处理函数
static void bl_op_boot_handler(uint8_t *data, uint16_t length)
{
    log_i("boot");

    bl_response_ack(BL_OP_BOOT, BL_ERR_OK);       //响应成功

    boot_application();          //跳转到应用程序
}

//系统复位命令处理函数
static void bl_op_reset_handler(uint8_t *data, uint16_t length)
{
    log_i("reset");

    bl_response_ack(BL_OP_RESET, BL_ERR_OK);       

    NVIC_SystemReset();          //触发系统复位
}

//Flash擦除命令处理函数
static void bl_op_erase_handler(uint8_t *data, uint16_t length)
{
    log_i("erase");

    bl_erase_param_t *erase = (void *)data;

    if (length != sizeof(bl_erase_param_t))
    {
        log_w("length mismatch %d != %d", length, sizeof(bl_erase_param_t));
        bl_response_ack(BL_OP_ERASE, BL_ERR_PARAM);
        return;
    }

    //Bootloader区域保护检查
    if (erase->address >= FLASH_BOOT_ADDRESS &&
        erase->address < FLASH_BOOT_ADDRESS + FLASH_BOOT_SIZE)
    {
        log_w("address 0x%08X is protected", erase->address);
        bl_response_ack(BL_OP_ERASE, BL_ERR_UNKNOWN);
        return;
    }

    //执行Flash擦除操作
    log_i("erase 0x%08X, size %d", erase->address, erase->size);
    bl_norflash_unlock();
    bl_norflash_erase(erase->address, erase->size);
    bl_norflash_lock();

    bl_response_ack(BL_OP_ERASE, BL_ERR_OK);
}

//Flash读取命令处理函数（未实现）
static void bl_op_read_handler(uint8_t *data, uint16_t length)
{
    log_i("read");

    // bl_read_param_t *read = (void *)data;

    // if (length != sizeof(bl_read_param_t))
    // {
    //     bl_response_ack(BL_OP_READ, BL_ERR_PARAM);
    //     return;
    // }

    // log_i("read 0x%08X, size %d", read->address, read->size);
}

//Flash写入命令处理函数
static void bl_op_write_handler(uint8_t *data, uint16_t length)
{
    log_i("write");

    bl_write_param_t *write = (void *)data;

    if (length != sizeof(bl_write_param_t) + write->size)
    {
        log_w("length mismatch %d != %d", length, sizeof(bl_write_param_t) + write->size);
        bl_response_ack(BL_OP_WRITE, BL_ERR_PARAM);
        return;
    }

    if (write->address >= FLASH_BOOT_ADDRESS &&
        write->address < FLASH_BOOT_ADDRESS + FLASH_BOOT_SIZE)
    {
        log_w("address 0x%08X is protected", write->address);
        bl_response_ack(BL_OP_ERASE, BL_ERR_UNKNOWN);
        return;
    }

    //执行Flash写入操作
    log_i("write 0x%08X, size %d", write->address, write->size);
    bl_norflash_unlock();
    bl_norflash_write(write->address, write->data, write->size);
    bl_norflash_lock();

    bl_response_ack(BL_OP_WRITE, BL_ERR_OK);
}

//数据验证命令处理函数
static void bl_op_verify_handler(uint8_t *data, uint16_t length)
{
    log_i("verify");
    bl_verify_param_t *verify = (void *)data;

    if (length != sizeof(bl_verify_param_t))
    {
        log_w("length mismatch %d != %d", length, sizeof(bl_verify_param_t));
        bl_response_ack(BL_OP_VERIFY, BL_ERR_PARAM);
        return;
    }

    log_i("verify 0x%08X, size %d", verify->address, verify->size);

    //计算指定Flash区域的CRC32值
    uint32_t crc = crc32_update(0, (uint8_t *)verify->address, verify->size);
    log_i("crc: %08X verify: %08X", crc, verify->crc);

    //比较计算出的CRC与期望的CRC
    if (crc == verify->crc)
    {
        bl_response_ack(BL_OP_VERIFY, BL_ERR_OK);       //校验成功
    }
    else
    {
        bl_response_ack(BL_OP_VERIFY, BL_ERR_VERIFY);   //校验失败
    }
}

//数据包分发处理函数
static void bl_pkt_handler(bl_pkt_t *pkt)
{
    log_i("opcode: %02X, length: %d", pkt->opcode, pkt->length);

    //根据操作码分发到对应的处理函数
    switch (pkt->opcode)
    {
        case BL_OP_INQUIRY:
            bl_op_inquiry_handler(pkt->param, pkt->length);
            break;
        case BL_OP_BOOT:
            bl_op_boot_handler(pkt->param, pkt->length);
            break;
        case BL_OP_RESET:
            bl_op_reset_handler(pkt->param, pkt->length);
            break;
        case BL_OP_ERASE:
            bl_op_erase_handler(pkt->param, pkt->length);
            break;
        case BL_OP_READ:
            bl_op_read_handler(pkt->param, pkt->length);
            break;
        case BL_OP_WRITE:
            bl_op_write_handler(pkt->param, pkt->length);
            break;
        case BL_OP_VERIFY:
            bl_op_verify_handler(pkt->param, pkt->length);
            break;
        default:
            break;          //未知操作码，忽略
    }
}

//验证数据包CRC32校验和
static bool bl_pkt_verify(bl_pkt_t *pkt)
{
    const uint8_t head = 0xAA;

    //按照协议格式重新计算CRC32
    uint32_t crc = 0;
    crc = crc32_update(crc, (uint8_t *)&head, 1);
    crc = crc32_update(crc, (uint8_t *)&pkt->opcode, 1);
    crc = crc32_update(crc, (uint8_t *)&pkt->length, 2);
    crc = crc32_update(crc, pkt->param, pkt->length);

    return crc == pkt->crc;     //比较计算值与接收值
}

//协议解析状态机处理函数
static bool bl_recv_handler(bl_ctrl_t *ctrl, uint8_t data)
{
    bool fullpkt = false;       //完整数据包标志

    bl_rx_t *rx = &ctrl->rx;        //接收缓冲区
    bl_pkt_t *pkt = &ctrl->pkt;     //数据包缓冲区

    rx->data[rx->index++] = data;       //将当前字节存入接收缓冲区

    //根据当前状态机状态处理数据
    switch (ctrl->sm)
    {
        case BL_SM_IDLE:        //空闲状态：等待起始字节
        {
            log_d("sm idle");

            rx->index = 0;      //重置接收索引
            if (rx->data[0] == 0xAA)        //检测到起始字节
            {
                ctrl->sm = BL_SM_START;        //进入起始状态
            }
            break;
        }
        case BL_SM_START:           //起始状态：解析操作码
        {
            log_d("sm start");

            rx->index = 0;
            pkt->opcode = (bl_op_t)rx->data[0];         //提取操作码
            ctrl->sm = BL_SM_OPCODE;            //进入操作码状态
            break;
        }
        case BL_SM_OPCODE:            //操作码状态：解析数据长度
        {
            log_d("sm opcode");

            if (rx->index == 2)       //已收到2字节长度数据
            {
                rx->index = 0;
                uint16_t length = *(uint16_t *)rx->data;        //提取数据长度

                //长度合法性检查
                if (length <= BL_PACKET_PARAM_SIZE)
                {
                    pkt->length = length;
                    if (length == 0) ctrl->sm = BL_SM_CRC;          //无参数数据，直接进入CRC状态
                    else             ctrl->sm = BL_SM_PARAM;        //有参数数据，进入参数状态
                }
                else
                {
                    bl_response_ack(pkt->opcode, BL_ERR_OVERFLOW);      //长度溢出错误
                    bl_reset(ctrl);               //重置状态机
                }
            }
            break;
        }
        case BL_SM_PARAM:               //参数状态：接收参数数据
        {
            log_d("sm param");

            rx->index = 0;
            if (pkt->index < pkt->length)           //检查参数索引是否在范围内
            {
                pkt->param[pkt->index++] = rx->data[0];       //存储参数数据
                if (pkt->index == pkt->length)
                {
                    ctrl->sm = BL_SM_CRC;          //参数接收完成，进入CRC状态
                }
            }
            else
            {
                bl_response_ack(pkt->opcode, BL_ERR_OVERFLOW);      //参数索引溢出错误
                bl_reset(ctrl);
            }
            break;
        }
        case BL_SM_CRC:             //CRC状态：接收CRC校验值
        {
            log_d("sm crc");

            if (rx->index == 4)         //已收到4字节CRC数据
            {
                rx->index = 0;
                pkt->crc = *(uint32_t *)rx->data;       //提取CRC值

                //CRC校验
                if (bl_pkt_verify(pkt))
                {
                    fullpkt = true;             //标记完整数据包
                }
                else
                {
                    log_w("crc mismatch");
                    bl_response_ack(pkt->opcode, BL_ERR_VERIFY);
                    bl_reset(ctrl);
                }
            }
            break;
        }
        default:                //未知状态：重置状态机
        {
            rx->index = 0;
            bl_response_ack(BL_OP_NONE, BL_ERR_OPCODE);
            bl_reset(ctrl);
            break;
        }
    }

    return fullpkt;             //返回是否解析到完整数据包
}

//底层硬件反初始化
static void bl_lowlevel_deinit(void)
{
#if DEBUG
    elog_deinit();          //调试日志反初始化
#endif

    bl_uart_deinit();       //串口反初始化

    SysTick->CTRL = 0;      //停止系统定时器

    // 停用所有中断
    // __disable_irq();
}

#if CONFIG_BOOT_DELAY > 0
//启动延迟定时器回调函数(freeRTOS )
static void boot_tim_handler(TimerHandle_t xTimer)
{
    static uint32_t count = 0;
    uint32_t timeout = *(uint32_t *)pvTimerGetTimerID(xTimer);        //获取超时时间
    if (++count < timeout)
    {
        log_i("boot in %d seconds", timeout - count);           //打印剩余时间
        return;
    }

    xTaskNotify(bl_task_handle, BL_EVT_BOOT, eSetBits);         //超时后发送启动事件
    xTimerStop(xTimer, 0);                 //停止定时器
}
#endif

//Bootloader主函数
void bootloader_main(uint32_t boot_delay)
{
    bool main_trap = false;             //主循环陷阱标志（收到命令后不再自动启动）
    uint32_t main_enter_time = 0;       //主循环进入时间（用于启动延迟计算）

    bl_uart_recv_register(serial_recv_callback);            //初始化串口接收回调

    serial_rb = rb8_new(serial_rb_buffer, BL_UART_BUFFER_SIZE);         //初始化串口环形缓冲区

    main_enter_time = bl_now();         //记录进入时间
    while (1)
    {
        //启动延迟处理（如果设置了延迟且未收到命令）
        if (boot_delay > 0 && !main_trap)
        {
            static uint32_t last_time_passed = 0;
            uint32_t time_passwd = bl_now() - main_enter_time;      //计算经过时间

            //首次进入，打印启动倒计时
            if (last_time_passed == 0)
            {
                log_i("boot in %d seconds", boot_delay);
                last_time_passed = 1;
                time_passwd = 1;
            }
            //每秒更新一次倒计时显示
            else if (time_passwd / 1000 != last_time_passed / 1000)
            {
                log_i("boot in %d seconds", boot_delay - time_passwd / 1000);
            }

            //超时后自动启动应用程序
            if (time_passwd > boot_delay * 1000)
            {
                boot_application();
            }

            last_time_passed = time_passwd;
        }

        //按键重启
        if (bl_button_pressed())
        {
            bl_delay_ms(5);         //消抖延时
            if (bl_button_pressed())       // 确认按键按下
            {
                log_i("button pressed, reset");
                NVIC_SystemReset();
                break;
            }
        }

        //串口数据处理
        if (rb8_empty(serial_rb))       //环形缓冲区为空
        {
            if (bl_ctrl.rx.index == 0)          //当前没有正在解析的数据
            {
                last_pkt_time = bl_now();       //更新最后数据包时间
            }
            else
            {
                //检查接收超时
                if (bl_now() - last_pkt_time > BL_TIMEOUT_MS)
                {
                    log_w("recv timeout");
                    #if DEBUG
                    elog_hexdump("recv", 16, bl_ctrl.rx.data, bl_ctrl.rx.index);        //调试信息
                    #endif
                    bl_reset(&bl_ctrl);         //超时重置状态机
                }
            }
            continue;       //跳过本次循环
        }

        //从环形缓冲区读取数据并处理
        uint8_t data;
        rb8_get(serial_rb, &data);      //获取一个字节
        log_d("recv: %02X", data);      //调试信息

        //调用协议解析状态机
        if (bl_recv_handler(&bl_ctrl, data))
        {
            //完整数据包解析完成
            bl_pkt_handler(&bl_ctrl.pkt);       //处理数据包
            bl_reset(&bl_ctrl);                 //重置状态机

            main_trap = true;                //设置陷阱标志（阻止自动启动）
            last_pkt_time = bl_now();        //更新最后数据包时间

        #if CONFIG_BOOT_DELAY > 0
            //停止启动延迟定时器（如果使用FreeRTOS）
            if (boot_tim_handle)
            {
                log_i("command received, stop boot");
                xTimerStop(boot_tim_handle, portMAX_DELAY);
                boot_tim_handle = NULL;
            }
        #endif
        }
    }
}

//验证应用程序完整性
bool verify_application(void)
{
    uint32_t size, crc;
    bool result = bl_arginfo_read(&size, &crc);         //读取应用程序信息
    CHECK_RETX(result, false);              //检查返回值

    uint32_t address = FLASH_APP_ADDRESS;

    //计算应用程序区域的CRC32
    uint32_t ccrc = crc32_update(0, (uint8_t *)address, size);
    if (ccrc != crc)        //CRC校验
    {
        log_w("crc mismatch: %08X != %08X", ccrc, crc);
        return false;
    }

    return true;            //验证通过
}

//跳转到应用程序执行
void boot_application(void)
{
    //定义应用程序入口函数类型
    typedef int (*entry_t)(void);

    uint32_t address = FLASH_APP_ADDRESS;
    //从应用程序向量表读取栈指针和程序计数器
    uint32_t _sp = *(volatile uint32_t*)(address + 0);      //栈指针
    uint32_t _pc = *(volatile uint32_t*)(address + 4);      //程序计数器

    (void)_sp;          //避免未使用警告
    entry_t app_entry = (entry_t)_pc;          //转换为函数指针

    log_i("booting application at 0x%08X", address);

    bl_lowlevel_deinit();           //反初始化Bootloader硬件

    // 设置主栈指针和向量表偏移（具体实现取决于MCU）
    // __set_MSP(_sp);
    // SCB->VTOR = address;

    app_entry();      //跳转到应用程序
}
