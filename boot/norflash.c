#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "norflash.h"

#define LOG_TAG "flash"
#define LOG_LVL ELOG_LVL_INFO
#include "elog.h"

#define FLASH_BASE_ADDR 0x08000000

typedef struct
{
    uint32_t sector;
    uint32_t size;
} sector_desc_t;

static const sector_desc_t sector_descs[] =
{
    {FLASH_Sector_0, 16 * 1024},
    {FLASH_Sector_1, 16 * 1024},
    {FLASH_Sector_2, 16 * 1024},
    {FLASH_Sector_3, 16 * 1024},
    {FLASH_Sector_4, 64 * 1024},
    {FLASH_Sector_5, 128 * 1024},
    {FLASH_Sector_6, 128 * 1024},
    {FLASH_Sector_7, 128 * 1024},
    {FLASH_Sector_8, 128 * 1024},
    {FLASH_Sector_9, 128 * 1024},
    {FLASH_Sector_10, 128 * 1024},
    {FLASH_Sector_11, 128 * 1024},
};

void bl_norflash_unlock(void)
{
    log_i("norflash unlock");

    FLASH_Unlock();      // 解除Flash写保护

    // FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
    //                 FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);        //清除状态标志
}

void bl_norflash_lock(void)
{
    FLASH_Lock();        // 重新锁定Flash
}

void bl_norflash_erase(uint32_t address, uint32_t size)
{
    uint32_t addr = FLASH_BASE_ADDR;

    // 遍历所有扇区
    for (uint32_t i = 0; i < sizeof(sector_descs) / sizeof(sector_desc_t); i++)         //sizeof(sector_descs) / sizeof(sector_desc_t)扇区数
    {
        // 检查当前扇区是否在要擦除的范围内
        if (addr >= address && addr < address + size)
        {
            log_i("erase sector %u, addr: 0x%08x, size: %u", i, addr, sector_descs[i].size);

             // 执行扇区擦除
            if (FLASH_EraseSector(sector_descs[i].sector, VoltageRange_3) != FLASH_COMPLETE)        //VoltageRange_3：当设备电压范围在2.7V至3.6V之间时使用
            {
                log_w("erase sector %u failed", i);
            }
        }

        // 移动到下一个扇区起始地址
        addr += sector_descs[i].size;
    }
}

void bl_norflash_write(uint32_t address, uint8_t *data, uint32_t size)
{
    // 循环写入，每次递增4字节（32位）
    for (uint32_t i = 0; i < size; i += 4)
    {
        // 以32位(字)为单位写入
        if (FLASH_ProgramWord(address + i, *(uint32_t *)(data + i)) != FLASH_COMPLETE)
        {
            log_w("write data failed, addr: 0x%08x", address + i);
        }
    }
}

