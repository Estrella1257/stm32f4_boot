#ifndef __BL_FLASH_LAYOUT_H
#define __BL_FLASH_LAYOUT_H

#define FLASH_BOOT_ADDRESS          0x08000000    // Bootloader起始地址
#define FLASH_BOOT_SIZE             48 * 1024     // Bootloader大小：48KB

#define FLASH_ARG_ADDRESS           0x0800C000    // 参数区起始地址 
#define FLASH_ARG_SIZE              16 * 1024     // 参数区大小：16KB

#define FLASH_APP_ADDRESS           0x08010000    // 应用程序起始地址
#define FLASH_APP_SIZE              256 * 1024    // 应用程序大小：256KB


#endif /* __BL_FLASH_LAYOUT_H */