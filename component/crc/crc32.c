#include "crc32.h"

/*
@查表法实现CRC32,用空间换时间
@非标准CRC32
*/

#define CRC32_TABLE_SIZE    0x100  //CRC32表的大小为256

static uint8_t inited = 0;         //用于标记CRC32表是否已经初始化
static uint32_t crc32_table[CRC32_TABLE_SIZE];      //用于存储CRC32表

/*
@计算单个字节的CRC32值
@param r: 输入字节值
@return: 计算后的CRC32值，已进行0xFF000000异或调整
*/
static uint32_t crc32_for_byte(uint32_t r)
{
    for (uint32_t i = 0; i < 8; i++)
    {
        r = (r & 1 ? 0 : (uint32_t)0xedb88320) ^ r >> 1;        
    }

    return r ^ (uint32_t)0xff000000;
}

//与以上写法等价
// static uint32_t crc32_for_byte(uint32_t r)
// {
//     for (int j = 0; j < 8; j++) {
//         if (r & 1) {
//             r = (r >> 1) ^ 0xEDB88320;  // 反转的生成多项式
//         } else {
//             r = r >> 1;
//         }
//     }
//     return r ^ 0xFF000000;  // 反射调整
// }

/*
@初始化CRC32查找表
@预先计算0-255所有字节的CRC值，存入表中
*/
void crc32_init(void)
{
    for (uint32_t i = 0; i < CRC32_TABLE_SIZE; ++i)
    {
        crc32_table[i] = crc32_for_byte(i);
    }

    inited = 1;   //设置初始化标志防止重复计算
}

/*
@更新CRC32校验值
@param crc: 当前的CRC32值
@param data: 输入数据指针
@param len: 输入数据长度
@return: 更新后的CRC32值
*/
uint32_t crc32_update(uint32_t crc, uint8_t *data, uint32_t len)
{
    if (!inited)
    {
        crc32_init();
    }

    for (uint32_t i = 0; i < len; i++)
    {
        crc = crc32_table[(uint8_t)crc ^ ((uint8_t *)data)[i]] ^ crc >> 8;
    }

    return crc;
}

// 这一步是CRC32查表法的核心步骤。分解一下：
// (uint8_t)crc：取当前crc值的低8位（即最低的一个字节）
// ((uint8_t *)data)[i]：取数据中当前的一个字节
// (uint8_t)crc ^ ((uint8_t *)data)[i]：将crc的低8位与数据字节进行异或，得到一个8位的值（0-255）作为查表的索引
// crc32_table[索引]：通过索引从预计算的CRC32表中取出对应的32位值
// crc >> 8：将当前crc值右移8位（相当于去掉最低的那个字节）
// 将查表得到的32位值与右移8位后的crc进行异或，得到新的crc值
// 这个过程相当于将当前crc的低8位与数据字节进行异或，然后通过查表得到一个新的值，再与crc右移8位后的值进行异或，从而更新crc。
// 这是标准CRC32查表法的实现步骤。但是，需要注意的是，标准CRC32有多种变体，它们在初始值、最终异或值、输入输出是否反转等方面有所不同。