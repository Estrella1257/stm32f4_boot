#include <stdlib.h>
#include <string.h>
#include "ringbuffer8.h"


#define rbb_len         rb->length
#define rbb_idx(x)      (uint8_t *)rbb_buff + rbb_size * (x)
#define dat_idx(d, x)   (uint8_t *)(d) + rbb_size * (x)


struct ringbuffer8
{
    uint32_t tail;     //指向下一个读取位置
    uint32_t head;     //指向下一个写入位置
    uint32_t length;   // 缓冲区容量
    uint8_t buffer[];  //实际数据存储区
};
//柔性数组：C99特性，允许结构体包含可变长度的数组成员

//初始化一个环形缓冲区(内存复用)
//它接收一个已经分配好的内存块(buff)和该内存块的长度(length),然后将其初始化为一个环形缓冲区结构
ringbuffer8_t rb8_new(uint8_t *buff, uint32_t length)
{
    ringbuffer8_t rb = (ringbuffer8_t)buff;
    rb->length = length - sizeof(struct ringbuffer8);

    return rb;
}

//下一个写入位置计算
static inline uint16_t next_head(ringbuffer8_t rb)
{
    return rb->head + 1 < rbb_len ? rb->head + 1 : 0;            //当到达末尾时回到起点，形成环形
}

//下一个读取位置计算
static inline uint16_t next_tail(ringbuffer8_t rb)
{
    return rb->tail + 1 < rbb_len ? rb->tail + 1 : 0;
}

//空判断
bool rb8_empty(ringbuffer8_t rb)
{
    return rb->head == rb->tail;
}

//满判断
bool rb8_full(ringbuffer8_t rb)
{
    return next_head(rb) == rb->tail;        //采用"空一格"的策略来判断满状态，避免head和tail相等时的歧义
}

//写入数据：
bool rb8_put(ringbuffer8_t rb, uint8_t data)
{
    if (next_head(rb) == rb->tail)         // 检查是否满
        return false;

    rb->buffer[rb->head] = data;           // 写入数据
    rb->head = next_head(rb);              // 移动头指针

    return true;
}

//批量写入
bool rb8_puts(ringbuffer8_t rb, uint8_t *data, uint32_t size)
{
    bool ret = true;

    for (uint16_t i = 0; i < size && ret; i++)
    {
        ret = rb8_put(rb, data[i]);
    }

    return ret;
}


//读取数据：
bool rb8_get(ringbuffer8_t rb, uint8_t *data)
{
    if (rb->head == rb->tail)             // 检查是否空
        return false;

    *data = rb->buffer[rb->tail];         // 读取数据
    rb->tail = next_tail(rb);             // 移动尾指针

    return true;
}

//批量读取
bool rb8_gets(ringbuffer8_t rb, uint8_t *data, uint32_t size)
{
    bool ret = true;

    for (uint16_t i = 0; i < size && ret; i++)
    {
        ret = rb8_get(rb, &data[i]);
    }

    return ret;
}
