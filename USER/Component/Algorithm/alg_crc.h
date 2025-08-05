#ifndef ALG_CRC_H
#define ALG_CRC_H
/* 包含头文件 ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>

/* 类型定义 ------------------------------------------------------------------*/

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);

void append_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
void append_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);
#endif //ALG_CRC_H
