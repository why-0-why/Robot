#ifndef DVC_LED_H
#define DVC_LED_H

#include "drv_gpio.h"

/* 宏定义 */

#define BLACK       (0xFF000000)    // 黑色
#define DARKGRAY    (0xFF555555)    // 深灰
#define LIGHTGRAY   (0xFFAAAAAA)    // 浅灰
#define WHITE       (0xFFFFFFFF)    // 白色
#define GRAY        (0xFF7F7F7F)    // 灰色
#define RED         (0xFFFF0000)    // 红色
#define GREEN       (0xFF00FF00)    // 绿色
#define BLUE        (0xFF0000FF)    // 蓝色
#define CYAN        (0xFF00FFFF)    // 青色
#define YELLOW      (0xFFFFFF00)    // 黄色
#define MAGENTA     (0xFFFF00FF)    // 洋红
#define ORANGE      (0xFFFF7F00)    // 橙色
#define PURPLE      (0xFF7F007F)    // 紫色
#define BROWN       (0xFF996633)    // 棕色
#define CLEAR       (0x00000000)    // 透明

/* IO对象结构 */

extern GPIO_Object_t led_r_gpio;
extern GPIO_Object_t led_g_gpio;
extern GPIO_Object_t led_b_gpio;

/* 函数声明 */

void LED_Init(void);
void LED_show(uint32_t aRGB);

#endif //DVC_LED_H
