#include "dvc_app_gpio.h"

/* IO对象结构 */
GPIO_Object_t app_gpio;

/* 函数体 --------------------------------------------------------------------*/

void APP_GPIO_Init(void) {
    BSP_GPIO_NewObject(&app_gpio, GPIO_INPUT_DEV, APP_CONFIG_GPIO_Port, APP_CONFIG_Pin);
}