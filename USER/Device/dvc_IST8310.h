#ifndef DVC_IST8310D_H
#define DVC_IST8310D_H

#include "drv_timer.h"

#define IST8310_DATA_READY_BIT 2

#define IST8310_NO_ERROR 0x00

#define IST8310_NO_SENSOR 0x40

#define IST8310_IIC_ADDRESS (0x0E)  //IST8310的IIC地址
#define IST8310_IIC_READ_MSB (0x80) //IST8310的SPI读取发送第一个bit为1

extern void IST8310_GPIO_init(void); //ist8310的io初始化
extern void IST310_com_init(void);  //ist8310的通讯初始化
extern uint8_t IST8310_IIC_read_single_reg(uint8_t reg);
extern void IST8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
extern void IST8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void IST8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
extern void IST8310_delay_ms(uint16_t ms);
extern void IST8310_delay_us(uint16_t us);
extern void IST8310_RST_H(void); //复位IO 置高
extern void IST8310_RST_L(void); //复位IO 置地 置地会引起IST8310重启

typedef struct ist8310_real_data_t
{
    uint8_t status;
    float mag[3];
} ist8310_real_data_t;

extern uint8_t IST8310_init(void);
extern void IST8310_read_over(uint8_t *status_buf, ist8310_real_data_t *mpu6500_real_data);
extern void IST8310_read_mag(float mag[3]);
#endif
