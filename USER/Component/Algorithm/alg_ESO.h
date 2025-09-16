//
// Created by Administrator on 25-8-7.
//

#ifndef ALG_ESO_H
#define ALG_ESO_H
#include <stdint.h>

/* 包含头文件 ----------------------------------------------------------------*/
/* 类型定义 ------------------------------------------------------------------*/
typedef struct
{
    float h;
    float K;
    float Km;
    float Ke;
    float J;
    float R;
    float b;
    float w;
    float y;
    float last_y;
    float delta_v;
}ESO_t;

/* 宏定义 --------------------------------------------------------------------*/
#define ABS(x) (((x) > 0) ? (x) : (-(x)))
/* 扩展变量 ------------------------------------------------------------------*/
extern ESO_t ESO;
extern float e,z1,z2,z3,last_z1,last_z2,last_z3,u;
extern float deta1,deta2,deta3;
/* 函数声明 ------------------------------------------------------------------*/
void ESO_init(void);
int32_t ESO_calc(void *argc);
#endif //ALG_ESO_H
