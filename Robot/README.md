 # [RobotMaser步兵战车-嵌入式STM32工程框架参考](https://github.com/why-0-why/ArmController)

## 软/硬件工具

- 开发对象:2块C板(STM32F407)
- 软件工具:STM32CubeMX+STM32CubeCLT+CLion
[视频教学](https://www.bilibili.com/video/BV1pnjizYEAk/?spm_id_from=333.337.search-card.all.click&vd_source=bac180abef9ba0c773fe14d848ce89e0)
- 烧录工具:CMSIS_DAP/正点原子无线仿真器[配置教学!!教学有需要修改的地方,参考文件User/cfg/dap.cfg](https://www.cnblogs.com/xs314/articles/18906035/stm32_daplink_clion)

## 文件架构

- User:所有人为编写的文件
  - cfg:config各类配置文件，dap下载器配置
  - Algorithm:算法层，pid算法，数学工具
  - Drive:驱动层，can驱动，uart驱动
  - Device:设备层，电机对象
  - Task:任务层，FreeRTOS任务函数（在cubeMX中配置任务函数为虚函数）

算法层和驱动层是底层组件，设备层调用算法层和驱动层实现设备对象，任务层操作设备对象，实现任务目标。

## C++开发框架（以Class_PID为例）

文件架构如下

- alg_pid.h:头文件，包含C的接口声明和类的声明
- alg_pid.cpp:类的定义，遵循c++语法
- alg_pid_capi.cpp:实现C的接口，遵循c++语法

代码注意点

头文件中预处理格式如下

```c++
#ifdef __cplusplus
extern "C" {
#endif

//C的接口声明（一般包含：前向声明，创建/销毁/初始化对象 ，类内类指针，类函数）
//遵循C语言语法

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

//类的声明
//遵循c++语法

#endif
```

## CMakeLists修改

把用到的源文件放入target_sources()，把头文件路径放入target_include_directories()。

```cmake
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
        User/Drive/drv_math.cpp
        User/Algorithm/alg_pid.cpp
        User/Algorithm/alg_pid_capi.cpp
        User/Drive/drv_uart.c
        User/Drive/drv_can.c
        User/Task/ReadTask.c
        User/Device/dvc_motor.cpp
        User/Device/dvc_motor_capi.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
        User/Algorithm
        User/Device
        User/Drive
        User/Task
)
```

# 未知TODO

7处，why存文件传输助手