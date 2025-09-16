# [RobotMaster步兵战车框架-嵌入式STM32工程框架参考](https://github.com/why-0-why/Robot)

## 软/硬件工具

- 开发对象:C板(STM32F407)
- 软件工具:STM32CubeMX+STM32CubeCLT+CLion
  [视频教学](https://www.bilibili.com/video/BV1pnjizYEAk/?spm_id_from=333.337.search-card.all.click&vd_source=bac180abef9ba0c773fe14d848ce89e0)
- 烧录工具:CMSIS_DAP/正点原子无线仿真器[配置教学!!教学有需要修改的地方,参考文件USER/Tool/STM32F4_dap.cfg](https://www.cnblogs.com/xs314/articles/18906035/stm32_daplink_clion)

## 文件架构

- USER:所有人为编写的文件
    - TOOL:各类工具/配置文件，dap下载器配置
    - Component：组件层，底层驱动
      - Algorithm:算法层（pid算法，数学工具。。。）
      - Drive:驱动层（can驱动，uart驱动。。。）
    - Device:设备层（执行机构对象，传感器对象，内外通讯对象）
    - Module:模组层（底盘模组，云台模组，控制接口模组）
    - Task:任务层，FreeRTOS任务函数（在cubeMX中配置任务函数为虚函数）

算法层和驱动层是底层组件，设备层调用算法层和驱动层实现设备对象，模组层由设备组成，任务层操作模组对象，实现任务目标。

## C++开发注意点

### 方法一：任务层用C++，无需任何修改，可直接运行

### 方法二：任务层用C，各个设备类单独提供C接口（不推荐）
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
        User/Algorithm/alg_pid.cpp
        User/Drive/drv_uart.c
        User/Drive/drv_can.c
        User/Task/ReadTask.cpp
        User/Device/dvc_motor.cpp
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



2025/08/07 省略其他模式（键盘和遥控器保留），省略底盘的island模式，超电限制函数忽略,函数去掉关键字static,修改云台PID为普通的双环PID（结构体）