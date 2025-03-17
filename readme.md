

# 简介
   本程序为单片机控制灵足时代电机案例代码，使用C语言在STM32CubeIDE编写,在STM32F446运行，通过2路CAN总线，每条总线挂载3个灵足RS04电机和一个灵足RS02电机，程序可以以500hz的频率控制共计8个灵足RS04电机。


# 前期准备
1. 本程序需要准备STM32F446单片机,其他单片机型号可在本程序基础上进行修改,本程序使用的是自己画的pcb，原理图位于:```PCB```文件夹，包含两路CAN接口

2. 本程序使用STM32CubeIDE开发，需要安装STM32CubeIDE，安装地址如下：
https://www.st.com.cn/zh/development-tools/stm32cubeide.html

3. ```灵足时代电机购买地址：```
https://e.tb.cn/h.TAnAHUN38QORoTB?tk=zZ92eKjIaTxHU591



# 安装
1. 克隆仓库到本地 :
```bash
git clone https://github.com/SOULDE-Studio/USB2CAN-Demo-Lingzu.git
```
2. 进入项目目录 :
```bash
cd USB2CAN-Demo-Lingzu
```
3. 编译项目 :
```bash
mkdir build
cd build
cmake ..
make
```
4. 运行项目 :
```bash
./can_code
```


# 注意事项
1. 本程序使用的两个USB2CAN模块其设备名称分别为USB2CAN0、USB2CAN1
2. 若还需要拓展多个USB2CAN模块，可在本程序基础上进行修改，一个电脑最多拓展4个模块即8路CAN总线。
3. 在同一模块的同一条CAN总线发送的控制命令间隔不应小于300us，可以交错发送不同CAN总线上的控制命令
4. 程序封装了电机数据结构体，只需要对结构体对象赋值再调用发送函数，即可控制电机，赋值数值范围请参考灵足电机说明书
5. 本程序使用灵足RS04电机，如使用其他型号电机请修改头文件参数。
6. 本程序主要控制函数为`CAN_TX_test_thread`
7. 本模块CAN接口与灵足时代电机CAN接口线序相反，请使用反序CAN线相连。

# 拓展
> 若想要以本项目开发四足机器人，可配合强化学习训练代码、强化学习部署代码使用，相关开源库地址如下：
> 
> ```isaaclab训练代码地址： ```https://github.com/fan-ziqi/robot_lab
> 
>  ```仿真、实物部署代码地址： ```https://github.com/fan-ziqi/rl_sar
> 
>  ```相关机器人展示视频链接：``` https://www.bilibili.com/video/BV17oPEeHEfM/?share_source=copy_web&vd_source=97170e52311d304767c925aed213e556
> 



# 引用说明

Please cite the following if you use this code or parts of it:

```
@software{tangair2025USB2CAN-Demo-Lingzu,
  author = {tangair},
  title = {{USB2CAN-Demo-Lingzu: An  project based on USB2CAN and Lingzu motor.}},
  url = {https://github.com/SOULDE-Studio/USB2CAN-Demo-Lingzu.git},
  year = {2025}
}
```


