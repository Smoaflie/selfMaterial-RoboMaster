这是一个用`STM32CubeMX`生成的基于`开发板C`且支持`EIDE`的示例工程

# 开发环境

开发环境为`VSCode`搭配`EIDE`插件，故生成了相关文件

EIDE配置烧录芯片为`STM32F407IG`

如要使用其他开发环境，请自行删除以下文件

>  ┣ 📂.eide
>  ┣ 📂.vscode
>
>  ┣ 📜.clang-format
>  ┣ 📜.eide.usr.ctx.json
>  ┣ 📜.gitignore
>  ┣ 📜STM32F407IGHx.code-workspace

# 一种将用户代码放在一个文件夹内的思路

用户代码编写于`User\usr_main.c`文件中

详见该仓库下`Study\selfLibrary\readme.md`



## 已配置

- 时钟源、时钟树

  > In `Pinout & Configuration`-`System Core`-`RCC` 
  >
  > ​	配置`HSE`为`Crystal/Ceramic Resonator`
  >
  > In `Clock Configuration` 
  >
  > ​	配置`Input frequency`=`12`
  >
  > ​	配置`HCLK(MHz)`=`168`
  >
  > ​	其余由软件自行设置

- 文件输出选项

  > In `Project`
  >
  > - `IDE`选择`STM32CubeIDE`
  >
  > In `Code Generator`:
  >
  > - `STM32Cube MCU packages and embedded software packsSTM32Cube`：
  >
  >   选择 `Add necessary library files as reference in the toolchain project configuration file`	
  >
  > - `Generated files`:
  >
  >   勾选 `Generate peripheral initialization as a pair of'.c/.h' files per peripheral`

- 修改`堆/栈大小`=`0x2000`

## 以下配置内容基于官方原理图，请按需自行取消

- LED-GPIO口

  > LED_R/LED_G/LED_B

- 陀螺仪-SPI1+GPIO（CS1_Accel/CS1_Gyro）
- 外接摄像头-I2C1
- 磁力计IST8310-I2C3
- 陀螺仪中断输出-INT1_Accel/INT1_Gyro
- 蜂鸣器-TIM4_CH3

- 遥控器数据接收-USART3
- 电机控制-CAN1/CAN2



- 电池电量检测-ADC3

(其余待补充)



- 用户I2C-I2C2

- 用户SPI-SPI2+GPIO(SPI2_CS)
- 用户按键-KEY
- 用户串口-USART1