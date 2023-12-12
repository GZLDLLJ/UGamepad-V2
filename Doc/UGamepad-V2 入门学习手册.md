

# 序

欢迎踏入LDSCITECH的评估板世界，一个充满激情、创新和未知可能性的领域！评估板是连接你和技术未来之间的桥梁，它们是探索新概念、验证创意和打造创新产品的关键。在这个令人兴奋的旅程中，LDSCITECH将与您一同探索微控制器、传感器和电机评估板的神奇世界。

评估板是通往数字创新的大门，为您提供实验和测试的平台，让您在不牺牲复杂性的情况下深入了解各种技术。无论您是初学者还是经验丰富的工程师，评估板都是您在设计过程中的得力助手，帮助您迅速验证概念、加速产品开发，最终将您的想法变为现实。

在LDSCITECH，我们深知评估板的关键作用，因此致力于为您提供最先进、高性能的微控制器、先进的传感器技术以及创新的电机评估板。通过这些工具，您将能够突破技术界限，挑战创新高峰，为您的项目赋能，引领行业变革。

我们相信，每一个从事技术创新的人都值得有一个让创意飞翔的平台。LDSCITECH将与您携手共进，提供全方位的支持，帮助您在评估板的世界里找到您独特的路径。

准备好迎接新的技术挑战吗？让我们一同踏上这段激动人心的评估板之旅吧！

非常感谢您的理解和支持！技术领域的学习和分享是一个不断演进的过程，每个人都在不断提高自己的水平。善意的反馈和建议是进步的动力，同时也为作者提供了改进的机会。

如果您在文档或视频中发现任何不准确或不清楚的地方，或者有任何改进的建议，我们真诚地欢迎您提出。您的意见将有助于提高质量，使信息更加准确和易于理解，使得文档和视频更好地满足读者和观众的需求。

LDSCITECH始终秉承着开放、包容和学习的精神，我们期待与您共同进步，共同创造更优质的技术资源。再次感谢您的关注和支持！



| **实例**             | **描述**                             |
| -------------------- | ------------------------------------ |
| Eg1_Joystick         | 实现一个Joystick摇杆设备             |
| Eg2_WS2812B          | 点亮WS2812B灯珠并实现七彩渐变        |
| Eg3_MultiTimer       | 移植MultiTimer软件定时器模块         |
| Eg4_Mouse            | 实现模拟鼠标功能                     |
| Eg5_KeyBoard         | 实现模拟键盘功能                     |
| Eg6_DoubleJoystick   | 实现一个USB双摇杆                    |
| Eg7_CompositeGMK     | 实现Joystick、MOUSE、Keyboard的组合  |
| Eg8_Gamepad          | 实现游戏手柄Gamepad的功能            |
| Eg9_AbsoluteMouse    | 实现绝对值鼠标的功能                 |
| Eg10_Xinput          | 实现Xbox手柄功能，Xinput（出厂默认） |
| Eg11_Xinput01        | 外接摇杆电位器实现Xbox手柄功能       |
| Eg12_MultiAxisButton | 实现8轴32键摇杆                      |

# 第一部分、硬件概述

## 1.1 3D图

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/1f243e531c824c7c92ab98acf9c48af7.png#pic_center)



如图1.1所示Gamepad评估板配置了8个6*6轻触按键，一个摇杆（Joystick），搭载一颗WS2812B灯珠，并将UART1串口，编程接口（SWD），外接Joystick接口，Type-C接口引出;  

## 1.2 UGamepad-V2原理图

UGamepad-V2原理图如图1.2所示，如看不清可打开Doc目录下的PDF文档查阅  
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/6851040ccf744df29f2166ef25d3159b.png#pic_center)


# 第二部分、软件工具

## 2.1 软件概述

   在 /Software 目录下是常用的工具软件：
1. Dt2_4：配置USB设备Report描述符的工具；

2. USBHID调试助手/呀呀USB： USB调试工具，相当于串口调试助手功能；

3. BUSHound：总线调试工具；

4. USBlyzer：一款专业的USB协议分析软件

5. MounRiver: 编译器；

6. 在线测试工具：

   https://devicetests.com/  

   https://key-test.com/cn/

   https://www.sqlsec.com/tools/mouse.html

   https://www.onlinemictest.com/zh/mouse-test/

   

## 2.2 MounRiver软件入门

MounRiver Studio基于Eclipse GNU版本开发，在保留原平台强大代码编辑功能、便捷组件框架的同时，针对嵌入式C /C++开发，进行了一系列界面、功能、操作方面的修改与优化，以及工具链的指令增添、定制工作。力求打造一款硬件工程师喜爱的、以RISC-V内核为主的嵌入式集成开发环境。大家访问以下链接获取下载：http://mounriver.com/help

# 第三部分、实战训练

## 3.1 实例Eg01_ButtonDebug

本节作为第一个实例，目的是两个：一是为了测试按键的可用性；二是引入组件MultiButton；向大家展示开源软件的使用，对于我们做应用的工程师来说，开源套件的使用，往往能达到事半功倍的效果；

### 3.1.1硬件设计

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/a7001e3beaed438d85322406e46101d2.png#pic_center)

如上图是 11个6x6的独立按键； 

下图是五向开关，支持上下左右中五个方向；

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/60a99857890749448f8080b3c5e92324.png#pic_center)

另外有2个按键是摇杆电位器上的中键，高电平有效

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/b4d84c401bc745cfa4bad9338f81b6e2.png#pic_center)


所以，我们只要配置18个GPIO作为输入去检测按键信号;  

### 3.1.2 软件设计

#### 3.1.2.1 工程树

首先是工程树，我们打开工程，可以看到Project Explorer下Gamepad目录如下图

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/7a4f032911974a46981e92219484c6de.png#pic_center)

其中

> - __Binaries：__ 二进制文件；
> - __Includes：__ 包含的头文件；
> - __Core：__内核文件，存放core_riscv内核文件；
> - __Debug：__ 存放串口打印和延迟函数相关的文件
> - **Ld：**链接文件，链接文件通常是指用来生成可执行程序的各种源代码文件和库文件。
> - **Midleware：**用于存放开源软件；如MultiButton、MultiTimer
> - __myBSP：__ 自主编写的外设驱动文件；
> - __obj：__ 编译的生成的obj文件；	
> - __Peripheral：__ 这是MCU厂商提供外设相关驱动；
> - __Startup：__ ch32v203的启动文件；
> - __User: __ch32v203的系统配置文件，中断相关文件，main函数等；

工程目录这里只做一次介绍，后面的样例目录大同小异。

#### 3.1.2.2 系统时钟

我们先打开startup_ch32v10x.S启动文件，我们看到如下代码

```C
  jal  SystemInit
	la t0, main
```

定位到SystemInit

```c
void SystemInit (void)
{
  RCC->CTLR |= (uint32_t)0x00000001;
  RCC->CFGR0 &= (uint32_t)0xF8FF0000;
  RCC->CTLR &= (uint32_t)0xFEF6FFFF;
  RCC->CTLR &= (uint32_t)0xFFFBFFFF;
  RCC->CFGR0 &= (uint32_t)0xFF80FFFF;
  RCC->INTR = 0x009F0000;    
  SetSysClock();
}
```

关于RCC寄存器的配置，请各位自行查阅用户手册；我们接着打开SetSysClock函数

```C
static void SetSysClock(void)
{
#ifdef SYSCLK_FREQ_HSE
    SetSysClockToHSE();
#elif defined SYSCLK_FREQ_48MHz_HSE
    SetSysClockTo48_HSE();
#elif defined SYSCLK_FREQ_56MHz_HSE
    SetSysClockTo56_HSE();
#elif defined SYSCLK_FREQ_72MHz_HSE
    SetSysClockTo72_HSE();
#elif defined SYSCLK_FREQ_96MHz_HSE
    SetSysClockTo96_HSE();
#elif defined SYSCLK_FREQ_120MHz_HSE
    SetSysClockTo120_HSE();
#elif defined SYSCLK_FREQ_144MHz_HSE
    SetSysClockTo144_HSE();
#elif defined SYSCLK_FREQ_48MHz_HSI
    SetSysClockTo48_HSI();
#elif defined SYSCLK_FREQ_56MHz_HSI
    SetSysClockTo56_HSI();
#elif defined SYSCLK_FREQ_72MHz_HSI
    SetSysClockTo72_HSI();
#elif defined SYSCLK_FREQ_96MHz_HSI
    SetSysClockTo96_HSI();
#elif defined SYSCLK_FREQ_120MHz_HSI
    SetSysClockTo120_HSI();
#elif defined SYSCLK_FREQ_144MHz_HSI
    SetSysClockTo144_HSI();

#endif
 
 /* If none of the define above is enabled, the HSI is used as System clock
  * source (default after reset) 
	*/ 
}
```

这里我们使用96Mhz内部高速时钟HSI：SYSCLK_FREQ_96MHz_HSI，因为USB外设需要48Mhz，所以为了分频方便，需要选择48的整数倍；

#### 3.1.2.3 用户代码

##### 3.1.2.3.1 Button部分

Button模块主要是独立按键扫描，这里使用了MultiButton事件回调机制；关于MultiButton的使用，大家在GitHub上找到：

https://github.com/0x1abin/MultiButton

同时附带使用方法，这里就不赘述了。

```c
#include "Button.h"


struct Button btn1, btn2, btn3, btn4, btn5, btn6, btn7, btn8, btn9, btn10,
        btn11, btn12, btn13,btn14,btn15,btn16,btn17,btn18;

u8 read_button_GPIO(u8 button_id) {
    // you can share the GPIO read function with multiple Buttons
    switch (button_id) {
    case btn1_id:
        return GPIO_ReadInputDataBit(LSW_PORT, LSW_PIN);
    case btn2_id:
        return GPIO_ReadInputDataBit(RSW_PORT, RSW_PIN);
    case btn3_id:
        return GPIO_ReadInputDataBit(LB_PORT, LB_PIN);
    case btn4_id:
        return GPIO_ReadInputDataBit(LT_PORT, LT_PIN);
    case btn5_id:
        return GPIO_ReadInputDataBit(RB_PORT, RB_PIN);
    case btn6_id:
        return GPIO_ReadInputDataBit(RT_PORT, RT_PIN);
    case btn7_id:
        return GPIO_ReadInputDataBit(BACK_PORT, BACK_PIN);
    case btn8_id:
        return GPIO_ReadInputDataBit(HOME_PORT, HOME_PIN);
    case btn9_id:
        return GPIO_ReadInputDataBit(START_PORT, START_PIN);
    case btn10_id:
        return GPIO_ReadInputDataBit(A_BT_PORT, A_BT_PIN);
    case btn11_id:
        return GPIO_ReadInputDataBit(B_BT_PORT, B_BT_PIN);
    case btn12_id:
        return GPIO_ReadInputDataBit(X_BT_PORT, X_BT_PIN);
    case btn13_id:
        return GPIO_ReadInputDataBit(Y_BT_PORT, Y_BT_PIN);
    case btn14_id:
        return GPIO_ReadInputDataBit(GTA_PORT, GTA_PIN);
    case btn15_id:
        return GPIO_ReadInputDataBit(GTB_PORT, GTB_PIN);
    case btn16_id:
        return GPIO_ReadInputDataBit(GTC_PORT, GTC_PIN);
    case btn17_id:
        return GPIO_ReadInputDataBit(GTD_PORT, GTD_PIN);
    case btn18_id:
        return GPIO_ReadInputDataBit(GTE_PORT, GTE_PIN);
    default:
        return 0;
    }
}

void LSW_SINGLE_Click_Handler(void* btn) {
    printf("LSW_SINGLE_Click_Handler\r\n");
}

void RSW_SINGLE_Click_Handler(void* btn) {
    printf("RSW_SINGLE_Click_Handler\r\n");

}
void LB_SINGLE_Click_Handler(void* btn) {
    printf("LB_SINGLE_Click_Handler\r\n");
}

void LT_SINGLE_Click_Handler(void* btn) {
    printf("LT_SINGLE_Click_Handler\r\n");

}

void RB_SINGLE_Click_Handler(void* btn) {
    printf("RB_SINGLE_Click_Handler\r\n");
}

void RT_SINGLE_Click_Handler(void* btn) {
    printf("RT_SINGLE_Click_Handler\r\n");

}
void BACK_SINGLE_Click_Handler(void* btn) {
    printf("BACK_SINGLE_Click_Handler\r\n");
}

void HOME_SINGLE_Click_Handler(void* btn) {
    printf("HOME_SINGLE_Click_Handler\r\n");
}
void START_SINGLE_Click_Handler(void* btn) {
    printf("START_SINGLE_Click_Handler\r\n");
}
void A_SINGLE_Click_Handler(void* btn) {
    printf("A_SINGLE_Click_Handler\r\n");
}
void B_SINGLE_Click_Handler(void* btn) {
    printf("B_SINGLE_Click_Handler\r\n");
}
void X_SINGLE_Click_Handler(void* btn) {
    printf("X_SINGLE_Click_Handler\r\n");
}
void Y_SINGLE_Click_Handler(void* btn) {
    printf("Y_SINGLE_Click_Handler\r\n");
}
void GTA_SINGLE_Click_Handler(void* btn) {
    printf("GTA_SINGLE_Click_Handler\r\n");
}
void GTB_SINGLE_Click_Handler(void* btn) {
    printf("GTB_SINGLE_Click_Handler\r\n");
}
void GTC_SINGLE_Click_Handler(void* btn) {
    printf("GTC_SINGLE_Click_Handler\r\n");
}
void GTD_SINGLE_Click_Handler(void* btn) {
    printf("GTD_SINGLE_Click_Handler\r\n");
}
void GTE_SINGLE_Click_Handler(void* btn) {
    printf("GTE_SINGLE_Click_Handler\r\n");
}

void ButtonInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(
            RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD |
            RCC_APB2Periph_AFIO, ENABLE); // 使能GPIOA时钟
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
    // 配置GPIOA的Pin 0/3为输入下拉模式（IPD）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置GPIOA的Pin 6/7/8/15为输入下拉模式（IPU）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8
            | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // 配置GPIOB的Pin 6/7/8/15为输入下拉模式（IPU）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2
            | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_13
            | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 配置GPIOB的Pin 6/7/8/15为输入下拉模式（IPU）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    button_init(&btn1, read_button_GPIO, 1, btn1_id);
    button_init(&btn2, read_button_GPIO, 1, btn2_id);
    button_init(&btn3, read_button_GPIO, 0, btn3_id);
    button_init(&btn4, read_button_GPIO, 0, btn4_id);
    button_init(&btn5, read_button_GPIO, 0, btn5_id);
    button_init(&btn6, read_button_GPIO, 0, btn6_id);
    button_init(&btn7, read_button_GPIO, 0, btn7_id);
    button_init(&btn8, read_button_GPIO, 0, btn8_id);
    button_init(&btn9, read_button_GPIO, 0, btn9_id);
    button_init(&btn10, read_button_GPIO, 0, btn10_id);
    button_init(&btn11, read_button_GPIO, 0, btn11_id);
    button_init(&btn12, read_button_GPIO, 0, btn12_id);
    button_init(&btn13, read_button_GPIO, 0, btn13_id);
    button_init(&btn14, read_button_GPIO, 0, btn14_id);
    button_init(&btn15, read_button_GPIO, 0, btn15_id);
    button_init(&btn16, read_button_GPIO, 0, btn16_id);
    button_init(&btn17, read_button_GPIO, 0, btn17_id);
    button_init(&btn18, read_button_GPIO, 0, btn18_id);

    button_attach(&btn1, SINGLE_CLICK, LSW_SINGLE_Click_Handler);
    button_attach(&btn2, SINGLE_CLICK, RSW_SINGLE_Click_Handler);
    button_attach(&btn3, SINGLE_CLICK, LB_SINGLE_Click_Handler);
    button_attach(&btn4, SINGLE_CLICK, LT_SINGLE_Click_Handler);
    button_attach(&btn5, SINGLE_CLICK, RB_SINGLE_Click_Handler);
    button_attach(&btn6, SINGLE_CLICK, RT_SINGLE_Click_Handler);
    button_attach(&btn7, SINGLE_CLICK, BACK_SINGLE_Click_Handler);
    button_attach(&btn8, SINGLE_CLICK, HOME_SINGLE_Click_Handler);
    button_attach(&btn9, SINGLE_CLICK, START_SINGLE_Click_Handler);
    button_attach(&btn10, SINGLE_CLICK, A_SINGLE_Click_Handler);
    button_attach(&btn11, SINGLE_CLICK, B_SINGLE_Click_Handler);
    button_attach(&btn12, SINGLE_CLICK, X_SINGLE_Click_Handler);
    button_attach(&btn13, SINGLE_CLICK, Y_SINGLE_Click_Handler);
    button_attach(&btn14, SINGLE_CLICK, GTA_SINGLE_Click_Handler);
    button_attach(&btn15, SINGLE_CLICK, GTB_SINGLE_Click_Handler);
    button_attach(&btn16, SINGLE_CLICK, GTC_SINGLE_Click_Handler);
    button_attach(&btn17, SINGLE_CLICK, GTD_SINGLE_Click_Handler);
    button_attach(&btn18, SINGLE_CLICK, GTE_SINGLE_Click_Handler);


    button_start(&btn1);
    button_start(&btn2);
    button_start(&btn3);
    button_start(&btn4);
    button_start(&btn5);
    button_start(&btn6);
    button_start(&btn7);
    button_start(&btn8);
    button_start(&btn9);
    button_start(&btn10);
    button_start(&btn11);
    button_start(&btn12);
    button_start(&btn13);
    button_start(&btn14);
    button_start(&btn15);
    button_start(&btn16);
    button_start(&btn17);
    button_start(&btn18);

}
```

这段代码是用于初始化和检测按键的CH32微控制器代码。注释和解释如下：

1. ButtonInit函数用于初始化按键的IO引脚。它配置了不同的IO引脚作为输入，一些引脚使用了上拉（IPU）模式，另一些引脚使用了下拉（IPD）模式，这取决于按键硬件连接和工作原理；同时调用MultiTimer的初始化函数，添加需要的按键回调函数，这里是用到了单击事件。


### 3.1.3 下载验证

我们把固件程序下载进去可以，打开串口调试助手；接H3排针的TX到USB转TTL模块，可以打印这18个按键按下的Log信息；

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/ef16e31c8fa648ff85efda1b65a6211d.png#pic_center)

## 3.2 实例Eg02_AnalogDebug

该实例是为了测试两个摇杆；我们将采用ADC DMA方式读取4个通道的AD值，并打印出来；

### 3.1.1硬件设计

![image-20231212233809439](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20231212233809439.png)

如上图所示，共有4个模拟输入通道，我们将其连接到ADC1、ADC2、ADC4、ADC5； 




### 3.1.2 软件设计

#### 3.1.2.1 ADC配置

ADC的配置文件为Analog.c和Analog.h;

```C
/*
 * Button.c
 *
 *  Created on: Dec 4, 2023
 *      Author: Administrator
 */

#include "Analog.h"


u16 ADC_ConvertedValue[LENGTH]={0};

//ADC对应GPIO初始化配置以及ADC初始化配置
void adc_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1 , ENABLE ); //使能GPIOA时钟和ADC

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_5; //PA1~5对应ADC通道1~5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //GPIO模式为模拟输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  //配置ADC为独立模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;        //多通道模式下开启扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  //设置开启连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //设置转换不是由外部触发启动，软件触发启动
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //设置ADC数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = LENGTH;           //规则转换的ADC通道的数目
    ADC_Init(ADC1, &ADC_InitStructure);                    //根据ADC_InitStructure中指定的参数初始化ADC1寄存器

    RCC_ADCCLKConfig(RCC_PCLK2_Div6); //设置ADC时钟分频为6分频

    ADC_Cmd(ADC1, ENABLE);      //使能ADC1

    ADC_ResetCalibration(ADC1); //重置ADC1校准寄存器。

    while(ADC_GetResetCalibrationStatus(ADC1)); //等待复位校准结束

    ADC_StartCalibration(ADC1); //开启AD校准

    while(ADC_GetCalibrationStatus(ADC1));      //等待校准结束
}

//ADC DMA模式配置
void DMA_Tx_Init( void )
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE ); //使能开启DMA时钟

    DMA_DeInit(DMA1_Channel1); //复位DMA控制器

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->RDATAR;  //配置外设地址为ADC数据寄存器地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC_ConvertedValue; //配置存储器地址为读取ADC值地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;              //配置数据源为外设，即DMA传输方式为外设到存储器
    DMA_InitStructure.DMA_BufferSize = LENGTH;                      //设置DMA数据缓冲区大小，此处设置为LENGTH
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//设置DMA外设递增模式关闭
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;         //设置DMA存储器递增模式开启
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //设置外设数据大小为半字，即两个字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         //设置存储器数据大小为半字，即两个字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;     //设置DMA模式为循环传输模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; //设置DMA传输通道优先级为高，当使用一 DMA通道时，优先级设置不影响
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;        //因为此DMA传输方式为外设到存储器，因此禁用存储器到存储器传输方式
    DMA_Init( DMA1_Channel1, &DMA_InitStructure );      //初始化DMA

    DMA_Cmd(DMA1_Channel1 , ENABLE);  //使能DMA
}

void ADC_DMA_CONF(void)
{
    adc_Init();

    DMA_Tx_Init();

    // 配置 ADC 通道转换顺序为1，第一个转换，采样时间为55.5个时钟周期
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_239Cycles5);

    // 使能ADC DMA 请求
    ADC_DMACmd(ADC1, ENABLE);

    // 由于没有采用外部触发，所以使用软件触发ADC转换
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}


```

这段C代码是用于配置和使用模拟-数字转换器（ADC）和直接内存访问（DMA）的函数。主要步骤如下：

1. **ADC初始化（`adc_Init`函数）：**
   - 配置GPIO引脚，使其成为模拟输入。
   - 初始化ADC，设置为独立模式、连续转换模式，并启用扫描模式。
   - 配置ADC通道和执行校准。
2. **DMA初始化（`DMA_Tx_Init`函数）：**
   - 配置DMA，将ADC的转换结果传输到内存中的数组中。
   - 配置DMA为循环传输模式，以便持续地将ADC转换结果传输到数组中。
3. **ADC和DMA配置（`ADC_DMA_CONF`函数）：**
   - 调用`adc_Init`和`DMA_Tx_Init`函数以配置ADC和DMA。
   - 配置ADC通道，设置采样时间。
   - 启用ADC的DMA请求。
   - 使用软件触发启动ADC转换。

总体而言，这段代码的功能是初始化ADC和DMA，使系统能够以连续模式从多个通道采集模拟信号，并将结果存储在一个数组中。这对于需要高效处理模拟信号的应用（例如传感器数据采集）是很常见的配置。

#### 3.1.2.1 访问打印AD值

在main函数while循环里面，我们每隔500ms打印一次AD值

```c
    while(1)
    {
        tick++;
        if((tick%100)==0)//500ms
        {
            tick=0;
            printf("\r\n The current ADCH1 value = %d \r\n", ADC_ConvertedValue[0]);
            printf("\r\n The current ADCH2 value = %d \r\n", ADC_ConvertedValue[1]);
            printf("\r\n The current ADCH3 value = %d \r\n", ADC_ConvertedValue[2]);
            printf("\r\n The current ADCH4 value = %d \r\n", ADC_ConvertedValue[3]);
        }
        button_ticks();
        Delay_Ms(5);
    }
```




### 3.1.3 下载验证

我们把固件程序下载进去可以，打开串口调试助手；摇动摇杆打印Log信息；



