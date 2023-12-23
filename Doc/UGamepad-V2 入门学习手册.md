


# 序

欢迎踏入LDSCITECH的评估板世界，一个充满激情、创新和未知可能性的领域！评估板是连接你和技术未来之间的桥梁，它们是探索新概念、验证创意和打造创新产品的关键。在这个令人兴奋的旅程中，LDSCITECH将与您一同探索微控制器、传感器和电机评估板的神奇世界。

评估板是通往数字创新的大门，为您提供实验和测试的平台，让您在不牺牲复杂性的情况下深入了解各种技术。无论您是初学者还是经验丰富的工程师，评估板都是您在设计过程中的得力助手，帮助您迅速验证概念、加速产品开发，最终将您的想法变为现实。

在LDSCITECH，我们深知评估板的关键作用，因此致力于为您提供最先进、高性能的微控制器、先进的传感器技术以及创新的电机评估板。通过这些工具，您将能够突破技术界限，挑战创新高峰，为您的项目赋能，引领行业变革。

我们相信，每一个从事技术创新的人都值得有一个让创意飞翔的平台。LDSCITECH将与您携手共进，提供全方位的支持，帮助您在评估板的世界里找到您独特的路径。

准备好迎接新的技术挑战吗？让我们一同踏上这段激动人心的评估板之旅吧！

非常感谢您的理解和支持！技术领域的学习和分享是一个不断演进的过程，每个人都在不断提高自己的水平。善意的反馈和建议是进步的动力，同时也为作者提供了改进的机会。

如果您在文档或视频中发现任何不准确或不清楚的地方，或者有任何改进的建议，我们真诚地欢迎您提出。您的意见将有助于提高质量，使信息更加准确和易于理解，使得文档和视频更好地满足读者和观众的需求。

LDSCITECH始终秉承着开放、包容和学习的精神，我们期待与您共同进步，共同创造更优质的技术资源。再次感谢您的关注和支持！



| **实例**         | **描述**                      |
| ---------------- | ----------------------------- |
| Eg01_ButtonDebug | 实现一个Joystick摇杆设备      |
| Eg02_AnalogDebug | 点亮WS2812B灯珠并实现七彩渐变 |
| Eg03_MultiTimer  | 移植MultiTimer软件定时器模块  |
| Eg04_Mouse       | 实现模拟鼠标功能              |
|                  |                               |
|                  |                               |
|                  |                               |
|                  |                               |
|                  |                               |
|                  |                               |
|                  |                               |
|                  |                               |

# 第一部分、硬件概述

## 1.1 3D图

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/1f243e531c824c7c92ab98acf9c48af7.png#pic_center)



如图1.1所示Gamepad评估板配置了11个6*6轻触按键，两个摇杆（Joystick），搭载一颗LED灯珠，并将UART1串口（H3），编程接口（SWD H4），Type-C接口引出;  

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

本节这个实例主要是为了测试摇杆电位器；

### 3.2.1硬件设计

摇杆电位器原理图如下所示：

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/b4d84c401bc745cfa4bad9338f81b6e2.png#pic_center)

所以，我们只要配置4路ADC输入检测两个电位器的XY;  MCU的配置如下：
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/fcda7902e7e34a74af424b84adc1551d.png)

### 3.2.2 软件设计

#### 3.2.2.1 ADC初始化配置


```c
/*
 * Analog.c
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

这段代码的功能是配置STM32的ADC和DMA，实现对多个通道的模拟输入进行连续转换，并将转换结果存储在数组 ADC_ConvertedValue 中。这通常用于读取传感器等模拟信号。在使用时，可以在 ADC_ConvertedValue 数组中获取相应通道的ADC转换结果。



#### 3.2.2.2 用户代码

```c
int main(void)
{
    u16 tick=0;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d;\r\n", SystemCoreClock);
    ButtonInit();
    ADC_DMA_CONF();
    printf("ADC Debug Demo;\r\n");
    while(1)
    {
        tick++;
        if((tick%100)==0)//500ms
        {
            tick=0;
            printf("\r\n The current ADCH1 value = %d \r\n", 				ADC_ConvertedValue[0]);
            printf("\r\n The current ADCH2 value = %d \r\n", 	ADC_ConvertedValue[1]);
            printf("\r\n The current ADCH3 value = %d \r\n", 		ADC_ConvertedValue[2]);
            printf("\r\n The current ADCH4 value = %d \r\n", 	ADC_ConvertedValue[3]);
        }
        button_ticks();
        Delay_Ms(5);
    }
}
```

测试代码我们500ms直接打印AD值日志一次；


### 3.2.3 下载验证

我们把固件程序下载进去可以，打开串口调试助手；接H3排针的TX到USB转TTL模块，可以打印这4个通道ADC的Log信息；

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/f9c0bb22b2ee43728a6ca96eeeaf0b80.png)

## 3.3 实例Eg03_MultiTimer

本节使用一个开源的软件定时器套件MultiTimer来实现一个软件框架，用于实现LED、ADC、按键多任务；关于MultiTimer开源，大家可以访问开源的GitHub链接学习：https://github.com/0x1abin/MultiTimer.git


### 3.3.1硬件设计

本节用到摇杆电位器按钮和LED，摇杆电位器和按键上两节已经介绍，LED的原理图如下：
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/8a050ad74de049ee9ebf5d4121f2ac96.png)
LED接到了MCU的PB3：
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/fcda7902e7e34a74af424b84adc1551d.png)

### 3.3.2 软件设计

#### 3.3.2.1 LED配置


```c
#include "LED.h"

/*********************************************************************
 * @fn      GPIO_Toggle_INIT
 *
 * @brief   Initializes GPIOA.0
 *
 * @return  none
 */
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
```
这段代码的功能是配置LED的PB3作为推挽输出；

同时LED.h 用一个宏实现IO拉高拉低；

```c
#ifndef MYBSP_LED_H_
#define MYBSP_LED_H_

#include "debug.h"

#define     LED(x)      (x?GPIO_ResetBits(GPIOB,GPIO_Pin_3):GPIO_SetBits(GPIOB,GPIO_Pin_3))

extern void LED_Init(void);

#endif /* MYBSP_LED_H_ */
```

#### 3.3.2.2 SYStick配置
因为MultiTimer需要就像RTOS需要Tick，这里配置Systick作为它的时基；
```c
void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      SYSTICK_Init_Config
 *
 * @brief   SYSTICK_Init_Config.
 *
 * @return  none
 */
void SYSTICK_Init_Config(u64 ticks)
{
    SysTick->SR = 0;
    SysTick->CNT = 0;
    SysTick->CMP = ticks;
    SysTick->CTLR =0xF;

    NVIC_SetPriority(SysTicK_IRQn, 15);
    NVIC_EnableIRQ(SysTicK_IRQn);
}

void SysTick_Handler(void)
{
    SysTick->SR = 0;
    uwTick++;
}
```

#### 3.3.2.3 应用代码
最后我们创建三个定时器分别执行对应的任务，如下代码：

```c
vu32 uwTick;

MultiTimer timer1;
MultiTimer timer2;
MultiTimer timer3;
uint64_t PlatformTicksGetFunc(void) {
    return uwTick;
}
void LEDTimer1Callback(MultiTimer* timer, void *userData)
{
    static FlagStatus LedSta=RESET;
    LED(LedSta);
    LedSta=~LedSta;
    printf("LED Status:%d\r\n",LedSta);
    MultiTimerStart(timer, 500, LEDTimer1Callback, userData);
}
void ADCTimer2Callback(MultiTimer* timer, void *userData)
{
    printf("\r\n The current ADCH1 value = %d \r\n", ADC_ConvertedValue[0]);
    printf("\r\n The current ADCH2 value = %d \r\n", ADC_ConvertedValue[1]);
    printf("\r\n The current ADCH3 value = %d \r\n", ADC_ConvertedValue[2]);
    printf("\r\n The current ADCH4 value = %d \r\n", ADC_ConvertedValue[3]);

    MultiTimerStart(timer, 1000, ADCTimer2Callback, userData);
}
void ButtonTimer3Callback(MultiTimer* timer, void *userData)
{
    button_ticks();;
    MultiTimerStart(timer, 5, ButtonTimer3Callback, userData);
}
void PollSystemInit(void) {
    MultiTimerInstall(PlatformTicksGetFunc);
    MultiTimerStart(&timer1, 500, LEDTimer1Callback, NULL);
    MultiTimerStart(&timer2, 1000, ADCTimer2Callback, NULL);
    MultiTimerStart(&timer3, 5, ButtonTimer3Callback, NULL);
    SYSTICK_Init_Config(SystemCoreClock/1000-1);
}

```

### 3.3.3 下载验证

我们把固件程序下载进去可以，打开串口调试助手；接H3排针的TX到USB转TTL模块，可以打印三个任务Log信息；
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/0e664def8cf446f2af99d8f53f3af23e.png)

## 3.4 实例Eg04_Mouse

本节实现一个模拟鼠标功能；左摇杆实现的是XY轴移动；右摇杆实现的是滚轮上下，右摇杆按键实现的是鼠标中键，X和B键实现的是鼠标左右键；


### 3.4.1硬件设计

前面几个章节基本把所需的硬件介绍完成，本节开始，硬件直接参考原理图文档。


### 3.4.2 软件设计

我们直接在官方例程\CH32V20xEVT\EVT\EXAM\USB\USBFS\DEVICE\CompatibilityHID这个工程目录中User部分USB相关代码（ch32v20x_usb.h、ch32v20x_usbfs_device.c、ch32v20x_usbfs_device.h、usbd_desc.c、usbd_desc.h）复制粘贴到我们自己的Mouse工程中；如下图红色方框所示：
![Usb部分代码](https://img-blog.csdnimg.cn/direct/0943776cbb194507b54c5227ae75946c.png)



#### 3.4.2.1 USB中断请求

ch32v20x_usbfs_device中的USBHD_IRQHandler是USB的中断服务函数；

Usb标准请求USBFS_SetupReqCode中有个USB_GET_DESCRIPTOR获取描述符的请求，其中获取了以下几种描述符：

USB_DESCR_TYP_DEVICE：设备描述符；

USB_DESCR_TYP_CONFIG：配置描述符；

USB_DESCR_TYP_HID：HID描述符；

USB_DESCR_TYP_REPORT：报表描述符；

USB_DESCR_TYP_STRING；字符串描述符；

上述描述符定义在usbd_desc.c；

##### 3.4.2.1.1 设备描述符


```c
/* Device Descriptor */
const uint8_t MyDevDescr[ ] =
{
    0x12,                                                   // bLength
    0x01,                                                   // bDescriptorType
    0x00, 0x02,                                             // bcdUSB
    0x00,                                                   // bDeviceClass
    0x00,                                                   // bDeviceSubClass
    0x00,                                                   // bDeviceProtocol
    DEF_USBD_UEP0_SIZE,                                     // bMaxPacketSize0
    (uint8_t)DEF_USB_VID, (uint8_t)( DEF_USB_VID >> 8 ),    // idVendor
    (uint8_t)DEF_USB_PID, (uint8_t)( DEF_USB_PID >> 8 ),    // idProduct
    0x00, DEF_IC_PRG_VER,                                   // bcdDevice
    0x01,                                                   // iManufacturer
    0x02,                                                   // iProduct
    0x03,                                                   // iSerialNumber
    0x01,                                                   // bNumConfigurations
};
```

设备描述符的定义注释已经非常清楚，需要注意的是USB_VID和USB_PID，VID是厂商描述符，是需要向Usb-IF组织申请的，PID是产品ID可以自定义；我们这里定义如下：

```c
#define DEF_USB_VID                   0x0810
#define DEF_USB_PID                   0x0001
```

##### 3.4.2.1.2 配置描述符集合

```c
/* Configuration Descriptor Set */
const uint8_t MyCfgDescr[ ] =
{
        0x09, /* bLength: Configuration Descriptor size */
        USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
        USB_CUSTOM_HID_CONFIG_DESC_SIZ,
        /* wTotalLength: Bytes returned */
        0x00,
        0x01,         /*bNumInterfaces: 1 interface*/
        0x01,         /*bConfigurationValue: Configuration value*/
        0x00,         /*iConfiguration: Index of string descriptor describing
        the configuration*/
        0xC0,         /*bmAttributes: bus powered */
        0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

        /************** Descriptor of CUSTOM HID interface ****************/
        /* 09 */
        0x09,         /*bLength: Interface Descriptor size*/
        USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
        0x00,         /*bInterfaceNumber: Number of Interface*/
        0x00,         /*bAlternateSetting: Alternate setting*/
        0x01,         /*bNumEndpoints*/
        0x03,         /*bInterfaceClass: CUSTOM_HID*/
        0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
        0x02,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
        0,            /*iInterface: Index of string descriptor*/
        /******************** Descriptor of CUSTOM_HID *************************/
        /* 18 */
        0x09,         /*bLength: CUSTOM_HID Descriptor size*/
        CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
        0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
        0x01,
        0x00,         /*bCountryCode: Hardware target country*/
        0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
        0x22,         /*bDescriptorType*/
        USBD_CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
        0x00,
        /******************** Descriptor of Custom HID endpoints ********************/
        /* 27 */
        0x07,          /*bLength: Endpoint Descriptor size*/
        USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

        CUSTOM_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
        0x03,          /*bmAttributes: Interrupt endpoint*/
        CUSTOM_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
        0x00,
        CUSTOM_HID_FS_BINTERVAL,          /*bInterval: Polling Interval */
        /* 34 */
};
```

如上图，配置描述符集合里面包含了配置描述符、接口描述符、HID描述符、以及端点描述符；

##### 3.4.2.1.3 报表描述符

```c
/* Mouse Report Descriptor */
const uint8_t MouseRepDesc[ ] =
{
        0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
        0x09, 0x02,                    // USAGE (Mouse)
        0xa1, 0x01,                    // COLLECTION (Application)
        0x09, 0x01,                    //   USAGE (Pointer)
        0xa1, 0x00,                    //   COLLECTION (Physical)
        0x05, 0x09,                    //     USAGE_PAGE (Button)
        0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
        0x29, 0x03,                    //     USAGE_MAXIMUM (Button 3)
        0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
        0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
        0x95, 0x03,                    //     REPORT_COUNT (3)
        0x75, 0x01,                    //     REPORT_SIZE (1)
        0x81, 0x02,                    //     INPUT (Data,Var,Abs)
        0x95, 0x01,                    //     REPORT_COUNT (1)
        0x75, 0x05,                    //     REPORT_SIZE (5)
        0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
        0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
        0x09, 0x30,                    //     USAGE (X)
        0x09, 0x31,                    //     USAGE (Y)
        0x09, 0x38,                    //     USAGE (Wheel)
        0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
        0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
        0x75, 0x08,                    //     REPORT_SIZE (8)
        0x95, 0x03,                    //     REPORT_COUNT (3)
        0x81, 0x06,                    //     INPUT (Data,Var,Rel)
        0xc0,                          //   END_COLLECTION
        0xc0                           //END_COLLECTION
};
```

以上是鼠标的报表描述符,详细描述了鼠标协议报文的各个字段。

##### 3.4.2.3 字符串描述符

最后我们来看一下字符串描述符，字符串有以下这些，需要主要的是使用的是Unicode编码：

Descriptor 0, Language descriptor：语言ID描述符；

Descriptor 1, Manufacturers String descriptor：厂商字符串描述符；

Descriptor 2, Product String descriptor：产品字符串描述符；

Descriptor 3, Serial-number String descriptor ：序列号描述符；

###### 3.4.2.3.1 语言ID描述符

语言ID描述符一般比较固定，无需修改；

```c
/* Language Descriptor */
const uint8_t MyLangDescr[ ] =
{
    0x04,
    0x03,
    0x09,
    0x04
};
```

###### 3.4.2.3.2 厂商字符串描述符

这个是厂商名字的编码，这个是我们的商标名称“LDSCITECH”

```c
/* Manufacturer Descriptor */
const uint8_t MyManuInfo[ ] =
{
    0x16,0x03,0x4C,0x00,0x44,0x00,0x53,0x00,0x43,0x00,0x49,0x00,0x54,0x00,0x45,0x00,
    0x43,0x00,0x48,0x00,0x45,0x00
};
```

###### 3.4.2.3.3 产品字符串描述符

这个是我们自定义的编码，这个名称我们就取"LD Mouse"

```c
/* Product Information */
const uint8_t MyProdInfo[ ]  =
{
    0x12,0x03,0x4C,0x00,0x44,0x00,0x20,0x00,0x4D,0x00,0x6F,0x00,0x75,0x00,0x73,0x00,
    0x65,0x00
};
```

###### 3.4.2.3.4 序列号描述符

这里的序列号描述符参考了ST的做法，用的是UID，关于UID，我们的CH32V203参考手册也有介绍；

```c
#define         UID_BASE              0x1FFFF7E8UL    /*!< Unique device ID register base address */
#define         DEVICE_ID1          (UID_BASE)
#define         DEVICE_ID2          (UID_BASE + 0x4)
#define         DEVICE_ID3          (UID_BASE + 0x8)

/* Serial Number Information */
/* Serial Number Information */
uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] = {
        USB_SIZ_STRING_SERIAL,
        USB_DESC_TYPE_STRING,};

/**
  * @brief  Convert Hex 32Bits value into char
  * @param  value: value to convert
  * @param  pbuf: pointer to the buffer
  * @param  len: buffer length
  * @retval None
  */
void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len)
{
  uint8_t idx = 0;

  for (idx = 0; idx < len; idx++)
  {
    if (((value >> 28)) < 0xA)
    {
      pbuf[2 * idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2 * idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[2 * idx + 1] = 0;
  }
}
void Get_SerialNum(void)
{
  uint32_t deviceserial0, deviceserial1, deviceserial2;

  deviceserial0 = *(uint32_t *) DEVICE_ID1;
  deviceserial1 = *(uint32_t *) DEVICE_ID2;
  deviceserial2 = *(uint32_t *) DEVICE_ID3;

  deviceserial0 += deviceserial2;

  if (deviceserial0 != 0)
  {
    IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8);
    IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
  }
}
```

#### 3.4.2.2 用户代码

```c
void PollSystemInit(void) {
    MultiTimerInstall(PlatformTicksGetFunc);
    MultiTimerStart(&timer1, 5, LEDTimer1Callback, NULL);
    MultiTimerStart(&timer2, 10, ADCTimer2Callback, NULL);
    MultiTimerStart(&timer3, 15, ButtonTimer3Callback, NULL);
    MultiTimerStart(&timer4, 20, ReportTimer4Callback, NULL);
    SYSTICK_Init_Config(SystemCoreClock/1000-1);
}
```

我们还是创建新建4个软件定时器，在回调函数里面执行对应的外设任务；LED同上节，无任何变化；接下来分别介绍各线程任务；

##### 3.4.2.2.1 鼠标结构体

```c
typedef struct
{
    u8 button;
    s8 x;
    s8 y;
    s8 wheel;
}MouseTypdef;
MouseTypdef  MS_Data_Pack;
```

这里定义了鼠标报文，第一个结构体成员button代表鼠标按键，其中bit0代表鼠标左键，bit1代表鼠标右键，bit2则是鼠标中键；第二第三个结构体成员xy代表鼠标坐标位移；第四个wheel是滚轮位移；

##### 3.2.2.2.2 ADC数据处理

```c
void ADCTimer2Callback(MultiTimer* timer, void *userData)
{
    static u8 AdcCount=0;
    static u16 wtick=0;
    static u32 A1Sum=0,A2Sum=0,A3Sum=0,A4Sum=0;
    //u16 xtemp=0,ytemp=0,xtemp1=0,ytemp1=0;
    u16 xtemp=0,ytemp=0,xtemp1=0;//ytemp1=0;

    A1Sum+=ADC_ConvertedValue[0];
    A2Sum+=ADC_ConvertedValue[1];
    A3Sum+=ADC_ConvertedValue[2];
    A4Sum+=ADC_ConvertedValue[3];
    AdcCount+=1;
    if(AdcCount==10)
    {
        //X
        xtemp=Map( A1Sum/AdcCount, 0, 4095, 0, UINT8_MAX );
        ytemp=Map( A2Sum/AdcCount, 0, 4095, 0, UINT8_MAX );

        xtemp1=Map( A3Sum/AdcCount, 0, 4095, 0, UINT8_MAX );
        //ytemp1=Map( A4Sum/AdcCount, 0, 4095, 0, UINT8_MAX );
        //printf("CH: %d %d %d %d\r\n", xtemp,ytemp,xtemp1,ytemp1);
        A1Sum=0;
        A2Sum=0;
        A3Sum=0;
        //A4Sum=0;
        AdcCount=0;

        if(xtemp<(CENTER_X-10))
        {
            MS_Data_Pack.x=-(((CENTER_X-xtemp)>>DIV)+1);
        }else if(xtemp>(CENTER_X+10))
        {
            MS_Data_Pack.x=((xtemp-CENTER_X)>>DIV)+1;;
        }else{
            MS_Data_Pack.x=0;
        }
        if(ytemp<(CENTER_Y-10))
        {
            MS_Data_Pack.y=-(((CENTER_Y-ytemp)>>DIV)+1);
        }else if(ytemp>(CENTER_Y+10))
        {
            MS_Data_Pack.y=((ytemp-CENTER_Y)>>DIV)+1;;
        }else{
            MS_Data_Pack.y=0;
        }
        if(xtemp1<(CENTER_X1-64))
        {
            if(++wtick==5)
            {
                wtick=0;
                MS_Data_Pack.wheel=1;
            }else{
                MS_Data_Pack.wheel=0;
            }

        }else if(xtemp1>(CENTER_X1+64))
        {
            if(++wtick==5)
            {
                wtick=0;
                MS_Data_Pack.wheel=-1;
            }else{
                MS_Data_Pack.wheel=0;
            }
        }else{
            wtick=0;
            MS_Data_Pack.wheel=0;
        }

        //printf("X=%d Y=%d wheel=%d\r\n", MS_Data_Pack.x,MS_Data_Pack.y);
    }

    MultiTimerStart(timer, 1, ADCTimer2Callback, userData);
}
```

这部分代码是取10次平均后得到平均值再做一个映射，将范围0~4095转出0~255；根据摇杆偏移中心点多少算出鼠标坐标和滚轮位移；

##### 3.4.2.2.3 按键数据处理

按键任务回调函数如下，一样没有变化；

```c
void ButtonTimer3Callback(MultiTimer* timer, void *userData)
{
    button_ticks();
    MultiTimerStart(timer, 5, ButtonTimer3Callback, userData);
}
```

在button.c中主要注册几个按键事件回调，按下和释放，如下

```c
void RSW_PRESS_UP_Handler(void* btn) {
    printf("RSW_PRESS_UP_Handler\r\n");
    MS_Data_Pack.button&=(~0x04);
}
void RSW_PRESS_DOWN_Handler(void* btn) {
    printf("RSW_PRESS_DOWN_Handler\r\n");
    MS_Data_Pack.button|=0x04;
}
void B_PRESS_UP_Handler(void* btn) {
    printf("B_PRESS_UP_Handler\r\n");
    MS_Data_Pack.button&=(~0x02);
}
void B_PRESS_DOWN_Handler(void* btn) {
    printf("B_PRESS_DOWN_Handler\r\n");
    MS_Data_Pack.button|=0x02;
}
void X_PRESS_UP_Handler(void* btn) {
    printf("X_PRESS_UP_Handler\r\n");
    MS_Data_Pack.button&=(~0x01);
}
void X_PRESS_DOWN_Handler(void* btn) {
    printf("X_PRESS_DOWN_Handler\r\n");
    MS_Data_Pack.button|=0x01;
}
void ButtonInit(void) {
    //省略部分代码
    button_attach(&btn2, PRESS_UP, RSW_PRESS_UP_Handler);
    button_attach(&btn2, PRESS_DOWN, RSW_PRESS_DOWN_Handler);
    button_attach(&btn11, PRESS_UP, B_PRESS_UP_Handler);
    button_attach(&btn11, PRESS_DOWN, B_PRESS_DOWN_Handler);
    button_attach(&btn12, PRESS_UP, X_PRESS_UP_Handler);
    button_attach(&btn12, PRESS_DOWN, X_PRESS_DOWN_Handler);
 	//省略部分代码

}
```

可以看到，上面按键代码主要是在按键按下置位弹起清零；



### 3.4.3 下载验证

我们把固件程序下载进去可以看到LD Mouse鼠标，摇动左摇杆是鼠标指针的位移；右摇杆左右没有功能，上下是滚轮上下，右摇杆按键是鼠标中键；X是鼠标左键，B是鼠标右键；
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/340610e6775c422aaf8dca3b1d51a538.png)
