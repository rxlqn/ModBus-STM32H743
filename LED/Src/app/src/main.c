/*
*********************************************************************************************************
*
*	模块名称 : 主程序模块。
*	文件名称 : main.c
*	版    本 : V1.0
*	说    明 : RS485 MODBUS主站例程（使用的是串口3）。
*              本例程主要讲解MODBUS协议从站的命令处理方法,包含了常用的命令。
*   实验内容：
*              1. 接好硬件,(1)串口1(打印实验数据)  (2)485接口(收发命令)
*              2. 按键定义:
*								  读	  从机地址 功能码	寄存器首地址   寄存器数量	校验码
*					KEY_DOWN_K1 : 发送 0x 	01 		 01		   01 01 		00 04 		6D F5
*					KEY_DOWN_K2	: 发送 0x   01       03        03 01        00 02       95 8F	
* 					JOY_DOWN_OK	: 发送 0x   01       02        02 01        00 03       68 73  				
*					JOY_UP_OK   : 发送 0x   01       04        04 01        00 01       61 3A
*								  写(1个) 从机地址 功能码    寄存器地址	   写入的值		校验码
*					JOY_DOWN_U	: 发送 0x   01       06        03 01        00 01       19 8E
*					JOY_DOWN_D	: 发送 0x   01       06        03 01        00 00       D8 4E
*					JOY_DOWN_L	: 发送 0x   01       05        01 01        00 01       5C 36
*					JOY_DOWN_R	: 发送 0x   01       05        01 01        00 00       9D F6
*								  写(多个)从机地址 功能码    寄存器地址    寄存器数量  字节数   写入的值1   写入的值2   校验码
*					KEY_DOWN_K3 : 发送 0x   01       10        03 01        00 02        04      00 01       02 03      36 32
*	修改记录 :
*		版本号   日期       作者       说明
*		v1.0    2016-01-11  armfly     首发
*
*	Copyright (C), 2015-2020, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"			/* 底层硬件驱动 */
#include "main.h"
#include "modbus_host.h"

/* 定义例程名和例程发布日期 */
#define EXAMPLE_NAME	"V6-RS485 MODBUS主站例程"
#define EXAMPLE_DATE	"2016-01-11"
#define DEMO_VER		"1.0"

/* 仅允许本文件内调用的函数声明 */
static void PrintfLogo(void);
static void DispMenu(void);

PRINT_MODS_T g_tPrint;
/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: c程序入口
*	形    参：无
*	返 回 值: 错误代码(无需处理)
*********************************************************************************************************
*/
int main(void)
{
	uint8_t ucKeyCode;				/* 按键代码 */
	MSG_T ucMsg;					/* 消息代码 */

	bsp_Init();						/* 硬件初始化 */
	PrintfLogo();					/* 打印例程信息到串口1 */
	DispMenu();						/* 打印寄存器的值 */
	
	/* 进入主程序循环体 */
	while (1)
	{
		bsp_Idle();					/* 这个函数在bsp.c文件。用户可以修改这个函数实现CPU休眠和喂狗 */
		
		if (bsp_GetMsg(&ucMsg))		/* 读取消息代码 */
		{
			switch (ucMsg.MsgCode)
			{
				case MSG_MODS:		
					DispMenu();		/* 打印实验结果 */
					break;
				
				default:
					break;
			}
		}
	
		/* 按键滤波和检测由后台systick中断服务程序实现，我们只需要调用bsp_GetKey读取键值即可。 */
		ucKeyCode = bsp_GetKey();	/* 读取键值, 无键按下时返回 KEY_NONE = 0 */
		if (ucKeyCode != KEY_NONE)
		{
			bsp_PutMsg(MSG_MODS, 0);
			
			switch (ucKeyCode)
			{			
				case KEY_DOWN_K1:				/* K1键按下 */
					if (MODH_ReadParam_01H(REG_D01, 4) == 1) ;else ;
					break;
				
				case KEY_DOWN_K2:				/* K2键按下 */
					if (MODH_ReadParam_03H(REG_P01, 2) == 1) ;else ;
					break;
				
				case KEY_DOWN_K3:				/* K3键按下 */
					{
						uint8_t buf[4];
						
						buf[0] = 0x01;
						buf[1] = 0x02;
						buf[2] = 0x03;
						buf[3] = 0x04;
						if (MODH_WriteParam_10H(REG_P01, 2, buf) == 1) ;else ;
					}
					break;
				
				case JOY_DOWN_U:				/* 摇杆UP键弹起 */
					if (MODH_WriteParam_06H(REG_P01, 1) == 1) ;else ;
					break;
				
				case JOY_DOWN_D:				/* 摇杆DOWN键按下 */
					if (MODH_WriteParam_06H(REG_P01, 0) == 1) ;else ;
					break;
				
				case JOY_DOWN_L:				/* 摇杆LEFT键弹起 */
					if (MODH_WriteParam_05H(REG_D01, 1) == 1) ;else ;
					break;
				
				case JOY_DOWN_R:				/* 摇杆RIGHT键弹起 */
					if (MODH_WriteParam_05H(REG_D01, 0) == 1) ;else ;
					break;
				
				case JOY_DOWN_OK:				/* 摇杆OK键按下 */
					if (MODH_ReadParam_02H(REG_T01, 3) == 1) ;else ;
					break;

				case JOY_UP_OK:					/* 摇杆OK键弹起 */
					if (MODH_ReadParam_04H(REG_A01, 1) == 1) ;else ;	
					break;
				
				default:
					/* 其它的键值不处理 */
					break;
			}
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: PrintfLogo
*	功能说明: 打印例程名称和例程发布日期, 接上串口线后，打开PC机的超级终端软件可以观察结果
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void PrintfLogo(void)
{
	/* 检测CPU ID */
	{
		/* 参考手册：
			32.6.1 MCU device ID code
			33.1 Unique device ID register (96 bits)
		*/
		uint32_t CPU_Sn0, CPU_Sn1, CPU_Sn2;

		CPU_Sn0 = *(__IO uint32_t*)(0x1FFF7A10);
		CPU_Sn1 = *(__IO uint32_t*)(0x1FFF7A10 + 4);
		CPU_Sn2 = *(__IO uint32_t*)(0x1FFF7A10 + 8);

		printf("\r\nCPU : STM32F429BI, LQFP208, 主频: %dMHz\r\n", SystemCoreClock / 1000000);
		printf("UID = %08X %08X %08X\n\r", CPU_Sn2, CPU_Sn1, CPU_Sn0);
	}

	printf("\n\r");
	printf("*************************************************************\n\r");
	printf("* 例程名称   : %s\r\n", EXAMPLE_NAME);	/* 打印例程名称 */
	printf("* 例程版本   : %s\r\n", DEMO_VER);		/* 打印例程版本 */
	printf("* 发布日期   : %s\r\n", EXAMPLE_DATE);	/* 打印例程日期 */

	/* 打印ST固件库版本，这3个定义宏在stm32f10x.h文件中 */
	printf("* 固件库版本 : V%d.%d.%d (STM32F4xx_StdPeriph_Driver)\r\n", __STM32F4XX_STDPERIPH_VERSION_MAIN,
			__STM32F4XX_STDPERIPH_VERSION_SUB1,__STM32F4XX_STDPERIPH_VERSION_SUB2);
	printf("* \r\n");	/* 打印一行空格 */
	printf("* QQ    : 1295744630 \r\n");
	printf("* 旺旺  : armfly\r\n");
	printf("* Email : armfly@qq.com \r\n");
	printf("* 淘宝店: armfly.taobao.com\r\n");
	printf("* Copyright www.armfly.com 安富莱电子\r\n");
	printf("*************************************************************\n\r");
}

/*
*********************************************************************************************************
*	函 数 名: DispMenu
*	功能说明: 打印例程结果
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void DispMenu(void)
{	
	uint8_t i;
	
	printf("\n\r");
	printf("\33[K");						/* 清除从光标到行尾的内容 */ 
	
	printf(" 发送的命令 : 0x");				/* 打印发送命令 */
	for (i = 0; i < g_tPrint.Txlen; i++)
	{
		printf(" %02X", g_tPrint.TxBuf[i]);
	}

	printf("\n\r");
	printf("\33[K");						/* 清除从光标到行尾的内容 */ 
	
	printf(" 接收的命令 : 0x");				/* 打印接收命令 */
	for (i = 0; i < g_tPrint.Rxlen; i++)
	{
		printf(" %02X", g_tPrint.RxBuf[i]);
	}
	
	printf("\n\r");
	printf("\33[3A");						/* 光标上移3行 */
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
