/*
*********************************************************************************************************
*
*	模块名称 : MODSBUS通信程序 （主机）
*	文件名称 : modbus_host.c
*	版    本 : V1.4
*	说    明 : 无线通信程序。通信协议基于MODBUS
*	修改记录 :
*		版本号  日期        作者    说明
*       V1.4   2015-11-28 修改协议
*
*	Copyright (C), 2015-2016, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "modbus_host.h"
#include "usart.h"
#include "tim.h"
#include "string.h"
#define TIMEOUT		500		/* 接收命令超时时间, 单位ms */
#define NUM			1		/* 循环发送次数 */

/* 保存每个从机的计数器值 */

MODH_T g_tModH;
uint8_t g_modh_timeout = 0;
u8 Bytes;
extern u16 receive[64];

//static void MODH_RxTimeOut(void);
static void MODH_AnalyzeApp(void);


static void MODH_Read_03H(void);

static void MODH_Read_10H(void);

VAR_T g_tVar;


// CRC 高位字节值表
static const uint8_t s_CRCHi[] =
{
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;
// CRC 低位字节值表
const uint8_t s_CRCLo[] =
{
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};


/*
	全局运行时间，单位1ms
	最长可以表示 24.85天，如果你的产品连续运行时间超过这个数，则必须考虑溢出问题
*/
__IO int32_t g_iRunTime = 0;
/*
*********************************************************************************************************
*	函 数 名: CRC16_Modbus
*	功能说明: 计算CRC。 用于Modbus协议。
*	形    参: _pBuf : 参与校验的数据
*			  _usLen : 数据长度
*	返 回 值: 16位整数值。 对于Modbus ，此结果高字节先传送，低字节后传送。
*
*   所有可能的CRC值都被预装在两个数组当中，当计算报文内容时可以简单的索引即可；
*   一个数组包含有16位CRC域的所有256个可能的高位字节，另一个数组含有低位字节的值；
*   这种索引访问CRC的方式提供了比对报文缓冲区的每一个新字符都计算新的CRC更快的方法；
*
*  注意：此程序内部执行高/低CRC字节的交换。此函数返回的是已经经过交换的CRC值；也就是说，该函数的返回值可以直接放置
*        于报文用于发送；
*********************************************************************************************************
*/
uint16_t CRC16_Modbus(uint8_t* _pBuf, uint16_t _usLen)
{
    uint8_t ucCRCHi = 0xFF; /* 高CRC字节初始化 */
    uint8_t ucCRCLo = 0xFF; /* 低CRC 字节初始化 */
    uint16_t usIndex;  /* CRC循环中的索引 */

    while(_usLen--)
    {
        usIndex = ucCRCHi ^ *_pBuf++; /* 计算CRC */
        ucCRCHi = ucCRCLo ^ s_CRCHi[usIndex];
        ucCRCLo = s_CRCLo[usIndex];
    }
    return ((uint16_t)ucCRCHi << 8 | ucCRCLo);
}

/*
*********************************************************************************************************
*	函 数 名: BEBufToUint16
*	功能说明: 将2字节数组(大端Big Endian次序，高字节在前)转换为16位整数
*	形    参: _pBuf : 数组
*	返 回 值: 16位整数值
*
*   大端(Big Endian)与小端(Little Endian)
*********************************************************************************************************
*/
uint16_t BEBufToUint16(uint8_t* _pBuf)
{
    return (((uint16_t)_pBuf[0] << 8) | _pBuf[1]);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_GetRunTime
*	功能说明: 获取CPU运行时间，单位1ms。最长可以表示 24.85天，如果你的产品连续运行时间超过这个数，则必须考虑溢出问题
*	形    参:  无
*	返 回 值: CPU运行时间，单位1ms
*********************************************************************************************************
*/
int32_t bsp_GetRunTime(void)
{
    int32_t runtime;

//	DISABLE_INT();  	/* 关中断 */

    runtime = g_iRunTime;	/* 这个变量在Systick中断中被改写，因此需要关中断进行保护 */

//	ENABLE_INT();  		/* 开中断 */

    return runtime;
}
/*
*********************************************************************************************************
*	函 数 名: bsp_CheckRunTime
*	功能说明: 计算当前运行时间和给定时刻之间的差值。处理了计数器循环。
*	形    参:  _LastTime 上个时刻
*	返 回 值: 当前时间和过去时间的差值，单位1ms
*********************************************************************************************************
*/
int32_t bsp_CheckRunTime(int32_t _LastTime)
{
    int32_t now_time;
    int32_t time_diff;

//	DISABLE_INT();  	/* 关中断 */

    now_time = g_iRunTime;	/* 这个变量在Systick中断中被改写，因此需要关中断进行保护 */

//	ENABLE_INT();  		/* 开中断 */

    if(now_time >= _LastTime)
    {
        time_diff = now_time - _LastTime;
    }
    else
    {
        time_diff = 0x7FFFFFFF - _LastTime + now_time;
    }

    return time_diff;
}

void SysTick_ISR(void)
{

    /* 全局运行时间每1ms增1 */
    g_iRunTime++;
    if(g_iRunTime == 0x7FFFFFFF)	/* 这个变量是 int32_t 类型，最大数为 0x7FFFFFFF */
    {
        g_iRunTime = 0;
    }

}


///////////////////////////////////////////////////////////////////////////


/*
*********************************************************************************************************
*	函 数 名: MODH_SendPacket
*	功能说明: 发送数据包 COM1口
*	形    参: _buf : 数据缓冲区
*			  _len : 数据长度
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_SendPacket(uint8_t* _buf, uint16_t _len)
{
//	printf("hello\r\n");
    HAL_UART_Transmit(&huart4, _buf, _len, 0xFFFF);
}

/*
*********************************************************************************************************
*	函 数 名: MODH_SendAckWithCRC
*	功能说明: 发送应答,自动加CRC.
*	形    参: 无。发送数据在 g_tModH.TxBuf[], [g_tModH.TxCount
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_SendAckWithCRC(void)
{
    uint16_t crc;

    crc = CRC16_Modbus(g_tModH.TxBuf, g_tModH.TxCount);
    g_tModH.TxBuf[g_tModH.TxCount++] = crc >> 8;
    g_tModH.TxBuf[g_tModH.TxCount++] = crc;
    MODH_SendPacket(g_tModH.TxBuf, g_tModH.TxCount);

#if 0	/* 此部分为了串口打印结果,实际运用中可不要 */
    g_tPrint.Txlen = g_tModH.TxCount;
    memcpy(g_tPrint.TxBuf, g_tModH.TxBuf, g_tModH.TxCount);
#endif
}

/*
*********************************************************************************************************
*	函 数 名: MODH_AnalyzeApp
*	功能说明: 分析应用层协议。处理应答。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_AnalyzeApp(void)
{

    switch(g_tModH.RxBuf[1])			/* 第2个字节 功能码 */
    {
    case 0x01:	/* 读取线圈状态 */
        break;

    case 0x02:	/* 读取输入状态 */
        break;

    case 0x03:	/* 读取保持寄存器 在一个或多个保持寄存器中取得当前的二进制值 */
        MODH_Read_03H();
        break;

    case 0x04:	/* 读取输入寄存器 */
        break;

    case 0x05:	/* 强制单线圈 */
        break;

    case 0x06:	/* 写单个寄存器 */
        break;

    case 0x10:	/* 写多个寄存器 */
        MODH_Read_10H();
        break;

    default:
        break;
    }
}


/*
*********************************************************************************************************
*	函 数 名: MODH_Send03H
*	功能说明: 发送03H指令，查询1个或多个保持寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send03H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
    g_tModH.TxBuf[g_tModH.TxCount++] = 0x03;		/* 功能码 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */

    MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
    g_tModH.fAck03H = 0;		/* 清接收标志 */
    g_tModH.RegNum = _num;		/* 寄存器个数 */
    g_tModH.Reg03H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */
}


/*
*********************************************************************************************************
*	函 数 名: MODH_Send10H
*	功能说明: 发送10H指令，连续写多个保持寄存器. 最多一次支持23个寄存器。
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数n (每个寄存器2个字节) 值域
*			  _buf : n个寄存器的数据。长度 = 2 * n  一个寄存器两个字节
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send10H(uint8_t _addr, uint16_t _reg, uint8_t _num, uint8_t* _buf)
{
    uint16_t i;

    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
    g_tModH.TxBuf[g_tModH.TxCount++] = 0x10;		/* 功能码 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = 2 * _num;	/* 数据字节数 */

    for(i = 0; i < 2 * _num; i++)
    {
        if(g_tModH.TxCount > H_RX_BUF_SIZE - 3)
        {
            return;		/* 数据超过缓冲区超度，直接丢弃不发送 */
        }
        g_tModH.TxBuf[g_tModH.TxCount++] = _buf[i];		/* 后面的数据长度 */
    }
    g_tModH.fAck10H = 0;		/* 清接收标志 */

    MODH_SendAckWithCRC();	/* 发送数据，自动加CRC */
}

/*
*********************************************************************************************************
*	函 数 名: MODH_ReciveNew
*	功能说明: 串口接收中断服务程序会调用本函数。当收到一个字节时，执行一次本函数。
*	形    参:
*	返 回 值: 1 表示有数据
*********************************************************************************************************
*/
void MODH_ReciveNew(uint8_t _data)
{
    /*
    	3.5个字符的时间间隔，只是用在RTU模式下面，因为RTU模式没有开始符和结束符，
    	两个数据包之间只能靠时间间隔来区分，Modbus定义在不同的波特率下，间隔时间是不一样的，
    	所以就是3.5个字符的时间，波特率高，这个时间间隔就小，波特率低，这个时间间隔相应就大

    	4800  = 7.297ms
    	9600  = 3.646ms
    **	19200  = 1.771ms
    	38400  = 0.885ms
    */
    uint32_t timeout;

    g_modh_timeout = 0;

//	HAL_TIM_Base_Start_IT(&htim2);
    /* 硬件定时中断，定时精度us 硬件定时器2用于MODBUS从机, 定时器3用于MODBUS从机主机*/
    // 波特率19200 硬件定时1.771ms,定时器中断中调用MODH_RxTimeOut函数


    if(g_tModH.RxCount < H_RX_BUF_SIZE)
    {
        g_tModH.RxBuf[g_tModH.RxCount++] = _data;
    }
}

/*
*********************************************************************************************************
*	函 数 名: MODH_RxTimeOut
*	功能说明: 超过3.5个字符时间后执行本函数。 设置全局变量 g_rtu_timeout = 1; 通知主程序开始解码。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_RxTimeOut(void)
{
    g_modh_timeout = 1;
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Poll
*	功能说明: 接收控制器指令. 1ms 响应时间。
*	形    参: 无
*	返 回 值: 0 表示无数据 1表示收到正确命令
*********************************************************************************************************
*/
void MODH_Poll(void)
{
    uint16_t crc1;

    /* 计算CRC校验和 */
    crc1 = CRC16_Modbus(g_tModH.RxBuf, g_tModH.RxCount);
    if(crc1 != 0)
    {
		
    }
    else
        /* 分析应用层协议 */
        MODH_AnalyzeApp();

//err_ret:
#if 0	/* 此部分为了串口打印结果,实际运用中可不要 */
    g_tPrint.Rxlen = g_tModH.RxCount;
    memcpy(g_tPrint.RxBuf, g_tModH.RxBuf, g_tModH.RxCount);
#endif

}

/*
*********************************************************************************************************
*	函 数 名: MODH_Read_03H
*	功能说明: 分析03H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Read_03H(void)
{
    uint8_t bytes;
    uint8_t* p;
    if(g_tModH.RxCount > 0)
    {
        bytes = g_tModH.RxBuf[2];	/* 数据长度 字节数 */

            if(bytes == (Bytes-5))    // 数据字节数  5+num*2
            {
                p = &g_tModH.RxBuf[3];

                for(int i = 0; i < bytes/2; i++)  // 高低字节合并
                {
                    receive[i] = BEBufToUint16(p);
//					printf("%d",receive[i]);
                    p += 2;	/* 寄存器 */
                }

                g_tModH.fAck03H = 1;
            }
    }
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Read_10H
*	功能说明: 分析10H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Read_10H(void)
{
    /*
    	10H指令的应答:
    		从机地址                11
    		功能码                  10
    		寄存器起始地址高字节	00
    		寄存器起始地址低字节    01
    		寄存器数量高字节        00
    		寄存器数量低字节        02
    		CRC校验高字节           12
    		CRC校验低字节           98
    */
    if(g_tModH.RxCount > 0)
    {
        if(g_tModH.RxBuf[0] == SlaveAddr)
        {
            g_tModH.fAck10H = 1;		/* 接收到应答 */
        }
    }
}

/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_03H
*	功能说明: 单个参数. 通过发送03H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_03H(uint16_t _reg, uint16_t _num)
{
    int32_t time1;
    uint8_t i;

    Bytes = 5 + _num * 2;
    for(i = 0; i < NUM; i++)
    {
        MODH_Send03H(SlaveAddr, _reg, _num);
        time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */

        while(1)
        {

            MODH_Poll();								// 查询是否接收正确

            if(bsp_CheckRunTime(time1) > TIMEOUT)		//超时检测
            {
                break;		/* 通信超时了 */
            }

            if(g_tModH.fAck03H > 0)					// 如果正确该位被置1
            {
                break;
            }
        }

    }

    // 超时或者接收完成
    g_tModH.RxCount = 0;	/* 必须清零计数器，方便下次帧同步 */
    memset(g_tModH.RxBuf,0,256);	// 接收缓冲区清0
	HAL_UART_Receive_IT(&huart4, &RXdata, 1);	 // 重新打开串口接收，我也不知道为什么他会自己关闭

    if(g_tModH.fAck03H == 0)
    {
        return 0;	/* 通信超时了 */
    }
    else
    {
        return 1;	/* 写入03H参数成功 */
    }
}

/*
*********************************************************************************************************
*	函 数 名: MODH_WriteParam_10H
*	功能说明: 单个参数. 通过发送10H指令实现，发送之后，等待从机应答。循环NUM次写命令
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_10H(uint16_t _reg, uint8_t _num, uint8_t* _buf)
{
    int32_t time1;
    uint8_t i;

    for(i = 0; i < NUM; i++)
    {
        MODH_Send10H(SlaveAddr, _reg, _num, _buf);
        time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */

        while(1)
        {
            MODH_Poll();

            if(bsp_CheckRunTime(time1) > TIMEOUT)
            {
                break;
            }

            if(g_tModH.fAck10H > 0)
            {
                break;
            }
        }

    }

	
    // 超时或者接收完成
    g_tModH.RxCount = 0;	/* 必须清零计数器，方便下次帧同步 */
    memset(g_tModH.RxBuf,0,256);	// 接收缓冲区清0
	
	HAL_UART_Receive_IT(&huart4, &RXdata, 1);	 // 重新打开串口接收，我也不知道为什么他会自己关闭
	
    if(g_tModH.fAck10H == 0)
    {
        return 0;	/* 通信超时了 */
    }
    else
    {
        return 1;	/* 写入10H参数成功 */
    }
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/

