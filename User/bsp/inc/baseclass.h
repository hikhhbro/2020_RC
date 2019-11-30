/**
	|-------------------------------- Copyright |--------------------------------|
	|                                                                            |
	|                        (C) Copyright 2018,海康平头哥,                       |
	|         1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China        |
	|                         All Rights Reserved                                |
	|                                                                            |
	|          By(GCU The wold of team | 华南理工大学广州学院机器人野狼队)          |
	|                   https://github.com/GCUWildwolfteam                       |
	|----------------------------------------------------------------------------|
	|--FileName    : baseclass.h
	|--Version     : v1.0
	|--Author      : 海康平头哥
	|--Date        : 2018-11-27 
	|-- Libsupports:           
	|--Description :1.常用宏定义，枚举，函数等
	|--FunctionList
	|-------1. ....
	|          <version>:
	|     <modify staff>:
	|             <data>:
	|      <description>:
	|-------2. ...
	|-----------------------------declaration of end-----------------------------|
 **/
#ifndef __BASECLASS_H 
#define __BASECLASS_H 
#include "userdriverconfig.h"
/* -------------- 常用运算宏 ----------------- */
	#define MAX(X,Y) 							((X)>(Y)?(Y):(X))//限做大值
	#define MIN(X,Y) 							((X)<(Y)?(Y):(X))//限做小值
	#define ABS(X)   							((X)<0?(-X):(X))//取绝对值
  #define ARR_SIZE(a)  					(sizeof((a))/sizeof((a[0])))//返回数组元素的个数
  #define INC_SAT(val) (val=((val)+1>(val)) ? (val)+1 : (val))//防止溢出的一个方法
  #define HEXCHK(c) (((c) >= '0' && (c)<='9') ((c)>='A' && (c)<= 'F') \
                     ((c)>='a' && (c)<='f'))//判断字符是不是16进值的数
  #define  DECCHK(c) ((c)>='0' && (c)<='9')//判断字符是不是10进值的数字
  #define  RND8(x)						((((x) + 7)/8) * 8)//返回一个比X大的最接近的8的倍数
  #define  WORD_LO(xxx)       ((byte) ((word)(xxx) & 255))//得到一个字低位字节
  #define  WORD_HI(xxx)         ((byte) ((word)(xxx) >> 8))//到一个字的高位和
	/*得到一个结构体中field所占用的字节数*/		
  #define  FSIZ(type,field)     sizeof(((type *)0)->field)
	/*得到一个field在结构体(struct)中的偏移量*/	
  #define  FPOS(type,field)     ((dword)&((type *)0)->field)
  #define  MEM_B(x)             (*((byte *)(x)))// 得到指定地址上的一个字节
  #define  MEM_W(x)             (*((word *)(x)))//得到指定地址上的一个字
  #define  IS_CLOSE_INT(x,a,b)  (x>=a && x<=b)//判断是否在一个闭区间内
  #define  IS_OPEN_INT(x,a,b)   (x>a && x<b)//判断是否在一个闭区间内
  #define  LEFT_SH(_a,_x)            (_a <<_x)//左移_x位
  #define  RIGHT_SH(_a,_x)           (_a >>_x)//右移_x位
  #define  GET_BYTE(_a,_x)           (_a & _x)//获取_x位
  #define  IS_BYTE(_a,_x)            ((_a & _x) ==_x)//判断对应的位是否相等
	#define	CYCLE_NUMERICAL(data,max) ((data+1)%max)//自增循环
  #define PI (3.14159f) //π
/* ----------------- 运行状态位 宏 -------------------- */
/* uint32_t status(0代表最低地位，31代表最高位)（为了方便添加其他标志位）
 * -------------------------------------------------
 *任务数 |   0bit   |   1bit    |   2bit   |    3bit    | 7~3bit |
 *据初始--------------------------------------------
 *化位   | 初始化成功| 正常运行 | 接受正常 |启动设置成功| 保留  |
 * ----------------------------------------------------------------
 * 外设| 8bit | 9bit | 10bit  | 11bit  | 12bit  | 13bit | 14~15bit | 
 * 初始-------------------------------------------------------------
 * 化位| can1 | CAN2 | UASRT1 | UASRT3 | UASRT6 | UART7 |   保留    | 
 * -----------------------------------------------------------------
 * |        24~31bit      | 
 * -------------------------
 * |  保留(模块自定义位)  | 
 * -------------------------
 */
	#define MOD_READ                    0x00000000U//模块准备就绪
	#define INIT_OK                     0x00000001U//初始化成功
  #define RUNING_OK                   0x00000002U//正常运行
  #define RX_OK                       0x00000004U//接受正常
  #define START_OK                    0x00000008U//启动设置成功
  #define DHECK_DATA_RUNING           0x00000010U//数据校准任务运行   
/* -------------- 运行状态设置 ----------------- */
  #define IS_INIT_OK(_status_)      IS_BYTE(_status_,INIT_OK)
  #define IS_RUNING_OK(_status_)    IS_BYTE(_status_,RUNING_OK)
  #define IS_RX_OK(_status_)        IS_BYTE(_status_,RX_OK)
  #define IS_START_OK(_status_)     IS_BYTE(_status_,START_OK)
	/* --这两个宏一定要一起用，缓存之后一定要释放 -- */
	#define CACHE_ADDR(CACHES,ADDRS) 	(CACHES = ADDRS) //缓存地址
	#define FREE_ADDR(CACHES) 	      (CACHES = NULL)		//释放地址  
/* -------------- 类型转换联合体 ----------------- */
typedef union  
{   
  float  f;   
  int16_t s_16[2];
  uint16_t u_16[2];
  unsigned char u_8[4];
   uint32_t u_32; 
   int32_t s_32;  
}floatToUnion; 
/* -------------- 系统枚举 ----------------- */
	typedef enum
	{
		SYS_OK       = 0x00U,
		SYS_ERROR    = 0x01U,
		SYS_BUSY     = 0x02U,
		SYS_LOST     = 0x03U
	}SYS_Status;
/* -------------- 模块状态枚举 ----------------- */
typedef enum 
{
  MOD_OK       = 0x00U,
  MOD_ERROR    = 0x01U,
  MOD_BUSY     = 0x02U,
  MOD_LOST     = 0x03U
}MOD_Status;
/* -------------- 函数定义 ----------------- */
	void* Get_Peripheral_type(void* Peripheral);
	void MultibyteToByle(uint32_t data,uint8_t *pdata);
	void ByleToMultibyte(uint8_t *pdata,uint32_t *data);
	void TwobyteToByle(int16_t data,uint8_t *pdata);
#endif	// __BASECLASS_H
	
 /*--------------------------------file of end--------------------------------*/

