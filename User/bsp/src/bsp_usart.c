/**
	|------------------------------ Copyright -----------------------------------|
	|                                                                            |
	|                       (C) Copyright 2019,海康平头哥,                        |
	|          1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China       |
	|                            All Rights Reserved                             |
	|                                         -                                  |
	|          By(GCU The wold of team | 华南理工大学广州学院机器人野狼队)          |
	|                    https://github.com/GCUWildwolfteam                      |
	|----------------------------------------------------------------------------|
	|--FileName    : bsp_usart.c                                                
	|--Version     : v1.0                                                            
	|--Author      : 海康平头哥                                                       
	|--Date        : 2019-01-15               
	|--Libsupports : 
	|--Description :1.cubemx配置串口和相应的DMA，开启相应的中断
  |               2.调用UsartAndDMAInit初始化串口相应的配置
                  3.UserUsartCallback 在stm32f4xx_it.c 中的相应中断函数中调用
                  4.UserUsartQueueRX 在需要接受数据的模块调用接受
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|------------------------------declaration of end----------------------------|
 **/
#include "bsp_usart.h"
#define USART_NUM 9
/* ----------------- 结构体地址列表 -------------------- */
usartDataStrcut *pusart_t[USART_NUM] = {NULL};
/**
	* @Data    2019-01-15 15:31
	* @brief   串口初始化
	* @param	 usartx(x=1~8)
	* @param   uint8_t frame_size 一帧的长度
	* @param   on_off   DISABLE = 0U, ENABLE = 1U //初始化开关
	* @retval  SYS_Status
	*/
SYS_Status UsartAndDMAInit(UART_HandleTypeDef *huartx,uint8_t frame_size\
																															,uint8_t on_off)
{
	usartDataStrcut *addr;
	/* -------- 对外接口数据获取 --------- */
	if(AllocateUsartxSpace(huartx) != HAL_OK)//用户串口结构体空间分配
	{
		free(addr);
		return SYS_ERROR;
	}
	addr = GetUsartAddr(huartx); //获取相应用户串口结构体地址
  if(addr == NULL)
    return SYS_ERROR;
	addr->usart_queue = NULL; 
	addr->rx_buff_size = frame_size; //得到一帧的长度
	addr->rx_on_off = on_off;//接收开关
	/* ------ 分配接收缓存地址空间 ------- */
	addr->rx_buff_data = (uint8_t *)malloc(addr->rx_buff_size * sizeof(uint8_t));
  if(addr->rx_buff_data == NULL)
    return SYS_ERROR; 
	if(UsartQueueCreate(addr,5,addr->rx_buff_size) != HAL_OK) //创建队列
	{
		free(addr->rx_buff_data);
		free(addr);
		return SYS_ERROR;
	}
  	/* -------- 使能 --------- */
  HAL_UART_Receive_DMA(huartx,addr->rx_buff_data,addr->rx_buff_size);
	__HAL_UART_ENABLE_IT(huartx, UART_IT_IDLE);					 //使能串口中断
	return SYS_OK;
}
/**
	* @Data    2019-01-15 18:58
	* @brief   串口用户中断回调函数
	* @param   huartx(x=1~8) 用户串口结构体地址
	* @retval  void
	*/
	void UserUsartCallback(UART_HandleTypeDef *huartx)
	{
		NoLenRXforUsart(huartx);
	}
/**
	* @Data    2019-01-15 19:07
	* @brief   任意字节接收
	* @param   huartx（1~8） 用户串口结构体地址
	* @retval SYS_Status
	*/
SYS_Status NoLenRXforUsart(UART_HandleTypeDef *huartx)
{
  uint32_t temp; 
  usartDataStrcut *addr = NULL;
  floatToUnion p;
  addr = GetUsartAddr(huartx); //获取相应用户串口结构体地址
  if(addr == NULL || huartx ==NULL)
    return SYS_ERROR;
  if((__HAL_UART_GET_FLAG(huartx,UART_FLAG_IDLE) != RESET))  
  {
    __HAL_UART_CLEAR_IDLEFLAG(huartx);
    HAL_UART_DMAStop(huartx);  
    temp = huartx->hdmarx->Instance->NDTR; 
    addr->datalen = addr->rx_buff_size-temp;
    p.u_16[0] = addr->datalen;
    addr->rx_buff_data[DATA_LEN_BYTE_HIGH_8] = p.u_8[DATA_LEN_BYTE_HIGH_8];
    addr->rx_buff_data[DATA_LEN_BYTE_LOW_8]  = p.u_8[DATA_LEN_BYTE_LOW_8];
    xQueueSendToBackFromISR(addr->usart_queue, addr->rx_buff_data,0);
    memset(addr->rx_buff_data,0,addr->rx_buff_size);
    HAL_UART_Receive_DMA(huartx,(addr->rx_buff_data + RX_HEAD_ADDR),addr->rx_buff_size);
  }
  return SYS_OK;
}
/**
	* @Data    2019-01-16 10:24
	* @brief   队列创建
	* @param   void
	* @retval  SYS_Status
	*/
	SYS_Status UsartQueueCreate(usartDataStrcut *usartx,uint8_t len,uint8_t deep)
	{
    if(usartx == NULL)
      return SYS_ERROR; 
		usartx->usart_queue = xQueueCreate(len,deep);//创建深度len长度21队列
		if(usartx->usart_queue == NULL)
			return SYS_ERROR;
		return SYS_OK;
	}
/**
	* @Data    2019-01-16 10:54
	* @brief   分配相应串口类型数据的空间
	* @param   huartx（1~8）
	* @retval  SYS_Status
	*/
SYS_Status AllocateUsartxSpace(UART_HandleTypeDef *huartx)
{
	if (huartx->Instance == USART1) 
	{
		pusart_t[1]	= (struct usartDataStrcut*)malloc(sizeof(struct usartDataStrcut));
		if(pusart_t[1] == NULL)
		return SYS_ERROR;
		return SYS_OK;
	} 
	else if (huartx->Instance == USART2) 
	{
		pusart_t[2]	= (struct usartDataStrcut*)malloc(sizeof(struct usartDataStrcut));
		if(pusart_t[2]== NULL)
		return SYS_ERROR;
		return SYS_OK;
	} 
	else if (huartx->Instance == USART3) 
	{
		pusart_t[3]	= (struct usartDataStrcut*)malloc(sizeof(struct usartDataStrcut));
		if(pusart_t[3] == NULL)
		return SYS_ERROR;
		return SYS_OK;
	} 
  else if (huartx->Instance == UART4) 
	{
		pusart_t[4]	= (struct usartDataStrcut*)malloc(sizeof(struct usartDataStrcut));
		if(pusart_t[4] == NULL)
		return SYS_ERROR;
		return SYS_OK;
	} 
  else if (huartx->Instance == UART5) 
	{
		pusart_t[5]	= (struct usartDataStrcut*)malloc(sizeof(struct usartDataStrcut));
		if(pusart_t[5] == NULL)
		return SYS_ERROR;
		return SYS_OK;
	} 
	else if (huartx->Instance == USART6) 
	{
		pusart_t[6]	= (struct usartDataStrcut*)malloc(sizeof(struct usartDataStrcut));
		if(pusart_t[6] == NULL)
		return SYS_ERROR;
		return SYS_OK;
	}
  else if(huartx->Instance == UART7)
  {
    pusart_t[7]	= (struct usartDataStrcut*)malloc(sizeof(struct usartDataStrcut));
		if(pusart_t[7] == NULL)
		return SYS_ERROR;
		return SYS_OK;
  }
   else if(huartx->Instance == UART8)
  {
    pusart_t[8]	= (struct usartDataStrcut*)malloc(sizeof(struct usartDataStrcut));
		if(pusart_t[8] == NULL)
		return SYS_ERROR;
		return SYS_OK;
  }
	else	return SYS_ERROR;
}
/**
	* @Data    2019-01-16 11:08
	* @brief   自动判别串口类型获取相应用户串口结构体地址
	* @param   huartx（1~8）
	* @retval  usartDataStrcut* 用户串口结构体指针
	*/
	usartDataStrcut* GetUsartAddr(UART_HandleTypeDef *huartx)
	{
		if (huartx->Instance == USART1) 
			return pusart_t[1];
		else if (huartx->Instance == USART2) 
			return pusart_t[2];
		else if (huartx->Instance == USART3) 
			return pusart_t[3];
		else if (huartx->Instance == UART4) 
			return pusart_t[4];
		else if (huartx->Instance == UART5)
			return pusart_t[5];
		else if (huartx->Instance == USART6)  
			return pusart_t[6];
    else if(huartx->Instance == UART7) 
			return pusart_t[7];
    else if(huartx->Instance == UART8) 
			return pusart_t[8];
		else	return NULL;
	}

/**
	* @Data    2019-01-16 15:22
	* @brief   队列接收
	* @param   huartx（1~8）
	* @param 	 pvBuffer 接收数据地址
	* @retval  一帧数据接收的真实长度，无数据时，返回空EMPTY
	*/
	uint16_t UserUsartQueueRX(UART_HandleTypeDef *huartx,uint8_t* pvBuffer)
	{
    portBASE_TYPE xStatus;
    usartDataStrcut *addr = NULL;
    addr = GetUsartAddr(huartx);
    if(addr == NULL || pvBuffer == NULL)
      return DATA_EMPTY;
    xStatus = xQueueReceive(addr->usart_queue, pvBuffer, 0);
    if(pdFAIL != xStatus)
      return (LEFT_SH(pvBuffer[DATA_LEN_BYTE_HIGH_8],8) |pvBuffer[DATA_LEN_BYTE_LOW_8]);
    return DATA_EMPTY;
	}
/*-----------------------------------file of end------------------------------*/
