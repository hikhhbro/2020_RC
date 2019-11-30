/**
	|-------------------------------- Copyright ---------------------------------|
	|                                                                            |
	|                        (C) Copyright 2019,海康平头哥,                       |
	|           1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China      |
	|                          All Rights Reserved                               |
	|                                                                            |
	|           By(GCU The wold of team | 华南理工大学广州学院机器人野狼队)         |
	|                    https://github.com/GCUWildwolfteam                      |
	|----------------------------------------------------------------------------|
	|--FileName    : bsp_can.c                                                
	|--Version     : v1.0                                                            
	|--Author      : 海康平头哥                                                       
	|--Date        : 2019-01-18               
	|--Libsupports : STM32CubeF1 Firmware Package V1.6.0(用别的库出问题别问我)
	|--Description :                                                       
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|--------------------------------declaration of end--------------------------|
 **/
#include "bsp_can.h" 
/* ----------------- 结构体地址列表 -------------------- */
canDataStrcut *pcan_t[3] = {NULL};
/**
	* @Data    2019-01-26 12:27
	* @brief   can配置初始化
	* @param   void
	* @retval SYS_Status
	*/
SYS_Status UserCanConfig(void)   
{
  CAN_HandleTypeDef *hcanx;
  /*如果can1没被配置，就配置can1*/
  if(pcan_t[1] == NULL && BSP_CAN1 != NULL)
    hcanx = BSP_CAN1;
  /*如果can1已被配置，就配置can2*/
  else if(pcan_t[1] != NULL && pcan_t[2] == NULL && BSP_CAN2 != NULL)
    hcanx = BSP_CAN2;
  else return SYS_ERROR; 
	if(AllocateCanxSpace(hcanx) != HAL_OK)//用户can结构体空间分配
    return SYS_ERROR;
	else if(CanFilterInit(hcanx) != HAL_OK)//邮箱配置
    return SYS_ERROR;
	else if(CanTxInit(hcanx) != HAL_OK)//发送配置
		return SYS_ERROR;
	else if(CanRxInit(hcanx) != HAL_OK)//接收配置
		return SYS_ERROR;
  else
  {
    HAL_CAN_Start(hcanx);//启动
    HAL_CAN_ActivateNotification(hcanx, CAN_IT_RX_FIFO0_MSG_PENDING); //开启中断
    return SYS_OK;   
  }

}
/**
	* @Data    2019-01-26 12:28
	* @brief   can邮箱配置
	* @param   CAN_HandleTypeDef *hcanx(x=1,2)
	* @retval   HAL Status
	*/
	
SYS_Status CanFilterInit(CAN_HandleTypeDef* hcanx)
{
	canDataStrcut *addr;
	addr = GetCanAddr(hcanx);
	addr->filter.FilterIdHigh = 0x0000;//要过滤的ID高位
	addr->filter.FilterIdLow = 0x0000;//要过滤的ID低位
	addr->filter.FilterMaskIdHigh = 0x0000;//过滤器高16位每位必须匹配
	addr->filter.FilterMaskIdLow = 0x0000;//过滤器低16位每位必须匹配
	addr->filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;//过滤器被关联到FIFO 0
	addr->filter.FilterScale = CAN_FILTERSCALE_32BIT;
	addr->filter.FilterMode = CAN_FILTERMODE_IDMASK;
	addr->filter.FilterActivation = ENABLE;//使能过滤器
  if(hcanx->Instance ==CAN1)
  {
    addr->filter.FilterBank = 0;
    addr->filter.SlaveStartFilterBank = 0;
  }
  else if(hcanx->Instance ==CAN2)
  {
    addr->filter.FilterBank = 14;
    addr->filter.SlaveStartFilterBank = 14;
  }
  if(HAL_CAN_ConfigFilter(hcanx, &addr->filter) != HAL_ERROR)//使能筛选器
    return SYS_ERROR;
	else return SYS_OK; 
}
/**
	* @Data    2019-01-26 12:28
	* @brief   can 发送结构体初始化
	* @param   CanTxMsgTypeDef* TxMsg
	* @retval  HAL Status
	*/
SYS_Status CanTxInit(CAN_HandleTypeDef* hcanx)
{
	canDataStrcut *addr;
	addr = GetCanAddr(hcanx);//获取相应用户can结构体地址
	if(addr == NULL)
	{
		return SYS_ERROR;
	}
	addr->txMsg.StdId = 0x000; //设置id
	addr->txMsg.IDE = CAN_ID_STD; //选择标准id
	addr->txMsg.RTR = CAN_RTR_DATA; //0为数据帧，1为远程帧
	addr->txMsg.DLC = 8; //设置数据长度为8个字节
	return SYS_OK;
}
/**
	* @Data    2019-01-26 12:29
	* @brief   can 接受结构体初始化
	* @param   CanRxMsgTypeDef* RxMsg
	* @retval  void
	*/
SYS_Status CanRxInit(CAN_HandleTypeDef* hcanx)
{
		canDataStrcut *addr;
	  addr = GetCanAddr(hcanx); //获取相应用户can结构体地址
		if(addr == NULL)
		{
			return SYS_ERROR;
		}
    addr->rxMsg.StdId=0x00;
    addr->rxMsg.ExtId=0x00;
    addr->rxMsg.DLC=0x00;
    addr->rxdata[0] = 0x00; //接收数据位都清零
    addr->rxdata[1] = 0x00;
    addr->rxdata[2] = 0x00;
    addr->rxdata[3] = 0x00;
    addr->rxdata[4] = 0x00;
    addr->rxdata[5] = 0x00;
    addr->rxdata[6] = 0x00;
    addr->rxdata[7] = 0x00;
		return SYS_OK;
}
/**
	* @Data    2019-01-16 10:54
	* @brief   分配相应can类型数据的空间
	* @param   hcanx （x=1,2）
	* @retval  HAL Status
	*/
	SYS_Status AllocateCanxSpace(CAN_HandleTypeDef *hcanx)
	{
		if (hcanx->Instance == CAN1) 
		{
			pcan_t[1]	= (struct canDataStrcut*)malloc(sizeof(struct canDataStrcut));
			if(pcan_t[1] == NULL)
			return SYS_ERROR;
			return SYS_OK;
		} 
		else if (hcanx->Instance == CAN2) 
		{
			pcan_t[2]	= (struct canDataStrcut*)malloc(sizeof(struct canDataStrcut));
			if(pcan_t[2]== NULL)
			return SYS_ERROR;
			return SYS_OK;
		}
		else	return SYS_ERROR;
	}
/**
	* @Data    2019-01-16 11:08
	* @brief   自动判别can 类型获取相应用户can结构体地址
	* @param   hcanx （x=1,2）
	* @retval  canDataStrcut* 用户串口结构体指针
	*/
	canDataStrcut* GetCanAddr(CAN_HandleTypeDef *hcanx)
	{
		if(hcanx->Instance == CAN1)
		{
			return pcan_t[1];
		} 
		else if(hcanx->Instance == CAN2)  
		{
			return pcan_t[2];
		} 
		else	return NULL;
	}
/**
	* @Data    2019-01-19 00:58
	* @brief   can接受回调函数
	* @param   void
	* @retval  void
	*/
	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
	{
		canDataStrcut *addr;
	  addr = GetCanAddr(hcan); //获取相应用户can结构体地址
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&addr->rxMsg,addr->rxdata);
    if(hcan->Instance ==CAN1)
      can1_rx(addr->rxMsg.StdId,addr->rxdata);
    else if(hcan->Instance ==CAN2)
      can2_rx(addr->rxMsg.StdId,addr->rxdata);
	}
/**
	* @Data    2019-02-15 13:40
	* @brief   can发送函数
	* @param   void
	* @retval  void
	*/
 SYS_Status CanTxMsg(CAN_HandleTypeDef* hcanx,int id,uint8_t *message)
 {
  canDataStrcut *addr = NULL;
  addr = GetCanAddr(hcanx); //查找对应的私有结构体
  if(addr == NULL)	
    return SYS_ERROR;														      										
  addr->txMsg.StdId = id; //设置id
  addr->txMsg.IDE = CAN_ID_STD; //选择标准id
  addr->txMsg.RTR = CAN_RTR_DATA; //0为数据帧，1为远程帧
  addr->txMsg.DLC = 8; //设置数据长度为8个字节
  HAL_CAN_AddTxMessage(hcanx,&addr->txMsg,message,(uint32_t*)CAN_TX_MAILBOX0);
  return SYS_ERROR;
}
/**
	* @Data    2019-02-15 13:40
	* @brief   can1接收数据处理回调
	* @param   uint32_t id can接收id
  * @param   uint8_t *data 接收缓存地址
	* @retval  void
	*/		       
__weak void can1_rx(uint32_t id,uint8_t *data)
{
   (void)id;
   (void)data;
}
/**
	* @Data    2019-02-15 13:40
	* @brief   can2接收数据处理回调
	* @param   uint32_t id can接收id
  * @param   uint8_t *data 接收缓存地址
	* @retval  void
	*/	
__weak void can2_rx(uint32_t id,uint8_t *data)
{
   (void)id;
   (void)data;  
}
/*-----------------------------------file of end------------------------------*/


