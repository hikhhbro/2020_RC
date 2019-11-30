/**
	|------------------------------- Copyright ----------------------------------|
	|                                                                            |
	|                       (C) Copyright 2019,海康平头哥,                        |
	|          1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China       |
	|                            All Rights Reserved                             |
	|                                                                            |
	|          By(GCU The wold of team | 华南理工大学广州学院机器人野狼队)          |
	|                     https://github.com/GCUWildwolfteam                     |
	|----------------------------------------------------------------------------|
	|--FileName    : DJI_dbus.c                                                
	|--Version     : v1.0                                                            
	|--Author      : 海康平头哥                                                       
	|--Date        : 2019-01-15               
	|--Libsupports : 
	|--Description : 1.调用DJIDbusInit初始化遥控
  |                2.在接收任务中调用 DbusParseData 解析数据 
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|-------------------------------declaration of end---------------------------|
 **/
#include "DJI_dbus.h"        
#define RC_DATA_RX_LEN   (18 + HEAD_FRAME_LEN)
uint8_t databuff[RC_DATA_RX_LEN];//数据接收
static dbusStruct lostdata;
/**
	* @Data    2019-01-15 20:24
	* @brief   大疆遥控初始化
	* @param   dbusStruct* dbs 遥控结构体指针
	* @param   huartx(x=1,2,3,6)
	* @retval  HAL Status
	*/
	HAL_StatusTypeDef DJIDbusInit(dbusStruct* dbs,UART_HandleTypeDef *huartx)
	{
		if(dbs == NULL)
		{
			return HAL_ERROR;
		}
		dbs->ch1 = 0;
		dbs->ch2 = 0;
		dbs->ch3 = 0;
		dbs->ch4 = 0;
		dbs->switch_left = 0;
		dbs->switch_right = 0;
		dbs->mouse.x = 0;
		dbs->mouse.y=0;
		dbs->mouse.z=0;
		dbs->mouse.press_left=0;
		dbs->mouse.press_right=0;
		dbs->mouse.jumppress_left=0;
		dbs->mouse.jumppress_right=0;
		dbs->keyBoard.key_code = 0;
		dbs->keyBoard.jumpkey_code = 0;
		dbs->state_flag = 0;//状态标志位
		dbs->a_frame_len = RC_DATA_RX_LEN;//一帧数据长度  2(数据长度) +18（一帧数据)
		dbs->huartx = huartx;
		/* -------- 初始化开启串口数据接收 --------- */
		if(UsartAndDMAInit(huartx, dbs->a_frame_len,ENABLE) != HAL_OK)
		{
			dbs->status = 0;
			return HAL_ERROR;
		}
    dbs->fps[FPS_ADDR]= 1;
    SetFpsAddress(dbs->fps);
		SET_BIT(dbs->status,INIT_OK);
		return HAL_OK;
	}
/**
	* @Data    2019-01-16 15:07
	* @brief   数据解析
	* @param   void
	* @retval  void
	*/
	void DbusParseData(dbusStruct* dbs)
	{
		if(UserUsartQueueRX(dbs->huartx,databuff) != DATA_EMPTY)
		{
		dbs->ch1 = (databuff[GET_DATA_ADDR(0)] | \
                LEFT_SH(databuff[GET_DATA_ADDR(1)],8)) & 0x07FF;
		dbs->ch1 -= 1024;
    dbs->ch1 = DbusAntiShake(20,dbs->ch1);
		dbs->ch2 = (RIGHT_SH(databuff[GET_DATA_ADDR(1)],3)| \
                LEFT_SH(databuff[GET_DATA_ADDR(2)],5)) & 0x07FF;
		dbs->ch2 -= 1024;
    dbs->ch2 = DbusAntiShake(20,dbs->ch2);
		dbs->ch3 = (RIGHT_SH(databuff[GET_DATA_ADDR(2)],6)| \
                LEFT_SH(databuff[GET_DATA_ADDR(3)],2) | \
                LEFT_SH(databuff[GET_DATA_ADDR(4)],10)) & 0x07FF;
		dbs->ch3 -= 1024;
    dbs->ch3 = DbusAntiShake(20,dbs->ch3);
		dbs->ch4 = (RIGHT_SH(databuff[GET_DATA_ADDR(4)],1) | \
                LEFT_SH(databuff[GET_DATA_ADDR(5)],7)) & 0x07FF;		
		dbs->ch4 -= 1024;
    dbs->ch4 = DbusAntiShake(20,dbs->ch4);
		
		dbs->switch_left = (RIGHT_SH(databuff[GET_DATA_ADDR(5)],4) & 0x000C ) >> 2;
		dbs->switch_right = RIGHT_SH(databuff[GET_DATA_ADDR(5)],4)& 0x0003 ;
		
		dbs->mouse.x = databuff[GET_DATA_ADDR(6)] | (databuff[GET_DATA_ADDR(7)] << 8);
		dbs->mouse.y = databuff[GET_DATA_ADDR(8)] | (databuff[GET_DATA_ADDR(9)] << 8);
		dbs->mouse.z = databuff[GET_DATA_ADDR(10)] | (databuff[GET_DATA_ADDR(10)] << 8);
		
		dbs->mouse.press_left 	= databuff[GET_DATA_ADDR(12)];	// is pressed?
		dbs->mouse.press_right 	= databuff[GET_DATA_ADDR(13)];
		
		dbs->keyBoard.key_code 	= databuff[GET_DATA_ADDR(14)] | \
                              databuff[GET_DATA_ADDR(15)] << 8; //key broad code
    dbs->thumbwheel =((int16_t)databuff[GET_DATA_ADDR(16)] | \
                     ((int16_t)databuff[GET_DATA_ADDR(17)] << 8)) & 0x07FF;
     dbs->thumbwheel -= 1024;
     Fps(dbs->fps);
    SET_BIT(dbs->status,RX_OK);
     lostdata = *dbs;
		}
		else
		{
      if(GetFps(dbs->fps) ==0)
      {
        dbs->ch1 = 0;
        dbs->ch2 = 0;
        dbs->ch3 = 0;
        dbs->ch4 = 0;
       CLEAR_BIT(dbs->status,RX_OK);
      }
      else *dbs = lostdata;  
		}
	}
	/**
		* @Data    2019-02-14 21:06
		* @brief   遥控防抖
		* @param   int16_t range 防抖范围
		* @param 	 int16_t *data 指向数据地址的指针
		* @retval  void
		*/
		int16_t DbusAntiShake(int16_t range,int16_t data)
		{
			if(data > -(ABS(range)) && data < (ABS(range)))
				data = 0;
			return data;
		} 
/**
	* @Data    2019-09-10 19:44
	* @brief   遥控键值定义 
	* @param   void
	* @retval  void
	* @note    重定义的时候要先初始化结构体redifRcStruct
	*/
	SYS_Status DefineRcKey(redifRcStruct* pRdK_t)
	{
    if(pRdK_t==NULL || pRdK_t->pRc_t==NULL)
      return SYS_ERROR;
		if(pRdK_t->ch1 != NULL)
			*pRdK_t->ch1 = pRdK_t->pRc_t->ch1;
		if(pRdK_t->ch2 != NULL)
    	*pRdK_t->ch2 = pRdK_t->pRc_t->ch2;
		if(pRdK_t->ch3 != NULL)	
    	*pRdK_t->ch3 = pRdK_t->pRc_t->ch3;
		if(pRdK_t->ch4 != NULL)	
    	*pRdK_t->ch4 = pRdK_t->pRc_t->ch4;
		if(pRdK_t->switch_left != NULL)	
    	*pRdK_t->switch_left = pRdK_t->pRc_t->switch_left;
		if(pRdK_t->switch_right != NULL)		
      *pRdK_t->switch_right = pRdK_t->pRc_t->switch_right;
		if(pRdK_t->thumbwheel != NULL)	
    	*pRdK_t->thumbwheel = pRdK_t->pRc_t->thumbwheel;  
      return SYS_OK;
	}
	/*----------------------------------file of end-----------------------------*/
