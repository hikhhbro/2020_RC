/**
	|----------------------------- Copyright ------------------------------------|
	|                                                                            |
	|                        (C) Copyright 2018,海康平头哥,                       |
	|        1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China         |
	|                         All Rights Reserved                                |
	|                                                                            |
	|         By(GCU The wold of team | 华南理工大学广州学院机器人野狼队)           |
	|                  https://github.com/GCUWildwolfteam                        |
	|----------------------------------------------------------------------------|
	|--FileName    : baseclass.c                                                
	|--Version     : v1.0                                                            
	|--Author      : 海康平头哥                                                       
	|--Date        : 2018-12-30               
	|--Libsupports : 
	|--Description :                                                       
	|--FunctionList                                                       
	|-------1. ....                                                       
	|          <version>:                                                       
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|------------------------------declaration of end----------------------------|
 **/
#include "baseclass.h" 
	/**
	* @Data    2018-12-30 17:23
	* @brief   获取外设类型指针
	* @param   void* 
	* @retval  void*
	*/
	void* Get_Peripheral_type(void* Peripheral)
	{
		return Peripheral;
	}
	/**
		* @Data    2019-02-14 15:09
		* @brief   四字节类型转单字节类型,高位在前
		* @param   uint32_t 四个字节
		* @param	 uint8_t *单字节指针 
		* @retval  void
		*/
		void MultibyteToByle(uint32_t data,uint8_t *pdata)
		{
			unsigned char *p;
			p = (unsigned char *)&data;
			*pdata = (unsigned char)*(p + 3);//高八位
			*(pdata+1)= (unsigned char)*(p + 2);
			*(pdata + 2) = (unsigned char)*(p + 1);
			*(pdata + 3) = (unsigned char)*p;//低八位
		}
		/**
		* @Data    2019-02-14 15:09
		* @brief   两个字节类型转单字节类型,高位在前
		* @param   uint32_t 四个字节
		* @param	 uint8_t *单字节指针 
		* @retval  void
		*/
		void TwobyteToByle(int16_t data,uint8_t *pdata)
		{
			unsigned char *p;
			p = (unsigned char *)&data;
			*pdata = (unsigned char)*(p + 1);//高八位
			*(pdata+1)= (unsigned char)*p;//低八位
		}
	/**
		* @Data    2019-02-14 15:45
		* @brief   单字节数组转四个字节，高位在前
		* @param	 uint8_t *单字节指针 
		* @param   uint32_t *四个字节指针
		* @retval  void
		*/
		void ByleToMultibyte(uint8_t *pdata,uint32_t *data)
		{
			*data = (uint32_t)((*pdata << 24) | (*(pdata + 1) << 16) | \
							(*(pdata + 2) << 8) |(*(pdata + 3)));
		}
/*--------------------------------file of end-------------------------------*/
