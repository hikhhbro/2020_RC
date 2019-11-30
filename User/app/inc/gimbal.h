/**
	|--------------------------------- Copyright --------------------------------|
	|                                                                            |
	|                      (C) Copyright 2019,海康平头哥,                         |
	|           1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China      |
	|                           All Rights Reserved                              |
	|                                                                            |
	|           By(GCU The wold of team | 华南理工大学广州学院机器人野狼队)         |
	|                    https://github.com/GCUWildwolfteam                      |
	|----------------------------------------------------------------------------|
	|--FileName    : gimbal.h                                              
	|--Version     : v1.0                                                          
	|--Author      : 海康平头哥                                                     
	|--Date        : 2019-11-30             
	|--Libsupports : 
	|--Description :                                                     
	|--FunctionList                                                     
	|-------1. ....                                                     
	|          <version>:                                                     
	|     <modify staff>:                                                       
	|             <data>:                                                       
	|      <description>:                                                        
	|-------2. ...                                                       
	|-----------------------------declaration of end-----------------------------|
 **/
#ifndef __GIMBAL_H 
#define __GIMBAL_H 
#include "Servo.h" 
#include "DJI_dbus.h" 
#include "motor.h"
#include "fps.h" 
#include "enconder.h" 
/*------------------------------------  宏定义  -------------------------------*/
#define GIMBAL_PERIOD 5//运行周期
#define GIMBAL_CAN_TX_ID 					0x1ff//云台电机发送id
#define YAW_LIMIT_SPEED   100//yaw轴速度限幅
#define SHOOT_LIMIT_SPEED  100//拉橡皮筋速度限幅
#define YAW_RX_ID         0x205//yaw轴CAN接受ID
#define SHOOT_RX_ID       0x204//拉橡皮筋电机CAN接受ID
#define EN_RADIUS            			 25    //编码器轮子半径单位 mm
#define ENCONDER_POLES    			 3600 
#define CLAW_YAW_ID     0x01//爪子yaw轴id
#define CLAW_PITCH_ID     0x01//爪子pitch轴id
#define CLAW_YAW_ANGLE    90//爪子yaw轴转的角度
#define CLAW_PITCH_ANGLE  90//爪子pitch轴转的角度
#define AIR_OFF   0//关闭
#define AIR_ON   1//开启
#define QUEUE_LEN      5U//深度为5
#define  QUEUE_SIZE    8U//长度为5;

#define ENCONDER_TIPS   (0x80000000U)//编码器信息

/*---------------------------------  结构体声明  ------------------------------*/
	typedef struct gimbalStruct
	{
		RM3508Struct* 		 pyaw_t;
		RM3508Struct* 		 pshoot_t;
		const dbusStruct* pRc_t;
		incrementalEnconderStruct* pEnconder_t;
		uint32_t status;
		int32_t  encoder;
		struct
		{
			uint8_t air_valve;//夹子气阀开关
			servoStruct* pitch;//pitch轴舵机
			servoStruct* yaw;//yaw轴舵机
		}claw_t;//爪子
	}gimbalStruct;
	
/*-----------------------------------  函数声明  ------------------------------*/
	void GimbalInit(const dbusStruct* pRc_t);
	void GimbalCtrl(void);
	RM3508Struct* YawInit(void);
	RM3508Struct* ShootInit(void);
	void SetGimbalInitStatus(void);
	void ClawInit(void);
	void ClawOff(void);
	void ClawOn(void);
	void GimbalParseDate(uint32_t id,uint8_t *data);
	int16_t YawPidControl(int16_t yaw);
	int16_t shootPidControl(int16_t shoot);
	HAL_StatusTypeDef GimbalCanTx(int16_t yaw,int16_t shoot);
#endif	// __GIMBAL_H
/*-----------------------------------file of end------------------------------*/
