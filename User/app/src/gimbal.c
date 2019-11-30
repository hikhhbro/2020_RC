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
	|--FileName    : gimbal.c                                              
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
#include "gimbal.h" 
/* -------------- 结构体 ----------------- */
	gimbalStruct gimbal_t;
	static RM3508Struct 		 yaw_t;
	static postionPidStruct yawOutLoop_t;
	static speedPidStruct 	 yawInLoop_t;
	static RM3508Struct 		 shoot_t;
	static speedPidStruct 	 shootspeedloop_t;
	static incrementalEnconderStruct yawEnconder_t;
	static servoStruct clawPitch_t;//爪子pitch轴舵机
	static servoStruct clawYaw_t;//爪子yaw轴舵机
/* -------------- 外部链接 ----------------- */
	extern osThreadId startGimbalTaskHandle;
/* -------------- 发送队列 ----------------- */
  xQueueHandle gimbal_queue;
/**
	* @Data    2019-11-30 15:59
	* @brief   初始化
	* @param   void
	* @retval  void
	*/
void GimbalInit(const dbusStruct* pRc_t){
	gimbal_t.pyaw_t = YawInit();
	gimbal_t.pshoot_t = ShootInit();
	ClawInit();//爪子初始化
	gimbal_t.pRc_t = pRc_t;
	/* ------ 编码器初始化 ------- */
	 gimbal_t.pEnconder_t = &yawEnconder_t;
	 if(EnconderInit(&yawEnconder_t,EN_RADIUS,ENCONDER_POLES) !=HAL_OK)
	 {
			gimbal_t.status = ENCONDER_TIPS;
	 }
	/* ------ 创建云台发送队列 ------- */
	  gimbal_queue	= xQueueCreate(QUEUE_LEN,QUEUE_SIZE);//一定要在用之前创建队列
	/* -------- can配置 --------- */
    if(UserCanConfig()!= HAL_OK)
    {
      while(1)
      {
		
      }//待添加防重复配置功能
    }
	/* ------ 挂起任务，等待初始化 ------- */
		vTaskSuspend(startGimbalTaskHandle);
	/* ------ 设置云台初始化状态 ------- */
		SetGimbalInitStatus();
}
/**
	* @Data    2019-11-30 15:59
	* @brief   云台控制
	* @param   void
	* @retval  void
	*/
int16_t yaw_out = 0;
float rc_cen = 0.07f;
void GimbalCtrl(void)
{
	gimbal_t.pyaw_t->target += (int16_t)(gimbal_t.pRc_t->ch1 * rc_cen);
	if(gimbal_t.pyaw_t->target <0)
    gimbal_t.pyaw_t->target = 8191 + gimbal_t.pyaw_t->target;
  if(gimbal_t.pyaw_t->target > 8191)
     gimbal_t.pyaw_t->target = 8191 - gimbal_t.pyaw_t->target;
	gimbal_t.pshoot_t->target = gimbal_t.pRc_t->ch3;
	gimbal_t.pyaw_t->error = gimbal_t.pyaw_t->target - gimbal_t.encoder;
	gimbal_t.pshoot_t->error = gimbal_t.pshoot_t->target - gimbal_t.pshoot_t->real_speed;
	GimbalCanTx(YawPidControl(gimbal_t.pyaw_t->error),shootPidControl(gimbal_t.pshoot_t->error));
	if(gimbal_t.pRc_t->switch_right == 1)
		ClawOn();
	else ClawOff();
}
/**
  * @Data    2019-11-30 23:24
  * @brief   yaw轴电机初始化
  * @param   void
  * @retval  RM3508Struct
  */
RM3508Struct* YawInit(void){
	yaw_t.id = YAW_RX_ID;//电机can的 ip
	yaw_t.target = 0;		 //目标值
	yaw_t.tem_target = 0;//临时目标值
	yaw_t.real_current = 0; //真实电流
	yaw_t.real_angle = 0;//真实角度
	yaw_t.tem_angle = 0;//临时角度
	yaw_t.real_speed = 0;//真实速度
	yaw_t.tem_speed = 0;//真实速度
	yaw_t.zero = 0;			 //电机零点
	yaw_t.Percentage = 0;//转换比例（减速前角度:减速后的角度 = x:1
	yaw_t.thresholds = 0; //电机反转阀值
	yaw_t.error = 0;//当前误差
	SetFpsAddress(yaw_t.fps); //开启帧率统计

	yaw_t.ppostionPid_t = &yawOutLoop_t;//外环
	yaw_t.pspeedPid_t = &yawInLoop_t;//内环
/* -------- 内环 --------- */
	yawInLoop_t.error = 0;
	yawInLoop_t.last_error = 0;//上次误差
	yawInLoop_t.before_last_error = 0;//上上次误差
	yawInLoop_t.integral_er = 0;//误差积分
	yawInLoop_t.pout = 0;//p输出
	yawInLoop_t.iout = 0;//i输出
	yawInLoop_t.dout = 0;//k输出
	yawInLoop_t.pid_out = 0;//pid输出
	yawInLoop_t.motor_lim = RM3508_LIM;//3508最大电流范围
	yawInLoop_t.kp = 4;
	yawInLoop_t.kd = 0;
	yawInLoop_t.ki = 0.2;
	yawInLoop_t.limiting = YAW_LIMIT_SPEED;//轮子1速度限幅

/* -------- 外环 --------- */
	yawOutLoop_t.integral_limint = 2000;
	yawOutLoop_t.integral_threshold =0;
	yawOutLoop_t.kp_separatecmd = 0;
	yawOutLoop_t.motor_lim = RM3508_LIM;
	yawOutLoop_t.kd = 0;
	yawOutLoop_t.ki = 0;
	yawOutLoop_t.kp =-4;
	return &yaw_t;
}
/**
  * @Data    2019-11-30 23:24
  * @brief   拉橡皮经电机初始化
  * @param   void
  * @retval  RM3508Struct
  */
RM3508Struct* ShootInit(void){
	shoot_t.id = YAW_RX_ID;//电机can的 ip
	shoot_t.target = 0;		 //目标值
	shoot_t.tem_target = 0;//临时目标值
	shoot_t.real_current = 0; //真实电流
	shoot_t.real_angle = 0;//真实角度
	shoot_t.tem_angle = 0;//临时角度
	shoot_t.real_speed = 0;//真实速度
	shoot_t.tem_speed = 0;//真实速度
	shoot_t.zero = 0;			 //电机零点
	shoot_t.Percentage = 0;//转换比例（减速前角度:减速后的角度 = x:1
	shoot_t.thresholds = 0; //电机反转阀值
	shoot_t.error = 0;//当前误差
	SetFpsAddress(shoot_t.fps); //开启帧率统计

	shoot_t.pspeedPid_t = &shootspeedloop_t;//速度环

	shootspeedloop_t.error = 0;
	shootspeedloop_t.last_error = 0;//上次误差
	shootspeedloop_t.before_last_error = 0;//上上次误差
	shootspeedloop_t.integral_er = 0;//误差积分
	shootspeedloop_t.pout = 0;//p输出
	shootspeedloop_t.iout = 0;//i输出
	shootspeedloop_t.dout = 0;//k输出
	shootspeedloop_t.pid_out = 0;//pid输出
	shootspeedloop_t.motor_lim = RM3508_LIM;//3508最大电流范围
	shootspeedloop_t.kp = 4;
	shootspeedloop_t.kd = 0;
	shootspeedloop_t.ki = 0.2;
	shootspeedloop_t.limiting = SHOOT_LIMIT_SPEED;//轮子1速度限幅

	return &shoot_t;
}
/**
  * @Data    2019-11-30 23:24
  * @brief   爪子初始化
  * @param   void
  * @retval  void
  */
void ClawInit(void){
	gimbal_t.claw_t.pitch = &clawPitch_t;
	gimbal_t.claw_t.pitch->max_limit = 2000;
	gimbal_t.claw_t.pitch->min_limit = 1000;
	gimbal_t.claw_t.pitch->zero = 1500;
	gimbal_t.claw_t.yaw = &clawYaw_t;
	gimbal_t.claw_t.yaw->max_limit = 2000;
	gimbal_t.claw_t.yaw->min_limit = 1000;
	gimbal_t.claw_t.yaw->zero = 1500;
}
 /**
	 * @Data    2019-11-30 23:47
	 * @brief   底盘初始化状态设置
	 * @param   void
	 * @retval  void
	 */
 void SetGimbalInitStatus(void){
	/* -------- 内环 --------- */
	yaw_t.target = 0;		 //目标值
	yawInLoop_t.kp = 0;
	yawInLoop_t.kd = 0;
	yawInLoop_t.ki = 0;
	/* -------- 外环 --------- */
	yawOutLoop_t.kd = 0;
	yawOutLoop_t.ki = 0;
	yawOutLoop_t.kp =0;
	shoot_t.target = 0;		 //目标值
	shootspeedloop_t.kp = 0;
	shootspeedloop_t.kd = 0;
	shootspeedloop_t.ki = 0;
	ClawOff();//收回爪子
}
/**
	* @Data   2019-11-30 23:51
	* @brief  爪子抓取
	* @param  舵机的pitch 和paw
	* @retval  void
	*/
void ClawOn(void){
	DXL1_setSyncTarAng(SERVO_USART,2,\
		CLAW_YAW_ID,SetTargetServoPoles(gimbal_t.claw_t.yaw,CLAW_YAW_ANGLE),\
		CLAW_PITCH_ID,SetTargetServoPoles(gimbal_t.claw_t.pitch,CLAW_PITCH_ANGLE));
	gimbal_t.claw_t.air_valve =  AIR_ON;
} 
/**
	* @Data   2019-11-30 23:51
	* @brief  爪子收回
	* @param  舵机的pitch 和paw
	* @retval  void
	*/
void ClawOff(void){
	DXL1_setSyncTarAng(SERVO_USART,2,\
		CLAW_YAW_ID,SetTargetServoPoles(gimbal_t.claw_t.yaw,0),\
		CLAW_PITCH_ID,SetTargetServoPoles(gimbal_t.claw_t.pitch,0));
	gimbal_t.claw_t.air_valve =  AIR_OFF;
}  
/**
	* @Data    2019-12-01 01:18
	* @brief   云台数据解析
	* @param   id 接收id
	* @param   *data 接收数据地址
	* @retval  void
	*/
	void GimbalParseDate(uint32_t id,uint8_t *data)
	{
		switch (id)
		{
			case SHOOT_RX_ID:
				RM3508ParseData(gimbal_t.pshoot_t,data);
				Fps(gimbal_t.pshoot_t->fps);
				break;
			case YAW_RX_ID:
				RM3508ParseData(gimbal_t.pyaw_t,data);
				Fps(gimbal_t.pyaw_t->fps);
				break;
		}
	}
/**
	* @Data    2019-02-15 15:10
	* @brief   云台数据发送
	* @param   void
	* @retval  void
	*/
HAL_StatusTypeDef GimbalCanTx(int16_t yaw,int16_t shoot){
	uint8_t s[8];
	memset(s,0,8);
	s[0] = (uint8_t)(yaw>>8) & 0xFF;
	s[1] = (uint8_t)yaw;
	s[2] = (uint8_t)(shoot>>8) & 0xFF;
	s[3] = (uint8_t)shoot;
	xQueueSendToBack(gimbal_queue,s,0);
	return HAL_OK;
}
/**
* @Data    2019-12-01 01:42
* @brief   yaw 轴pid控制
* @param   yaw yaw系统误差
* @retval  void
*/
float out_con = 1;
int16_t YawPidControl(int16_t yaw){
	/* -------- 外环 --------- */
	gimbal_t.pyaw_t->ppostionPid_t->pid_out = PostionPid(gimbal_t.pyaw_t->ppostionPid_t,yaw);
	/* -------- 内环 --------- */
	gimbal_t.pyaw_t->pspeedPid_t->error= (int16_t)	gimbal_t.pyaw_t->ppostionPid_t->pid_out * out_con - gimbal_t.pyaw_t->real_speed;
	return SpeedPid(gimbal_t.pyaw_t->pspeedPid_t,gimbal_t.pyaw_t->error);
}
/**
* @Data    2019-12-01 01:42
* @brief   拉橡皮筋电机pid控制
* @param   shoot系统误差
* @retval  void
*/
int16_t shootPidControl(int16_t shoot){
	return SpeedPid(gimbal_t.pshoot_t->pspeedPid_t,gimbal_t.pshoot_t->error);
}
/*-----------------------------------file of end------------------------------*/
