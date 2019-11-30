/**
	|----------------------------- Copyright ------------------------------------|
	|                                                                            |
	|                     (C) Copyright 2018,海康平头哥,                          |
	|         1 Xuefu Rd, Huadu Qu, Guangzhou Shi, Guangdong Sheng, China        |
	|                  -       All Rights Reserved                               |
	|                                                                            |
	|          By(GCU The wold of team | 华南理工大学广州学院机器人野狼队)          |
	|                  https://github.com/GCUWildwolfteam                        |
	|----------------------------------------------------------------------------|
	|--FileName    : counters.c                                                
	|--Version     : v1.0                                                            
	|--Author      : 海康平头哥                                                       
	|--Date        : 2018-11-27               
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
#include "counters.h" 
#include <Math.h>
/* ===================== Bezier(贝塞尔曲线) of begin ===================== */
/** 使用说明
创建需要的轴的缓存数组，大小为曲线的精度
初始化结构体，没有的轴要赋值NULL 则如下示例
      uint32_t data[100];
      bezier_t.coordinate_x = datax;
      bezier_t.coordinate_y = datay;
      bezier_t.n = 2;
      bezier_t.point_y[0] = 0;    //阶数不能超过八阶
      bezier_t.point_y[1] = 0;
      bezier_t.point_y[2] = 0;
      bezier_t.point_x[0] = 0;    //阶数不能超过八阶
      bezier_t.point_x[1] = 0;
      bezier_t.point_x[2] = 0;
      bezier_t.precision = 100;
调用 CreateDataSpace计算曲线，会自动储存在data中
 
*/
	/**
		* @Data    2019-02-21 16:38
		* @brief   创建坐标点储存空间
		* @param   void
		* @retval  void
		*/
  SYS_Status CreateDataSpace(bezierStruct* bs)
  {
    uint32_t i = 0;
    float t=0;
    if(bs->coordinate_x == NULL || bs->coordinate_y == NULL)
      return SYS_ERROR;
      for(i = 0;i<(bs->precision);i++)
      {
        t = i/bs->precision;
        *(bs->coordinate_x+i) = QuadTo(bs->n,bs->point_x,t);
        *(bs->coordinate_y+i) = QuadTo(bs->n,bs->point_y,t);
      }        
    return SYS_OK;
  }
	/**
	* @Data    2019-02-21 14:08
	* @brief   n阶贝塞尔曲线坐标生成 
							Bn(t) =(1?t)nP0+∑i=1n?1nti(1?t)n?iPi+tnPn(n≥2,n∈Z)
	* @param   n 贝塞尔阶数
	* @param   *point  控制点地址
	* @param  t 曲线系数
	* @retval uint32_t 计算得出坐标x或y的值
	*/
	uint32_t QuadTo(const uint8_t n,const uint32_t *point,float t)
	{
		if(n < 2 && n < 8)
		{
			return 0;
		}
		uint8_t i = 1;
		float sum_Pi = 0;
		for(i = 1;i < n;i++)
		{
			sum_Pi += n *pow(t,i) * pow((1-t),(n-i)) * (*(point+i));
		}
		return (uint32_t) \
						((pow((1-t),n) * (*(point))) + sum_Pi +( pow(t,n) * (*(point+n))));
	}
/**
  * @Data    2019-02-21 16:38
  * @brief   贝塞尔曲线公式计算
  * @param   float t 贝塞尔曲线系数
  * @param   PointU32* points 控制点坐标
  * @param   int count控制点个数
  * @retval  tmp_points[0] 返回缓存曲线点结构体数组的地址
  */  
  PointU32 BezierInterpolation(float t, PointU32* points, int count)
  {
    PointU32 tmp_points[count];
    for (int i = 1; i < count; ++i)
    {
      for (int j = 0; j < count - i; ++j)
      {
        if (i == 1)
        {
          tmp_points[j].X = (uint32_t)(points[j].X * (1 - t) + points[j + 1].X * t);
          tmp_points[j].Y = (uint32_t)(points[j].Y * (1 - t) + points[j + 1].Y * t);
          continue;
        }
        tmp_points[j].X = (uint32_t)(tmp_points[j].X * (1 - t) + tmp_points[j + 1].X * t);
        tmp_points[j].Y = (uint32_t)(tmp_points[j].Y * (1 - t) + tmp_points[j + 1].Y * t);
      }
    }
    return tmp_points[0];
  } 
/**
  * @Data    2019-02-21 16:38
  * @brief   生成n贝塞尔曲线
  * @param   PointU32* points控制点坐标
  * @param   int count控制点个数
  * @param   PointU32* out_points 输出曲线细分个点坐标
  * @param   int out_count 细分曲线的精度
  * @retval  void
  */    
void CreateBezierCurves(PointU32* points, int count, PointU32* out_points,int out_count)
{
  float step = 1.0 / out_count;
  float t =0;
  for(int i=0; i<out_count; i++)
  {
      PointU32 temp_point = BezierInterpolation(t, points, count);    // 计算插值点
      t += step;
      out_points[i] = temp_point;
  }
}
   
/* ====================== Bezier(贝塞尔曲线) of end ====================== */
/* =========================== PID控制器 of begin =========================== */
/**
	* @Data    2019-01-26 16:55
	* @brief   位置pid控制器
	* @param  postionPidStruct 位置pid结构体指针
	* @param  当前误差
	* @retval  int16_t
	*/
	int16_t PostionPid(postionPidStruct *pps,int16_t error)
	{
	  pps->error	= error;
    //pps->kp = KpSeparationCallback(pps);
		pps->pout = pps->kp * pps->error;
    pps->pout = MAX(pps->pout,pps->motor_lim);
    pps->pout = MIN(pps->pout,(-pps->motor_lim));
		pps->iout =  IntegralSeparationCallback(pps);
		pps->dout = pps->kd * (pps->error - pps->last_error);
		pps->last_error = pps->error;
    pps->pid_out = MAX(((int32_t)(pps->pout + pps->iout + pps->dout)),pps->motor_lim);
    pps->pid_out = MIN(((int32_t)(pps->pout + pps->iout + pps->dout)),(-pps->motor_lim));
		return pps->pid_out;
	}
__weak int16_t IntegralSeparationCallback(postionPidStruct *pps)
{
  if(pps->integral_threshold ==0)
    return 0;
    pps->integral_er += pps->error;
    pps->integral_er = MAX(pps->integral_er,pps->integral_limint);
    pps->integral_er = MIN(pps->integral_er,(-pps->integral_limint));
  if(ABS(pps->error) > pps->integral_threshold)
  {
    pps->integral_er = 0;
  }
  return  pps->ki * pps->integral_er;
}
__weak int16_t KpSeparationCallback(postionPidStruct *pps)
{
  if(pps->kp_separatecmd ==1)
  {
    if(ABS(pps->error) >43)
    {
      pps->kp = 220;
    }
    else pps->kp = (int16_t)(-0.015 *(float)(pps->error*pps->error) + 245);
  }
  return pps->kp;
}
/**
	* @Data    2019-01-26 16:55
	* @brief   速度pid控制器
	* @param  speedPidStruct 速度pid结构体指针
	* @param  当前误差
	* @retval  int16_t
	*/
	int16_t SpeedPid(speedPidStruct *sps,int16_t error)
	{
		if(sps==NULL)
		 return 0;
		sps->error = error;
//		if((ABS(sps->error)) < (ABS(sps->last_error)))
//			sps->last_error = sps->error;
//		if((ABS(sps->last_error)) < (ABS(sps->before_last_error)))
//			sps->before_last_error = sps->last_error;
		sps->pout = (int32_t)(sps->kp * (sps->error - sps->last_error));
    sps->pout = MAX(sps->pout,sps->motor_lim);
    sps->pout = MIN(sps->pout,-sps->motor_lim);
    
		sps->iout = (int32_t)(sps->ki * sps->error);
    sps->iout = MAX(sps->iout,sps->motor_lim);
    sps->iout = MIN(sps->iout,-sps->motor_lim);
    
		sps->dout = (int32_t)(sps->kd * (sps->error - 2*sps->last_error + \
													 sps->before_last_error));
    sps->dout = MAX(sps->dout,sps->motor_lim);
    sps->dout = MIN(sps->dout,-sps->motor_lim);
    
		sps->pid_out += (int32_t)(sps->pout + sps->iout + sps->dout);
    sps->pid_out = MAX(sps->pid_out,sps->motor_lim);
    sps->pid_out = MIN(sps->pid_out,(-sps->motor_lim));
    
    sps->pid_out = MAX(sps->pid_out,sps->limiting);
    sps->pid_out = MIN(sps->pid_out,(-sps->limiting));
    
    sps->before_last_error = sps->last_error;
		sps->last_error = sps->error;
		return (int16_t)sps->pid_out;
	}
/* ============================ PID控制器 of end ============================ */

/* =========================== 加减速 of begin =========================== */
/**
	* @Data    2019-03-30 16:48
	* @brief   直线减速
	* @param   int16_t
	* @retval  void
	*/
	int16_t LineAcc(int16_t r,int16_t t,int16_t p)
	{
		if(r < t)
		{
			r -= p;
			return r;
		}
		else return r;
	}
  
/* =========================== 加减速 of end =========================== */

void Insert(int16_t fx[], int16_t x[])
{
	int i, j, a, b = 2;
	for (i = 0; i<2; i++)
	{
		a = b;
		for (j = 2; j>i; j--)
		{
			fx[j] = (fx[j] - fx[j - 1]) / (x[j] - x[--a]);
		}
		b--;
	}
}
int16_t NewtonInterpolation(int16_t fx[], int16_t x[], int16_t a)
{
	int16_t i;
	int16_t sum;
	int16_t cc = 1;
	sum = fx[0];
	for (i = 1; i<3; i++)
	{
		cc = cc*(a - x[i - 1]);
		sum = sum + fx[i] * cc;
	}
	return sum;
}
/*--------------------------------file of end---------------------------------*/


