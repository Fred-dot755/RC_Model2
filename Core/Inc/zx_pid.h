#ifndef INC_ZXPID_H_
#define INC_ZXPID_H_

typedef struct zxPID{
	float Kp;                 //比例
	float Ki;                 //积分
	float Kd;                 //微分
	float SetSpeed;						//设定值
	float AcutualSpeed;				//实际值
	float err;								//偏差值
	float err_last;						//上一个偏差值
	float err_l_last;         //上上一个偏差值
	float I_Max;							//积分上限
	float V_Max;							//输出上限
	float voltage;						//定义电压值（控制执行器的变量）
	float interal;						//定义积分值

	float (*zx_Calc)(float target,float actual,struct zxPID*);
	float (*zx_Calc_Err)(float Err,float actual,struct zxPID*);


}zxPID;
void zxPID_init(float Kp,float Ki,float Kd,float I_Max,float V_Max,zxPID *_this);
float zx_Calc(float target,float actual,zxPID *_this);
float zx_Calc_Err(float Err,float actual,zxPID *_this);


#endif /* INC_ZX_PID_H_ */
