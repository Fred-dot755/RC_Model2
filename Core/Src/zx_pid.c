#include "zx_pid.h"

void zxPID_init(float Kp,float Ki,float Kd,float I_Max,float V_Max,zxPID *_this){
	_this->Kp=Kp;
	_this->Ki=Ki;
	_this->Kd=Kd;
	_this->I_Max=I_Max;
	_this->V_Max=V_Max;


	_this->zx_Calc=zx_Calc;
	_this->zx_Calc_Err=zx_Calc_Err;
}


float zx_Calc(float target,float actual,zxPID *_this){
	_this->SetSpeed=target;
	_this->AcutualSpeed=actual;
	_this->err	=	target	-	actual;

	_this->interal	+=	_this->Ki * _this->err;
	if(_this->interal > _this->I_Max)
					_this->interal	=		_this->I_Max;
	else if(_this->interal < -_this->I_Max)
					_this->interal	=		-_this->I_Max;

	_this->voltage	=		_this->Kp * _this->err + _this->interal + _this->Kd * (_this->err - _this->err_last);
	_this->err_last	=		_this->err;

	if(_this->voltage > _this->V_Max) 		_this->voltage 	= _this->V_Max;
	else if(_this->voltage < -_this->V_Max) 	_this->voltage	=	-_this->V_Max;

	return _this->voltage;

}
float zx_Calc_Err(float Err,float actual,zxPID *_this){
	_this->AcutualSpeed=actual;
	_this->err	=	Err;

	_this->interal	+=	_this->Ki * _this->err;
	if(_this->interal > _this->I_Max)
					_this->interal	=		_this->I_Max;
	else if(_this->interal < -_this->I_Max)
					_this->interal	=		-_this->I_Max;

	_this->voltage	=		_this->Kp * _this->err + _this->interal + _this->Kd * (_this->err - _this->err_last);
	_this->err_last	=		_this->err;

	if(_this->voltage > _this->V_Max) 		_this->voltage 	= _this->V_Max;
	else if(_this->voltage < -_this->V_Max) 	_this->voltage	=	-_this->V_Max;
	return _this->voltage;
}

