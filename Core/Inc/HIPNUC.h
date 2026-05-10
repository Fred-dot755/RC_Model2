//written by Fred Xiao

#ifndef __HIPNUC_H__
#define __HIPNUC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
typedef struct
{
	/*HIPNUC���ص�X��Ƕ�ֵ*/
	float HIPNUCAngleX;
	/*HIPNUC���ص�Y��Ƕ�ֵ*/
	float HIPNUCAngleY;
	/*HIPNUC���ص�Z��Ƕ�ֵ*/
	float HIPNUCAngleZ;
	/*HIPNUC���ص���Ԫ��w*/
	float HIPNUCQuaternionsW;
	/*HIPNUC���ص���Ԫ��x*/
	float HIPNUCQuaternionsX;
	/*HIPNUC���ص���Ԫ��y*/
	float HIPNUCQuaternionsY;
	/*HIPNUC���ص���Ԫ��z*/
	float HIPNUCQuaternionsZ;
	
	/*HIPNUC���صĽ��ٶ�X��*/
	float HIPNUCgyroscopeX;
	/*HIPNUC���صĽ��ٶ�Y��*/
	float HIPNUCgyroscopeY;
	/*HIPNUC���صĽ��ٶ�Z��*/
	float HIPNUCgyroscopeZ;

	/*HIPNUC���صļ��ٶ�X��*/
	float HIPNUCaccelerationX;
	/*HIPNUC���صļ��ٶ�Y��*/
	float HIPNUCaccelerationY;
	/*HIPNUC���صļ��ٶ�Z��*/
	float HIPNUCaccelerationZ;
	float v;//΢������ȡ���ڶ�ʱ��ʱ��
}ops_t;
extern ops_t ops;
float HIPNUC_anagle_decode(uint8_t *p);
void gyroscope_data_decode(uint8_t *buffer, uint8_t length);
void HIPNUC_data_printf(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __HIPNUC_H__ */



