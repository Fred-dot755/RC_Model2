//written by Fred Xiao

#include "main.h"
ops_t ops;

void gyroscope_data_decode(uint8_t *buffer, uint8_t length)
{
	if(length==82)//���ܵ�HIPNUC������������һ֡���ݣ�82�ֽڣ�
	{
		int offset = 6;
		ops.HIPNUCAngleX=HIPNUC_anagle_decode(buffer+offset+48);//������
		ops.HIPNUCAngleY=HIPNUC_anagle_decode(buffer+offset+52);//������
		ops.HIPNUCAngleZ=HIPNUC_anagle_decode(buffer+offset+56);//ƫ����
		ops.HIPNUCQuaternionsW=HIPNUC_anagle_decode(buffer+offset+60);//��Ԫ��W
		ops.HIPNUCQuaternionsX=HIPNUC_anagle_decode(buffer+offset+64);//��Ԫ��X
		ops.HIPNUCQuaternionsY=HIPNUC_anagle_decode(buffer+offset+68);//��Ԫ��Y
		ops.HIPNUCQuaternionsZ=HIPNUC_anagle_decode(buffer+offset+72);//��Ԫ��Z
		ops.HIPNUCaccelerationX=HIPNUC_anagle_decode(buffer+offset+12);//���ٶ�X
		ops.HIPNUCaccelerationY=HIPNUC_anagle_decode(buffer+offset+16);//���ٶ�Y
		ops.HIPNUCaccelerationZ=HIPNUC_anagle_decode(buffer+offset+20);//���ٶ�Z
		ops.HIPNUCgyroscopeX=HIPNUC_anagle_decode(buffer+offset+24);//���ٶ�X
		ops.HIPNUCgyroscopeY=HIPNUC_anagle_decode(buffer+offset+28);//���ٶ�y
		ops.HIPNUCgyroscopeZ=HIPNUC_anagle_decode(buffer+offset+32);//���ٶ�z
		
	}
}

//-------------------------------------------------------------------------------------------------------------------
// �������       	HIPNUC�����ǽǶ����ݽ���
// ����˵��       	*p				���ܵ�HIPNUC�����ǵ�һ֡���ݣ�82�ֽڣ�
// @return        	float			�Ƕ�ֵ
// Sample usage:	HIPNUC_anagle_decode(buffer+offset+48);
//-------------------------------------------------------------------------------------------------------------------
float HIPNUC_anagle_decode(uint8_t *p)
{
	float r;
	memcpy(&r,p,4);
	return r;
}
//-------------------------------------------------------------------------------------------------------------------//
void HIPNUC_data_printf(void)
{
	printf("%.2f %.2f %.2f\r\n",ops.HIPNUCAngleX,//������
	                            ops.HIPNUCAngleY,//������
	                            ops.HIPNUCAngleZ);//ƫ����
}
//-------------------------------------------------------------------------------------------------------------------//
void HIPNUC_Quaternions_decode(uint8_t *buffer, uint8_t length)
{
    
}


