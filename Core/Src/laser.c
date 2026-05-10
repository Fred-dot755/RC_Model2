//written by Fred Xiao
#include "laser.h"
#include "math.h"
int laser_adc[4]={0};
uint8_t step;
void Laser_Data_Decode(uint8_t *buffer, uint8_t length)
{
//	for(int i=0;i<length;i++)
//		{
//		printf_USART6("%d ",buffer[i]);
//
//		}
	step=0;
	uint8_t temp[8];
	uint8_t num_flag[4]={0};
	for(uint8_t i=0;i<length;i++)
    {
        uint8_t data = buffer[i];
//		if(data==67)printf_USART6("%d\n",step);

		switch(step)
        {
			case 0:
//				printf_USART6("0\n");
				if(data==65) step=1;
				else if(data==67) step=3;
				else if(data==69) step=5;
				else if(data==71) step=7;
				else step=0;
				break;
			case 1:
//				printf_USART6("1\n");
				if(data==66)
				{
					laser_adc[0]=0;
					for(uint8_t i=0;i<num_flag[0];i++)
					{
						laser_adc[0]+=(temp[i]-48)*pow(10,num_flag[0]-i-1);
					}
					step=0;
				}
				else
				{
					temp[num_flag[0]]=data;
					num_flag[0]++;
				}
				if(num_flag[0]==8) step=0;
				break;
			case 3:
				if(data==68)
				{
					laser_adc[1]=0;
					for(uint8_t i=0;i<num_flag[1];i++)
					{
						laser_adc[1]+=(temp[i]-48)*pow(10,num_flag[1]-i-1);
					}
					step=0;
				}
				else
				{
					temp[num_flag[1]]=data;
					num_flag[1]++;
				}
				if(num_flag[1]==8) step=0;
				break;
			case 5:
				if(data==70)
				{
					laser_adc[2]=0;
					for(uint8_t i=0;i<num_flag[2];i++)
					{
						laser_adc[2]+=(temp[i]-48)*pow(10,num_flag[2]-i-1);
					}
					step=0;
				}
				else
				{
					temp[num_flag[2]]=data;
					num_flag[2]++;
				}
				if(num_flag[2]==8) step=0;
				break;
			case 7:
				if(data==72)
				{
					laser_adc[3]=0;
					for(uint8_t i=0;i<num_flag[3];i++)
					{
						laser_adc[3]+=(temp[i]-48)*pow(10,num_flag[3]-i-1);
					}
					step=0;
				}
				else
				{
					temp[num_flag[3]]=data;
					num_flag[3]++;
				}
				if(num_flag[3]==8) step=0;
				break;
			default:
				step=0;
				break;

		}

	}
	laser_adc[0]=laser_adc[0]* 0.00009448;
	laser_adc[1]=laser_adc[1]* 0.00009448;
	laser_adc[2]=laser_adc[2]* 0.00009448;
	laser_adc[3]=laser_adc[3]* 0.00009448;

}

