//written by Fred Xiao

#include "motor3508.h" 

DJmotor_struct motor[12];

// 底盘电机控制数据结构
uint8_t chassis_fdcan1_send_data_0x200[8] = {0};
FDCAN_TxHeaderTypeDef chassis_tx_message_fdcan1_0x200;

uint8_t chassis_fdcan1_send_data_0x1FF[8] = {0};
FDCAN_TxHeaderTypeDef chassis_tx_message_fdcan1_0x1FF;

uint8_t chassis_fdcan2_send_data_0x200[8] = {0};
FDCAN_TxHeaderTypeDef chassis_tx_message_fdcan2_0x200;

uint8_t chassis_fdcan2_send_data_0x1FF[8] = {0};
FDCAN_TxHeaderTypeDef chassis_tx_message_fdcan2_0x1FF;

uint8_t chassis_fdcan3_send_data_0x200[8] = {0};
FDCAN_TxHeaderTypeDef chassis_tx_message_fdcan3_0x200;

uint8_t chassis_fdcan3_send_data_0x1FF[8] = {0};
FDCAN_TxHeaderTypeDef chassis_tx_message_fdcan3_0x1FF;


// FDCAN1 发送函数
void FDCAN_cmd_chassis_fdcan1_0x200(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    chassis_tx_message_fdcan1_0x200.Identifier = 0x200;
    chassis_tx_message_fdcan1_0x200.IdType = FDCAN_STANDARD_ID;
    chassis_tx_message_fdcan1_0x200.TxFrameType = FDCAN_DATA_FRAME;
    chassis_tx_message_fdcan1_0x200.DataLength = FDCAN_DLC_BYTES_8;
    chassis_tx_message_fdcan1_0x200.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    chassis_tx_message_fdcan1_0x200.BitRateSwitch = FDCAN_BRS_OFF;
    chassis_tx_message_fdcan1_0x200.FDFormat = FDCAN_CLASSIC_CAN;
    chassis_tx_message_fdcan1_0x200.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    chassis_tx_message_fdcan1_0x200.MessageMarker = 0;
    
    chassis_fdcan1_send_data_0x200[0] = (uint8_t)(motor1 >> 8);
    chassis_fdcan1_send_data_0x200[1] = (uint8_t)(motor1 & 0xFF);
    chassis_fdcan1_send_data_0x200[2] = (uint8_t)(motor2 >> 8);
    chassis_fdcan1_send_data_0x200[3] = (uint8_t)(motor2 & 0xFF);
    chassis_fdcan1_send_data_0x200[4] = (uint8_t)(motor3 >> 8);
    chassis_fdcan1_send_data_0x200[5] = (uint8_t)(motor3 & 0xFF);
    chassis_fdcan1_send_data_0x200[6] = (uint8_t)(motor4 >> 8);
    chassis_fdcan1_send_data_0x200[7] = (uint8_t)(motor4 & 0xFF);
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &chassis_tx_message_fdcan1_0x200, chassis_fdcan1_send_data_0x200);
}

void FDCAN_cmd_chassis_fdcan1_0x1FF(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    chassis_tx_message_fdcan1_0x1FF.Identifier = 0x1FF;
    chassis_tx_message_fdcan1_0x1FF.IdType = FDCAN_STANDARD_ID;
    chassis_tx_message_fdcan1_0x1FF.TxFrameType = FDCAN_DATA_FRAME;
    chassis_tx_message_fdcan1_0x1FF.DataLength = FDCAN_DLC_BYTES_8;
    chassis_tx_message_fdcan1_0x1FF.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    chassis_tx_message_fdcan1_0x1FF.BitRateSwitch = FDCAN_BRS_OFF;
    chassis_tx_message_fdcan1_0x1FF.FDFormat = FDCAN_CLASSIC_CAN;
    chassis_tx_message_fdcan1_0x1FF.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    chassis_tx_message_fdcan1_0x1FF.MessageMarker = 0;
    
    chassis_fdcan1_send_data_0x1FF[0] = (uint8_t)(motor1 >> 8);
    chassis_fdcan1_send_data_0x1FF[1] = (uint8_t)(motor1 & 0xFF);
    chassis_fdcan1_send_data_0x1FF[2] = (uint8_t)(motor2 >> 8);
    chassis_fdcan1_send_data_0x1FF[3] = (uint8_t)(motor2 & 0xFF);
    chassis_fdcan1_send_data_0x1FF[4] = (uint8_t)(motor3 >> 8);
    chassis_fdcan1_send_data_0x1FF[5] = (uint8_t)(motor3 & 0xFF);
    chassis_fdcan1_send_data_0x1FF[6] = (uint8_t)(motor4 >> 8);
    chassis_fdcan1_send_data_0x1FF[7] = (uint8_t)(motor4 & 0xFF);
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &chassis_tx_message_fdcan1_0x1FF, chassis_fdcan1_send_data_0x1FF);
}

// FDCAN2 发送函数
void FDCAN_cmd_chassis_fdcan2_0x200(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    chassis_tx_message_fdcan2_0x200.Identifier = 0x200;
    chassis_tx_message_fdcan2_0x200.IdType = FDCAN_STANDARD_ID;
    chassis_tx_message_fdcan2_0x200.TxFrameType = FDCAN_DATA_FRAME;
    chassis_tx_message_fdcan2_0x200.DataLength = FDCAN_DLC_BYTES_8;
    chassis_tx_message_fdcan2_0x200.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    chassis_tx_message_fdcan2_0x200.BitRateSwitch = FDCAN_BRS_OFF;
    chassis_tx_message_fdcan2_0x200.FDFormat = FDCAN_CLASSIC_CAN;
    chassis_tx_message_fdcan2_0x200.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    chassis_tx_message_fdcan2_0x200.MessageMarker = 0;
    
    chassis_fdcan2_send_data_0x200[0] = (uint8_t)(motor1 >> 8);
    chassis_fdcan2_send_data_0x200[1] = (uint8_t)(motor1 & 0xFF);
    chassis_fdcan2_send_data_0x200[2] = (uint8_t)(motor2 >> 8);
    chassis_fdcan2_send_data_0x200[3] = (uint8_t)(motor2 & 0xFF);
    chassis_fdcan2_send_data_0x200[4] = (uint8_t)(motor3 >> 8);
    chassis_fdcan2_send_data_0x200[5] = (uint8_t)(motor3 & 0xFF);
    chassis_fdcan2_send_data_0x200[6] = (uint8_t)(motor4 >> 8);
    chassis_fdcan2_send_data_0x200[7] = (uint8_t)(motor4 & 0xFF);
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &chassis_tx_message_fdcan2_0x200, chassis_fdcan2_send_data_0x200);
}

void FDCAN_cmd_chassis_fdcan2_0x1FF(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    chassis_tx_message_fdcan2_0x1FF.Identifier = 0x1FF;
    chassis_tx_message_fdcan2_0x1FF.IdType = FDCAN_STANDARD_ID;
    chassis_tx_message_fdcan2_0x1FF.TxFrameType = FDCAN_DATA_FRAME;
    chassis_tx_message_fdcan2_0x1FF.DataLength = FDCAN_DLC_BYTES_8;
    chassis_tx_message_fdcan2_0x1FF.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    chassis_tx_message_fdcan2_0x1FF.BitRateSwitch = FDCAN_BRS_OFF;
    chassis_tx_message_fdcan2_0x1FF.FDFormat = FDCAN_CLASSIC_CAN;
    chassis_tx_message_fdcan2_0x1FF.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    chassis_tx_message_fdcan2_0x1FF.MessageMarker = 0;
    
    chassis_fdcan2_send_data_0x1FF[0] = (uint8_t)(motor1 >> 8);
    chassis_fdcan2_send_data_0x1FF[1] = (uint8_t)(motor1 & 0xFF);
    chassis_fdcan2_send_data_0x1FF[2] = (uint8_t)(motor2 >> 8);
    chassis_fdcan2_send_data_0x1FF[3] = (uint8_t)(motor2 & 0xFF);
    chassis_fdcan2_send_data_0x1FF[4] = (uint8_t)(motor3 >> 8);
    chassis_fdcan2_send_data_0x1FF[5] = (uint8_t)(motor3 & 0xFF);
    chassis_fdcan2_send_data_0x1FF[6] = (uint8_t)(motor4 >> 8);
    chassis_fdcan2_send_data_0x1FF[7] = (uint8_t)(motor4 & 0xFF);
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &chassis_tx_message_fdcan2_0x1FF, chassis_fdcan2_send_data_0x1FF);
}


// FDCAN3 发送函数
void FDCAN_cmd_chassis_fdcan3_0x200(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    chassis_tx_message_fdcan3_0x200.Identifier = 0x200;
    chassis_tx_message_fdcan3_0x200.IdType = FDCAN_STANDARD_ID;
    chassis_tx_message_fdcan3_0x200.TxFrameType = FDCAN_DATA_FRAME;
    chassis_tx_message_fdcan3_0x200.DataLength = FDCAN_DLC_BYTES_8;
    chassis_tx_message_fdcan3_0x200.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    chassis_tx_message_fdcan3_0x200.BitRateSwitch = FDCAN_BRS_OFF;
    chassis_tx_message_fdcan3_0x200.FDFormat = FDCAN_CLASSIC_CAN;
    chassis_tx_message_fdcan3_0x200.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    chassis_tx_message_fdcan3_0x200.MessageMarker = 0;
    
    chassis_fdcan3_send_data_0x200[0] = (uint8_t)(motor1 >> 8);
    chassis_fdcan3_send_data_0x200[1] = (uint8_t)(motor1 & 0xFF);
    chassis_fdcan3_send_data_0x200[2] = (uint8_t)(motor2 >> 8);
    chassis_fdcan3_send_data_0x200[3] = (uint8_t)(motor2 & 0xFF);
    chassis_fdcan3_send_data_0x200[4] = (uint8_t)(motor3 >> 8);
    chassis_fdcan3_send_data_0x200[5] = (uint8_t)(motor3 & 0xFF);
    chassis_fdcan3_send_data_0x200[6] = (uint8_t)(motor4 >> 8);
    chassis_fdcan3_send_data_0x200[7] = (uint8_t)(motor4 & 0xFF);
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &chassis_tx_message_fdcan3_0x200, chassis_fdcan3_send_data_0x200);
}

void FDCAN_cmd_chassis_fdcan3_0x1FF(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    chassis_tx_message_fdcan3_0x1FF.Identifier = 0x1FF;
    chassis_tx_message_fdcan3_0x1FF.IdType = FDCAN_STANDARD_ID;
    chassis_tx_message_fdcan3_0x1FF.TxFrameType = FDCAN_DATA_FRAME;
    chassis_tx_message_fdcan3_0x1FF.DataLength = FDCAN_DLC_BYTES_8;
    chassis_tx_message_fdcan3_0x1FF.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    chassis_tx_message_fdcan3_0x1FF.BitRateSwitch = FDCAN_BRS_OFF;
    chassis_tx_message_fdcan3_0x1FF.FDFormat = FDCAN_CLASSIC_CAN;
    chassis_tx_message_fdcan3_0x1FF.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    chassis_tx_message_fdcan3_0x1FF.MessageMarker = 0;
    
    chassis_fdcan3_send_data_0x1FF[0] = (uint8_t)(motor1 >> 8);
    chassis_fdcan3_send_data_0x1FF[1] = (uint8_t)(motor1 & 0xFF);
    chassis_fdcan3_send_data_0x1FF[2] = (uint8_t)(motor2 >> 8);
    chassis_fdcan3_send_data_0x1FF[3] = (uint8_t)(motor2 & 0xFF);
    chassis_fdcan3_send_data_0x1FF[4] = (uint8_t)(motor3 >> 8);
    chassis_fdcan3_send_data_0x1FF[5] = (uint8_t)(motor3 & 0xFF);
    chassis_fdcan3_send_data_0x1FF[6] = (uint8_t)(motor4 >> 8);
    chassis_fdcan3_send_data_0x1FF[7] = (uint8_t)(motor4 & 0xFF);
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &chassis_tx_message_fdcan3_0x1FF, chassis_fdcan3_send_data_0x1FF);
}


// 电机数据处理函数（需要在FDCAN回调中调用）
void Motor3508_Process_Rx_Message(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData)
{
    if(hfdcan->Instance == FDCAN1)
    {
        if(RxHeader->Identifier & (2 << 8))
        {
            switch(RxHeader->Identifier)
            {
            case 0x201:
                motor[0].NowSpeed = (int16_t)(RxData[2] << 8 | RxData[3]);
                motor[0].LastEnconder = motor[0].NowEnconder;
                motor[0].NowEnconder = (uint16_t)(RxData[0] << 8 | RxData[1]);
                motor[0].current = (int16_t)(RxData[4] << 8 | RxData[5]);
                position_accumulation(0);
                break;
            case 0x202:
                motor[1].NowSpeed = (int16_t)(RxData[2] << 8 | RxData[3]);
                motor[1].LastEnconder = motor[1].NowEnconder;
                motor[1].NowEnconder = (uint16_t)(RxData[0] << 8 | RxData[1]);
                motor[1].current = (int16_t)(RxData[4] << 8 | RxData[5]);
                position_accumulation(1);
                break;
            case 0x203:
                motor[2].NowSpeed = (int16_t)(RxData[2] << 8 | RxData[3]);
                motor[2].LastEnconder = motor[2].NowEnconder;
                motor[2].NowEnconder = (uint16_t)(RxData[0] << 8 | RxData[1]);
                motor[2].current = (int16_t)(RxData[4] << 8 | RxData[5]);
                position_accumulation(2);
                break;
            case 0x204:
                motor[3].NowSpeed = (int16_t)(RxData[2] << 8 | RxData[3]);
                motor[3].LastEnconder = motor[3].NowEnconder;
                motor[3].NowEnconder = (uint16_t)(RxData[0] << 8 | RxData[1]);
                motor[3].current = (int16_t)(RxData[4] << 8 | RxData[5]);
                position_accumulation(3);
                break;
            case 0x205:
                motor[4].NowSpeed = (int16_t)(RxData[2] << 8 | RxData[3]);
                motor[4].LastEnconder = motor[4].NowEnconder;
                motor[4].NowEnconder = (uint16_t)(RxData[0] << 8 | RxData[1]);
                position_accumulation(4);
                break;
            case 0x206:
                motor[5].NowSpeed = (int16_t)(RxData[2] << 8 | RxData[3]);
                motor[5].LastEnconder = motor[5].NowEnconder;
                motor[5].NowEnconder = (uint16_t)(RxData[0] << 8 | RxData[1]);
                position_accumulation(5);
                break;
            case 0x207:
                motor[6].NowSpeed = (int16_t)(RxData[2] << 8 | RxData[3]);
                motor[6].LastEnconder = motor[6].NowEnconder;
                motor[6].NowEnconder = (uint16_t)(RxData[0] << 8 | RxData[1]);
                position_accumulation(6);
                break;
            case 0x208:
                motor[7].NowSpeed = (int16_t)(RxData[2] << 8 | RxData[3]);
                motor[7].LastEnconder = motor[7].NowEnconder;
                motor[7].NowEnconder = (uint16_t)(RxData[0] << 8 | RxData[1]);
                position_accumulation(7);
                break;
            default:
                // printf("未知ID:%x ", RxHeader->Identifier);
                break;
            }
        }
    }
    else if(hfdcan->Instance == FDCAN3)
    {
        if(RxHeader->Identifier & (2 << 8))
        {
            switch(RxHeader->Identifier)
            {
            case 0x201:
                motor[8].NowSpeed = (int16_t)(RxData[2] << 8 | RxData[3]);
                motor[8].LastEnconder = motor[8].NowEnconder;
                motor[8].NowEnconder = (uint16_t)(RxData[0] << 8 | RxData[1]);
                position_accumulation(8);
                break;
            case 0x202:
                motor[9].NowSpeed = (int16_t)(RxData[2] << 8 | RxData[3]);
                motor[9].LastEnconder = motor[9].NowEnconder;
                motor[9].NowEnconder = (uint16_t)(RxData[0] << 8 | RxData[1]);
                position_accumulation(9);
                break;
            case 0x203:
                motor[10].NowSpeed = (int16_t)(RxData[2] << 8 | RxData[3]);
                motor[10].LastEnconder = motor[10].NowEnconder;
                motor[10].NowEnconder = (uint16_t)(RxData[0] << 8 | RxData[1]);
                position_accumulation(10);
                break;
            case 0x204:
                motor[11].NowSpeed = (int16_t)(RxData[2] << 8 | RxData[3]);
                motor[11].LastEnconder = motor[11].NowEnconder;
                motor[11].NowEnconder = (uint16_t)(RxData[0] << 8 | RxData[1]);
                position_accumulation(11);
                break;
            default:

            break;
            }
        }
    }
}


/**
 * 设置云台的角度
 * @param angle_pitch 俯仰角度，单位为度，向前为正
 * @param angle_yaw 偏航角度，单位为度,逆时针是负
 */
void set_yuntai_angle(float angle_pitch , float angle_yaw)
{

    // 将角度从度转换为弧度
    float angle_pitch_temp = (angle_pitch+100) * CUR_TO_RADS;  // 俯仰角度转换为弧度
    float angle_yaw_temp = angle_yaw * CUR_TO_RADS;     // 偏航角度转换为弧度

    
    motor[9].ExpectAngle = -angle_pitch_temp; 
    motor[8].ExpectAngle = angle_yaw_temp; 
}


void set_quzhua_angle(float angle)
{
    motor[11].ExpectAngle = -angle * CUR_TO_RADS_3508;
}
