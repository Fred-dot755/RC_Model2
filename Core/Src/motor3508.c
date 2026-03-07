#include "motor3508.h" 

DJmotor_struct motor[8];

// 底盘电机控制数据结构
uint8_t chassis_fdcan1_send_data_0x200[8] = {0};
FDCAN_TxHeaderTypeDef chassis_tx_message_fdcan1_0x200;

uint8_t chassis_fdcan1_send_data_0x1FF[8] = {0};
FDCAN_TxHeaderTypeDef chassis_tx_message_fdcan1_0x1FF;

uint8_t chassis_fdcan2_send_data_0x200[8] = {0};
FDCAN_TxHeaderTypeDef chassis_tx_message_fdcan2_0x200;

uint8_t chassis_fdcan2_send_data_0x1FF[8] = {0};
FDCAN_TxHeaderTypeDef chassis_tx_message_fdcan2_0x1FF;

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
                position_accumulation(0);
                break;
            case 0x202:
                break;
            case 0x203:
                break;
            case 0x204:
                break;
            default:
                // printf("未知ID:%x ", RxHeader->Identifier);
                break;
            }
        }
    }
    else if(hfdcan->Instance == FDCAN2)
    {
        if(RxHeader->Identifier & (2 << 8))
        {
            switch(RxHeader->Identifier)
            {
            case 0x201:
                break;
            case 0x202:
                break;
            case 0x203:
                break;
            case 0x204:
                break;
            case 0x205:
                break;
            case 0x206:
                break;
            case 0x207:
                break;
            case 0x208:
                break;
            default:
                // printf("未知ID:%x ", RxHeader->Identifier);
                break;
            }
        }
    }
}


