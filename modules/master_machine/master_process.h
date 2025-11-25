#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#define Minipc_Recv_sIZE 18u // 当前为固定值,36字节
#define Minipc_Send_sIZE 36u

#pragma pack(1)
typedef struct
{
	struct
    {
		uint8_t header;  // 帧头，固定为0x5A
		float yaw;       // 需要云台转动的相对 yaw 角
		float pitch;     // 需要云台转动的相对 pitch 角
		uint8_t shoot;     // 物体距离(shoot)
		int32_t match;  // 上位机时间
		uint16_t checksum; // 校验和
	}Vision;
	int32_t TCNT;
	int32_t TCNTLast;
	int32_t FailCNT;
	uint8_t FailFlag;
} __attribute__((packed)) Minipc_Recv_s;

typedef enum
{
	COLOR_BLUE = 1,
	COLOR_RED = 0,
} Enemy_Color_e;

typedef struct
{
	struct
	{
		uint8_t header;  // 帧头，固定为0x5A
		uint8_t detect_color;
		float roll;
		float pitch;
		float yaw;
		int32_t match;  // 上位机时间
		uint16_t checksum; // 校验和
	}Vision;

} __attribute__((packed)) Minipc_Send_s;

#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Minipc_Recv_s *minipcInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 *
 */
void SendMinipcData();

/*更新发送数据帧，并计算发送数据帧长度*/
void get_protocol_send_Vision_data(
                            Minipc_Send_s *tx_data,          // 待发送的float数据
                            uint8_t float_length,    // float的数据长度
                            uint8_t *tx_buf,         // 待发送的数据帧
                            uint16_t *tx_buf_len) ;   // 待发送的数据帧长度



void get_protocol_info_vision(uint8_t *rx_buf, 
                           Minipc_Recv_s *recv_data);



#endif // !MASTER_PROCESS_H