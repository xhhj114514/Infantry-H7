#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
// #include "bsp_log.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *referee_daemon;		  // 裁判系统守护进程
static referee_info_t referee_info;			  // 裁判系统数据

// 创建命令映射表
static const JudgeCommandEntry_t judge_command_table[] = {
    // 基础命令 - 直接内存拷贝
    {ID_game_state,           &referee_info.GameState,           LEN_game_state,           NULL,                   "游戏状态"},
    {ID_game_result,          &referee_info.GameResult,          LEN_game_result,          NULL,                   "比赛结果"},
    {ID_game_robot_survivors, &referee_info.GameRobotHP,         LEN_game_robot_HP,        NULL,                   "机器人血量"},
    {ID_event_data,           &referee_info.EventData,           LEN_event_data,           NULL,                   "事件数据"},
    {ID_referee_warning,      &referee_info.RefereeWarning,      LEN_event_data,           NULL,                   "裁判警告"},
    {ID_game_robot_state,     &referee_info.GameRobotState,      LEN_game_robot_state,     NULL,                   "机器人状态"},
    {ID_power_heat_data,      &referee_info.PowerHeatData,       LEN_power_heat_data,      NULL,                   "功率热量"},
    {ID_game_robot_pos,       &referee_info.GameRobotPos,        LEN_game_robot_pos,       NULL,                   "机器人位置"},
    {ID_buff_musk,            &referee_info.BuffMusk,            LEN_buff_musk,            NULL,                   "buff状态"},
    {ID_robot_hurt,           &referee_info.RobotHurt,           LEN_robot_hurt,           NULL,                   "机器人伤害"},
    {ID_shoot_data,           &referee_info.ShootData,           LEN_shoot_data,           NULL,                   "射击数据"},
    {ID_projectile_allowance, &referee_info.ProjectileAllowance, LEN_projectile_allowance, NULL,                   "弹丸余量"},
    {ID_student_interactive,  &referee_info.ArmData,             LEN_receive_data,         NULL, 				"学生交互数据"},
};

static const uint16_t judge_command_count = sizeof(judge_command_table) / sizeof(JudgeCommandEntry_t);

// 内存拷贝处理函数
static void MemoryCopyHandler(uint8_t* src_data, void* dest_struct, uint16_t data_len)
{
    if (src_data != NULL && dest_struct != NULL&& data_len > 0) {
        memcpy(dest_struct, src_data, data_len);
    }
}

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
static void JudgeReadData(uint8_t *buff)
{
    if (buff == NULL) {
        return;
    }
    // 写入帧头数据
    memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);
    // 判断帧头起始字节
    if (buff[SOF] != REFEREE_SOF) {
        return;
    }
    // 帧头CRC8校验
    if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) != TRUE) {
        return;
    }
    // 计算完整帧长度
    uint16_t judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
    // 帧尾CRC16校验
    if (Verify_CRC16_Check_Sum(buff, judge_length) != TRUE) {
        return;
    }
    // 提取命令ID
    referee_info.CmdID = (buff[6] << 8 | buff[5]);
    // 使用表驱动查找并处理命令
    for (uint16_t i = 0; i < judge_command_count; i++) {
        if (judge_command_table[i].cmd_id == referee_info.CmdID) 
		{
			MemoryCopyHandler(buff + DATA_Offset, 
									judge_command_table[i].data_struct,
									judge_command_table[i].data_len);
            break;
        }
    }
    // 处理多帧数据（递归调用）
    uint8_t* next_frame_ptr = buff + sizeof(xFrameHeader) + LEN_CMDID + 
                             referee_info.FrameHeader.DataLength + LEN_TAIL;
    if (*next_frame_ptr == REFEREE_SOF) {
        JudgeReadData(next_frame_ptr);
    }
}

/*裁判系统串口接收回调函数,解析数据 */
static void RefereeRxCallback()
{
	DaemonReload(referee_daemon);
	JudgeReadData(referee_usart_instance->recv_buff);
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
	USARTServiceInit(referee_usart_instance);
	// LOGWARNING("[rm_ref] lost referee data");
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = RefereeRxCallback;
	conf.usart_handle = referee_usart_handle;
	conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
	referee_usart_instance = USARTRegister(&conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback = RefereeLostCallback,
		.owner_id = referee_usart_instance,
		.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
	};
	referee_daemon = DaemonRegister(&daemon_conf);

	return &referee_info;
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
	USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	osDelay(115);
}
