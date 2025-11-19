# master_machine

## 用于与bsp层的usart交互的指针
```c
// 定义一个小电脑 看门狗 实例指针，用来和bsp层的daemon 联系
static DaemonInstance *minipc_daemon_instance;
// 定义一个小电脑串口实例指针，用来和bsp层的usart联系
static USARTInstance *minipc_usart_instance;
```
## 离线回调函数
```c
/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
  */
static void VisionOfflineCallback()
{
    USARTServiceInit(minipc_usart_instance);      
}
```c
/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 */
static void DecodeMinpc()
{
    DaemonReload(minipc_daemon_instance); // 喂狗
get_protocol_info_vision(minipc_usart_instance->recv_buff,&minipc_recv_data);
}

Minipc_Recv_s *minipcInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeMinpc;
    conf.recv_buff_size = Minipc_Recv_sIZE;
    conf.usart_handle = _handle;
    minipc_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = minipc_usart_instance,
        .reload_count = 10,
    };
    minipc_daemon_instance = DaemonRegister(&daemon_conf);

    return &minipc_recv_data;
}
```

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void SendMinipcData()
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    static uint8_t send_buff[Minipc_Send_sIZE];
    static uint16_t tx_len;
    get_protocol_send_Vision_data(&minipc_send_data, 1, send_buff, &tx_len);
    USARTSend(minipc_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); 
}


```