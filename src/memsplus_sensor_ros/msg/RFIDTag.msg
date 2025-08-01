std_msgs/Header header  # 时间戳与帧ID
uint8 status            # 读卡状态（0x00=非读卡状态，0x01=正在读卡状态）
uint8 rssi              # 信号强度（0x00-0x07，由弱到强）
uint64 tag_id           # 标签ID（4字节或8字节，根据配置）
bool read_success       # 读取成功标志（Result指示灯状态映射）