import serial
import time

# ================= 配置参数 =================
COM_PORT = 'COM3'      # 请修改为你的实际串口号
BAUD_RATE = 1000000    # 协议文档默认波特率为 1Mbps
SERVO_IDS = [1, 2, 3, 4, 5, 6] # 需要读取的舵机 ID 列表
# ============================================

def calculate_checksum(data_bytes):
    """
    计算校验和：~ (ID + Length + Instruction + Parameter 1 + ... Parameter N) 取最低字节 
    """
    return (~sum(data_bytes)) & 0xFF

def read_servo_position(ser, servo_id):
    """
    读取单个舵机的原始位置数值
    """
    length = 0x04       
    instruction = 0x02  
    addr = 0x38         
    read_len = 0x02     
    
    payload = [servo_id, length, instruction, addr, read_len]
    check_sum = calculate_checksum(payload)
    tx_packet = bytearray([0xFF, 0xFF] + payload + [check_sum])
    
    ser.flushInput()  
    ser.write(tx_packet)
    time.sleep(0.005) 
    
    if ser.in_waiting >= 8: 
        rx_data = ser.read(8)
        
        if rx_data[0] == 0xFF and rx_data[1] == 0xFF:
            rx_error = rx_data[4] 
            pos_low = rx_data[5]
            pos_high = rx_data[6]
            
            rx_payload = rx_data[2:7]
            if calculate_checksum(rx_payload) == rx_data[7]:
                if rx_error == 0: 
                    position = (pos_high << 8) | pos_low
                    return position
                else:
                    return None
            else:
                return None
    return None

def raw_to_angle(raw_pos):
    """将 0~4095 的原始位置数值转换为 0~360 的物理角度"""
    if raw_pos is None:
        return None
    # 转换公式：(原始数值 / 4095) * 360
    angle = (raw_pos / 4095.0) * 360.0
    return angle

def main():
    try:
        # 打开串口，设置超时时间防止死等
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.05)
        print(f"成功打开串口 {COM_PORT}，波特率 {BAUD_RATE}")
    except Exception as e:
        print(f"打开串口失败: {e}")
        return

    try:
        print("开始扫描舵机角度，按 Ctrl+C 停止...\n")
        while True:
            positions = []
            for sid in SERVO_IDS:
                # 1. 拿到 0~4095 的原始机器数值
                raw_pos = read_servo_position(ser, sid)
                
                # 2. 转换为 0~360 度的物理角度
                angle = raw_to_angle(raw_pos)
                
                if angle is not None:
                    # 格式化输出：保留1位小数
                    positions.append(f"ID{sid}:{angle:5.1f}°")
                else:
                    positions.append(f"ID{sid}: N/A ")
            
            # 打印 6 个舵机的一帧数据
            print(" | ".join(positions), end="\r") 
            time.sleep(0.02) # 控制轮询频率
            
    except KeyboardInterrupt:
        print("\n\n已停止读取。")
    finally:
        ser.close()

# 这里才是程序的真正入口
if __name__ == '__main__':
    main()