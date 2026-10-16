import serial
import time

# ================= 配置参数 =================
COM_PORT = 'COM4'      # 请修改为你的实际串口号
BAUD_RATE = 1000000     # 幻尔HX-30HM 默认波特率
SERVO_IDS = [1, 2, 3, 4, 5, 6]  # 需要读取的舵机 ID 列表
# ============================================


def calculate_checksum(data_bytes):
    """
    计算校验和
    协议规定: Checksum = ~(ID + Length + Instruction + Params...) 取低8位
    """
    total = sum(data_bytes)
    checksum = (~total) & 0xFF
    return checksum


def read_servo_position(ser, servo_id):
    """
    发送读位置指令，解析返回的原始位置数值（0~4095）

    指令帧格式（发送）:
      0xFF 0xFF | ID | Length | Instruction | Address | ReadLen | Checksum
        - Length = 4（Instruction + Address + ReadLen 的字节数）
        - Instruction = 0x02（READ 指令）
        - Address = 0x38（位置寄存器起始地址）
        - ReadLen = 0x02（读取 2 字节）

    返回帧格式（接收）:
      0xFF 0xFF | ID | Length | Error | PosLow | PosHigh | Checksum
        - Error = 0 表示无错误
        - Position = (PosHigh << 8) | PosLow
    """
    length = 0x04
    instruction = 0x02  # READ 指令
    addr = 0x38         # 位置寄存器地址
    read_len = 0x02     # 读取 2 字节

    # 构造 payload：ID + Length + Instruction + Address + ReadLen
    payload = [servo_id, length, instruction, addr, read_len]
    checksum = calculate_checksum(payload)

    # 完整发送帧：两个 0xFF 帧头 + payload + 校验和
    tx_packet = bytearray([0xFF, 0xFF] + payload + [checksum])

    # 清空接收缓冲区，发送指令
    ser.flushInput()
    ser.write(tx_packet)
    time.sleep(0.005)  # 等待舵机响应

    # 接收舵机返回数据
    if ser.in_waiting >= 8:
        rx_data = ser.read(8)

        # 验证帧头
        if rx_data[0] == 0xFF and rx_data[1] == 0xFF:
            rx_id = rx_data[2]
            rx_length = rx_data[3]
            rx_error = rx_data[4]
            pos_low = rx_data[5]
            pos_high = rx_data[6]
            rx_checksum = rx_data[7]

            # 验证校验和（payload = ID + Length + Error + PosLow + PosHigh）
            rx_payload = rx_data[2:7]
            if calculate_checksum(rx_payload) == rx_checksum:
                if rx_error == 0:
                    # 将两个字节合成原始位置数值
                    position = (pos_high << 8) | pos_low
                    return position

    return None


def raw_to_angle(raw_pos):
    """将 0~4095 的原始位置数值映射为 0~360 度的物理角度"""
    if raw_pos is None:
        return None
    angle = (raw_pos / 4095.0) * 360.0
    return angle


def main():
    """主函数：循环读取 6 个舵机的角度并打印"""
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.05)
        print(f"成功打开串口 {COM_PORT}，波特率 {BAUD_RATE}")
    except Exception as e:
        print(f"打开串口失败: {e}")
        return

    print("开始读取舵机角度，按 Ctrl+C 停止...\n")
    try:
        while True:
            results = []
            for sid in SERVO_IDS:
                raw_pos = read_servo_position(ser, sid)
                angle = raw_to_angle(raw_pos)

                if angle is not None:
                    results.append(f"ID{sid}:{angle:6.1f}°")
                else:
                    results.append(f"ID{sid}:  N/A  ")

            # 打印一帧 6 个舵机的角度数据
            print(" | ".join(results), end="\r")
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\n已停止读取。")
    finally:
        ser.close()
        print("串口已关闭。")


if __name__ == '__main__':
    main()