import serial
import time

# ================= 配置参数 =================
TEACHER_COM = 'COM3'       # 教师机串口号
TEACHER_BAUD = 1000000      # 教师机波特率
STUDENT_COM = 'COM4'       # 学生机串口号
STUDENT_BAUD = 1000000     # 学生机波特率

SERVO_IDS = [1, 2, 3, 4, 5, 6]  # 舵机 ID 列表
POLL_INTERVAL = 0.03             # 轮询间隔（秒）

# 学生机各关节物理安全脉冲范围（防止碰撞/损坏）
# 教师机读数超出此范围时会被钳制到边界
STUDENT_ARM_LIMITS = {
    1: {"min": 835,  "max": 3216},
    2: {"min": 963,  "max": 3160},
    3: {"min": 895,  "max": 3172},
    4: {"min": 940,  "max": 3250},
    5: {"min": 965,  "max": 3066},
    6: {"min": 1975, "max": 3098},
}
# ============================================


def calculate_checksum(data_bytes):
    """
    计算校验和: Checksum = ~(ID + Length + Instruction + Params...) & 0xFF
    """
    total = sum(data_bytes)
    return (~total) & 0xFF


# ==================== 教师机：读取 ====================

def read_servo_position(ser, servo_id):
    """
    读取单个舵机的当前位置（原始值 0~4095）
    协议: 0xFF 0xFF | ID | 0x04 | 0x02(READ) | 0x38 | 0x02 | Checksum
    """
    instruction = 0x02  # READ 指令
    addr = 0x38         # 当前位置寄存器地址
    read_len = 0x02     # 读取 2 字节
    length = 0x04

    payload = [servo_id, length, instruction, addr, read_len]
    checksum = calculate_checksum(payload)
    tx_packet = bytearray([0xFF, 0xFF] + payload + [checksum])

    ser.flushInput()
    ser.write(tx_packet)
    time.sleep(0.005)

    if ser.in_waiting >= 8:
        rx_data = ser.read(8)
        # 应答帧头: 教师机返回 0xFF 0xFF（与协议文档的 0xFF 0xF5 不同）
        if rx_data[0] == 0xFF and rx_data[1] == 0xFF:
            rx_payload = rx_data[2:7]
            if calculate_checksum(rx_payload) == rx_data[7]:  #用收到的数据重新计算一次 checksum，看看是不是和舵机发来的 checksum 一样。
                if rx_data[4] == 0:  # Error = 0
                    pos_low = rx_data[5]
                    pos_high = rx_data[6]
                    return (pos_high << 8) | pos_low #把高字节和低字节重新拼成一个完整的位置数字
    return None


def read_all_servo_positions(ser, servo_ids):
    """读取所有舵机位置，返回 {servo_id: position} 字典"""
    positions = {}
    for sid in servo_ids:
        pos = read_servo_position(ser, sid)
        positions[sid] = pos
    return positions
#整体意思把机械臂所有舵机的位置一个一个读取出来，然后整理成一张“舵机编号 → 位置”的表。

# ==================== 学生机：写入 ====================

def write_servo_position(ser, servo_id, position, move_time=100):
    """
    向指定舵机写入目标位置
    协议: 0xFF 0xFF | ID | 0x07 | 0x03(WRITE) | 0x2A | pos_low pos_high | time_low time_high | Checksum
    """
    instruction = 0x03  # WRITE 指令
    addr = 0x2A         # 目标位置寄存器地址

    pos_low = position & 0xFF #& 0xFF的意思是取出低8位
    pos_high = (position >> 8) & 0xFF
    time_low = move_time & 0xFF
    time_high = (move_time >> 8) & 0xFF

    length = 0x07

    payload = [servo_id, length, instruction, addr,
               pos_low, pos_high, time_low, time_high]
    checksum = calculate_checksum(payload)

    tx_packet = bytearray([0xFF, 0xFF] + payload + [checksum])

    ser.write(tx_packet)
#整体意思：告诉某一个舵机：“请在指定时间内，移动到我给你的目标位置。

def clamp_position(sid, raw_pos):
    """将原始位置钳制到学生机安全范围内"""
    limits = STUDENT_ARM_LIMITS.get(sid)
    if limits is None:
        return raw_pos
    return max(limits["min"], min(limits["max"], raw_pos))#把位置强制限制在最小值和最大值之间。


# ==================== 格式化输出 ====================

def raw_to_angle(raw_pos):
    """将 0~4095 原始值转换为 0~360 角度"""
    if raw_pos is None:
        return None
    return (raw_pos / 4095.0) * 360.0


def format_positions(positions):
    """格式化所有舵机位置为一行字符串"""
    parts = []#准备一个空列表，用来存储每个舵机的位置信息
    for sid in SERVO_IDS:#遍历每个舵机
        pos = positions.get(sid)#取出它的位置
        if pos is None:
            parts.append(f"ID{sid}:  N/A  ")#如果没读到→ 写 N/A，append表示添加到列表末尾
        else:
            angle = raw_to_angle(pos)#如果读到了→ 转成角度→ 检查安全范围
            clamped = clamp_position(sid, pos)
            if pos != clamped:#如果超范围→ 写 [钳制后的角度]，否则写 普通角度
                parts.append(f"ID{sid}:{angle:5.1f}°[{clamped}]")
            else:
                parts.append(f"ID{sid}:{angle:5.1f}°")
    return " | ".join(parts)


# ==================== 主循环 ====================

def main():
    # --- 打开教师机串口 ---
    try:
        teacher_ser = serial.Serial(TEACHER_COM, TEACHER_BAUD, timeout=0.05)
        print(f"[教师机] 已连接 {TEACHER_COM} (baud={TEACHER_BAUD})")
    except Exception as e:
        print(f"[教师机] 串口打开失败: {e}")
        return

    # --- 打开学生机串口 ---
    try:
        student_ser = serial.Serial(STUDENT_COM, STUDENT_BAUD, timeout=0.05)
        print(f"[学生机] 已连接 {STUDENT_COM} (baud={STUDENT_BAUD})")
    except Exception as e:
        print(f"[学生机] 串口打开失败: {e}")
        teacher_ser.close()
        return

    print("\n遥操作已启动 — 教师机角度 → 学生机")
    print("按 Ctrl+C 停止...\n")

    last_printed = 0
    read_errors = 0

    try:
        while True:
            # 1. 读取教师机所有舵机位置
            positions = read_all_servo_positions(teacher_ser, SERVO_IDS)

            # 2. 统计读取成功数
            success_count = sum(1 for p in positions.values() if p is not None)

            if success_count == 0:
                read_errors += 1
                if read_errors > 10:
                    print("[警告] 教师机连续读取失败，请检查连接")
                    read_errors = 0
                time.sleep(POLL_INTERVAL)
                continue
            read_errors = 0

            # 3. 钳制到安全范围并写入学生机
            for sid in SERVO_IDS:
                teacher_pos = positions.get(sid)
                if teacher_pos is not None:
                    target_pos = clamp_position(sid, teacher_pos)
                    write_servo_position(student_ser, sid, target_pos,
                                         move_time=int(POLL_INTERVAL * 1000))
                    time.sleep(0.003)  # 短暂间隔避免总线拥塞

            # 4. 每秒打印一次状态
            now = time.time()
            if now - last_printed >= 1.0:
                print("\r" + format_positions(positions), end="")
                last_printed = now

            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        print("\n\n遥操作已停止。")

    finally:
        teacher_ser.close()
        student_ser.close()
        print("串口已关闭。")


if __name__ == '__main__':
    main()
