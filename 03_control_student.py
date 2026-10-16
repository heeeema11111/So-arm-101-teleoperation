import serial
import time
import threading
import tkinter as tk
from tkinter import ttk

# ================= 配置参数 =================
COM_PORT = 'COM4'       # 请修改为你的实际串口号
BAUD_RATE = 1000000     # HX-30HM 波特率（1Mbps）
SERVO_IDS = [1, 2, 3, 4, 5, 6]  # 需要控制的舵机 ID 列表
MID_POSITION = 2048     # 中位角度对应原始值 (0~4095)

# 各关节物理安全脉冲范围（超出可能碰撞或损坏机械臂）
ARM_LIMITS = {
    1: {"min": 835,  "max": 3216},
    2: {"min": 963,  "max": 3160},  # 实测值
    3: {"min": 895,  "max": 3172},
    4: {"min": 940,  "max": 3250},
    5: {"min": 965,  "max": 3066},
    6: {"min": 1975,  "max": 3098},
}
# ============================================


def calculate_checksum(data_bytes):
    """
    计算校验和
    协议规定: Checksum = ~(ID + Length + Instruction + Params...) & 0xFF
    """
    total = sum(data_bytes)
    checksum = (~total) & 0xFF
    return checksum


def write_servo_position(ser, servo_id, position, move_time=1000):
    """
    向指定舵机写入目标位置

    指令帧格式（发送）:
      0xFF 0xFF | ID | Length | Instruction | Address | Data... | Checksum
        - Instruction = 0x03（WRITE 指令）
        - Address    = 0x2A（目标位置寄存器地址）
        - Data       = position_low + position_high + time_low + time_high
        - Length     = N + 2 = 6

    参数:
        ser       : 串口对象
        servo_id  : 舵机 ID (1~253)
        position  : 目标位置原始值 (0~4095)
        move_time : 运动时间（毫秒），默认 1000ms
    """
    instruction = 0x03  # WRITE 指令
    addr = 0x2A         # 目标位置寄存器地址

    # 将 position 和 move_time 拆分为低字节和高字节
    pos_low = position & 0xFF
    pos_high = (position >> 8) & 0xFF
    time_low = move_time & 0xFF
    time_high = (move_time >> 8) & 0xFF

    # 参数: 地址 + 位置(2B) + 时间(2B)
    # N = 5 (地址1 + 位置2 + 时间2), Length = N + 2 = 7
    length = 0x07

    # payload: ID + Length + Instruction + Address + Data...
    payload = [servo_id, length, instruction, addr,
               pos_low, pos_high, time_low, time_high]
    checksum = calculate_checksum(payload)

    # 完整发送帧：帧头 + payload + 校验和
    tx_packet = bytearray([0xFF, 0xFF] + payload + [checksum])

    ser.flushInput()
    ser.write(tx_packet)
    time.sleep(0.005)  # 等待舵机处理

    # 读取舵机应答（可选）
    if ser.in_waiting >= 6:
        rx_data = ser.read(ser.in_waiting)
        # 验证应答帧头
        if len(rx_data) >= 6:
            if rx_data[0] == 0xFF and rx_data[1] == 0xF5:
                return rx_data[4] == 0  # Error=0 表示成功
    return False


def torque_off(ser, servo_id):
    """
    关闭指定舵机的扭矩（允许自由转动）

    向扭矩开关寄存器 (0x28) 写入 0 来卸载扭矩。
    协议：
      指令帧: 0xFF 0xFF | ID | 0x04 | 0x03(WRITE) | 0x28 | 0x00 | Checksum
      - 0x28: 扭矩开关寄存器地址
      - 0x00: 扭矩关闭
    """
    instruction = 0x03  # WRITE
    addr = 0x28         # 扭矩开关寄存器
    length = 0x04

    payload = [servo_id, length, instruction, addr, 0x00]
    checksum = calculate_checksum(payload)

    tx_packet = bytearray([0xFF, 0xFF] + payload + [checksum])
    ser.write(tx_packet)
    time.sleep(0.003)  # 短暂等待


def torque_off_all(ser, servo_ids):
    """关闭所有舵机扭矩"""
    for sid in servo_ids:
        torque_off(ser, sid)


def angle_to_raw(angle):
    """将 0~360 度的角度转换为 0~4095 的原始位置值"""
    angle = max(0, min(360, angle))  # 限制在 0~360 范围内
    raw = int((angle / 360.0) * 4095)
    return raw


def raw_to_angle(raw_pos):
    """将 0~4095 的原始位置值转换为 0~360 度的角度"""
    if raw_pos is None:
        return None
    return (raw_pos / 4095.0) * 360.0


def read_servo_position(ser, servo_id):
    """
    读取单个舵机的当前位置（原始值 0~4095）
    用于验证写入结果
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
        if rx_data[0] == 0xFF and rx_data[1] == 0xF5:
            rx_payload = rx_data[2:7]
            if calculate_checksum(rx_payload) == rx_data[7]:
                if rx_data[4] == 0:  # Error = 0
                    pos_low = rx_data[5]
                    pos_high = rx_data[6]
                    position = (pos_high << 8) | pos_low
                    return position
    return None


def set_all_servos_angle(ser, angles, move_time=1000):
    """
    设置所有舵机的角度（角度单位：度）

    参数:
        ser       : 串口对象
        angles    : 字典，{舵机ID: 角度(0~360)}，如 {1: 180, 2: 90}
        move_time : 运动时间（毫秒）
    """
    for sid, angle in angles.items():
        raw_pos = angle_to_raw(angle)
        write_servo_position(ser, sid, raw_pos, move_time)


def set_all_servos_raw(ser, positions, move_time=1000):
    """
    设置所有舵机的位置（原始值 0~4095）

    参数:
        ser       : 串口对象
        positions : 字典，{舵机ID: 原始值(0~4095)}
        move_time : 运动时间（毫秒）
    """
    for sid, raw_pos in positions.items():
        write_servo_position(ser, sid, raw_pos, move_time)


def demo_all_mid(ser):
    """示例：所有舵机回到中位 (2048)"""
    print("\n>>> 设置所有舵机到中位 (2048 / 180°) ...")
    for sid in SERVO_IDS:
        write_servo_position(ser, sid, MID_POSITION, move_time=1000)
    time.sleep(1.5)  # 等待运动完成

    print("读取当前位置：")
    for sid in SERVO_IDS:
        raw = read_servo_position(ser, sid)
        angle = raw_to_angle(raw)
        if angle is not None:
            print(f"  ID{sid}: {angle:.1f}° (raw={raw})")
        else:
            print(f"  ID{sid}: 读取失败")


def print_servo_angles(ser):
    """读取并打印所有舵机的当前角度"""
    results = []
    for sid in SERVO_IDS:
        raw = read_servo_position(ser, sid)
        angle = raw_to_angle(raw)
        if angle is not None:
            results.append(f"ID{sid}:{angle:6.1f}°")
        else:
            results.append(f"ID{sid}:  N/A  ")
    print(" | ".join(results))


def interactive_control(ser):
    """
    交互式控制模式：
      通过键盘输入来控制各个舵机
    """
    print("\n" + "=" * 60)
    print("  交互式舵机控制")
    print("  输入格式:")
    print("    1) 单独控制:  <舵机ID> <角度>    例如: 1 180")
    print("    2) 全部归中:  mid")
    print("    3) 读取角度:  read")
    print("    4) 退出程序:  quit 或 q")
    print("=" * 60 + "\n")

    while True:
        try:
            cmd = input(">>> ").strip().lower()

            if cmd in ('quit', 'q', 'exit'):
                print("退出控制程序。")
                break

            if cmd == 'mid':
                for sid in SERVO_IDS:
                    write_servo_position(ser, sid, MID_POSITION, move_time=1000)
                print("  所有舵机已归中 (2048 / 180°)")
                time.sleep(1.2)
                print_servo_angles(ser)
                continue

            if cmd == 'read':
                print_servo_angles(ser)
                continue

            # 解析 "ID 角度" 格式
            parts = cmd.split()
            if len(parts) == 2:
                sid = int(parts[0])
                angle = float(parts[1])

                if sid not in SERVO_IDS:
                    print(f"  错误: 舵机 ID {sid} 不在配置列表 {SERVO_IDS} 中")
                    continue

                raw_pos = angle_to_raw(angle)
                print(f"  发送: ID{sid} -> {angle:.1f}° (raw={raw_pos})")
                write_servo_position(ser, sid, raw_pos, move_time=1000)
                time.sleep(0.3)
                # 读取确认
                raw = read_servo_position(ser, sid)
                cur_angle = raw_to_angle(raw)
                if cur_angle is not None:
                    print(f"  当前: ID{sid} -> {cur_angle:.1f}°")
                else:
                    print(f"  当前: 读取失败")
            else:
                print("  无效输入，请重试。")

        except ValueError:
            print("  格式错误，请输入数字。")
        except KeyboardInterrupt:
            print("\n已退出交互模式。")
            break


# ======================== GUI 控制面板 ========================

JOINT_NAMES = [
    "关节 1 (ID1)",
    "关节 2 (ID2)",
    "关节 3 (ID3)",
    "关节 4 (ID4)",
    "关节 5 (ID5)",
    "关节 6 (ID6)",
]

SLIDER_MOVE_TIME = 200  # 滑块拖动时的运动时间（ms），较小值保证响应快


class ServoPanel:
    """舵机控制 GUI 面板"""

    def __init__(self, ser):
        self.ser = ser
        self.lock = threading.Lock()
        self.pending_jobs = {}  # {servo_id: after_id}  用于滑块防抖

        self.root = tk.Tk()
        self.root.title("机械臂舵机控制面板")
        self.root.resizable(True, True)

        # ---------- 主框架 ----------
        main_frame = ttk.Frame(self.root, padding="15 15 15 10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # ---------- 标题 ----------
        title = ttk.Label(main_frame, text="舵机控制面板",
                          font=("Microsoft YaHei", 14, "bold"))
        title.pack(pady=(0, 15))

        # ---------- 6 行滑块 ----------
        self.sliders = {}       # {servo_id: tk.Scale}
        self.value_labels = {}  # {servo_id: ttk.Label}

        for i, sid in enumerate(SERVO_IDS):
            limits = ARM_LIMITS[sid]
            range_min = limits["min"]
            range_max = limits["max"]
            center = MID_POSITION

            row_frame = ttk.Frame(main_frame)
            row_frame.pack(fill=tk.X, pady=4)

            # 关节名称标签（固定宽度对齐）
            name_label = ttk.Label(row_frame, text=JOINT_NAMES[i],
                                   width=16, anchor=tk.W,
                                   font=("Microsoft YaHei", 10))
            name_label.pack(side=tk.LEFT, padx=(0, 8))

            # 脉冲值显示标签
            val_label = ttk.Label(row_frame, text=str(center),
                                  width=6, anchor=tk.CENTER,
                                  font=("Consolas", 11, "bold"),
                                  foreground="#0066CC")
            val_label.pack(side=tk.RIGHT, padx=(8, 0))
            self.value_labels[sid] = val_label

            # 水平滑块（范围受 ARM_LIMITS 约束）
            slider = tk.Scale(
                row_frame,
                from_=range_max, to=range_min,   # 左小右大
                orient=tk.HORIZONTAL,
                length=380,
                resolution=1,
                showvalue=False,  # 不显示默认数值，用自定义标签
                command=lambda v, s=sid: self._on_slider_change(s, v),
            )
            slider.set(center)
            slider.pack(side=tk.RIGHT, fill=tk.X, expand=True)
            self.sliders[sid] = slider

        # ---------- 分隔线 ----------
        ttk.Separator(main_frame, orient=tk.HORIZONTAL).pack(
            fill=tk.X, pady=(15, 12))

        # ---------- 底部按钮区 ----------
        bottom_frame = ttk.Frame(main_frame)
        bottom_frame.pack(fill=tk.X)

        self.status_label = ttk.Label(
            bottom_frame, text="就绪",
            font=("Microsoft YaHei", 9), foreground="gray")
        self.status_label.pack(side=tk.LEFT, pady=(0, 5))

        reset_btn = tk.Button(
            bottom_frame,
            text="全部回中 (Reset All)",
            font=("Microsoft YaHei", 11, "bold"),
            bg="#E74C3C", fg="white",
            activebackground="#C0392B", activeforeground="white",
            relief=tk.RAISED, bd=2,
            padx=20, pady=6,
            cursor="hand2",
            command=self._on_reset_all,
        )
        reset_btn.pack(side=tk.RIGHT, pady=(0, 5))

        # ---------- 启动时读取舵机真实位置并同步滑块 ----------
        self.root.after(100, self._sync_sliders_to_actual)

        # ---------- 窗口居中 ----------
        self.root.update_idletasks()
        w = self.root.winfo_width()
        h = self.root.winfo_height()
        x = (self.root.winfo_screenwidth() - w) // 2
        y = (self.root.winfo_screenheight() - h) // 2
        self.root.geometry(f"+{x}+{y}")

        # ---------- 关闭窗口时清理 ----------
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ==================== 启动同步 ====================

    def _sync_sliders_to_actual(self):
        """启动时读取舵机真实位置，将滑块同步到实际值（不发送指令）"""
        self._set_status("正在读取当前舵机位置...")
        t = threading.Thread(target=self._sync_thread, daemon=True)
        t.start()

    def _sync_thread(self):
        """后台线程：读取所有舵机位置并更新滑块"""
        positions = {}
        for sid in SERVO_IDS:
            pos = read_servo_position(self.ser, sid)
            if pos is not None:
                # 钳制到安全范围
                limits = ARM_LIMITS[sid]
                pos = max(limits["min"], min(limits["max"], pos))
                positions[sid] = pos
            else:
                positions[sid] = MID_POSITION  # 读取失败则保持默认中位

        def _update_ui():
            for sid, pos in positions.items():
                if sid in self.sliders:
                    self.sliders[sid].set(pos)
                    self.value_labels[sid].config(text=str(pos))
            self._set_status("就绪（已同步当前舵机位置）")

        self.root.after(0, _update_ui)

    # ==================== 滑块事件处理 ====================

    def _on_slider_change(self, servo_id, value_str):
        """滑块数值变化时触发（实时拖动）"""
        try:
            position = int(float(value_str))
        except ValueError:
            return

        # 更新脉冲值标签
        self.value_labels[servo_id].config(text=str(position))

        # 防抖：取消之前的待发送任务，延迟 50ms 发送
        if servo_id in self.pending_jobs:
            self.root.after_cancel(self.pending_jobs[servo_id])

        job_id = self.root.after(50, lambda: self._send_position(servo_id, position))
        self.pending_jobs[servo_id] = job_id

    def _send_position(self, servo_id, position):
        """在后台线程中发送舵机位置指令"""
        if servo_id in self.pending_jobs:
            del self.pending_jobs[servo_id]

        self._set_status(f"发送: ID{servo_id} → {position}")
        t = threading.Thread(target=self._write_thread, args=(servo_id, position),
                             daemon=True)
        t.start()

    def _write_thread(self, servo_id, position):
        """串口写入线程"""
        try:
            with self.lock:
                write_servo_position(self.ser, servo_id, position,
                                     move_time=SLIDER_MOVE_TIME)
        except Exception as e:
            self.root.after(0, lambda: self._set_status(f"错误: {e}"))

    # ==================== 全部回中 ====================

    def _on_reset_all(self):
        """将所有滑块归中并发送指令"""
        for sid in SERVO_IDS:
            self.sliders[sid].set(MID_POSITION)
            self.value_labels[sid].config(text=str(MID_POSITION))
        self._set_status("归中: 正在发送...")
        t = threading.Thread(target=self._reset_thread, daemon=True)
        t.start()

    def _reset_thread(self):
        """归中串口写入线程"""
        try:
            with self.lock:
                for sid in SERVO_IDS:
                    write_servo_position(self.ser, sid, MID_POSITION,
                                         move_time=800)
                    time.sleep(0.01)
        except Exception as e:
            self.root.after(0, lambda: self._set_status(f"错误: {e}"))
        else:
            self.root.after(0, lambda: self._set_status("全部回中完成"))

    # ==================== 状态与关闭 ====================

    def _set_status(self, text):
        """更新底部状态栏"""
        self.status_label.config(text=text)

    def _on_close(self):
        """关闭窗口时释放资源，先卸载所有舵机扭矩"""
        self._set_status("正在卸载扭矩...")
        self.root.update()
        try:
            with self.lock:
                torque_off_all(self.ser, SERVO_IDS)
        except Exception as e:
            print(f"卸载扭矩失败: {e}")
        self.root.destroy()

    def run(self):
        """启动 GUI 主循环"""
        self.root.mainloop()


def main():
    """主函数 — 打开串口并启动 GUI"""
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.05)
        print(f"成功打开串口 {COM_PORT}，波特率 {BAUD_RATE}")
    except Exception as e:
        print(f"打开串口失败: {e}")
        # 即使串口失败也尝试显示 GUI（仅用于演示）
        root = tk.Tk()
        root.title("错误")
        ttk.Label(root, text=f"串口打开失败:\n{e}",
                  font=("Microsoft YaHei", 11), foreground="red").pack(padx=30, pady=20)
        root.mainloop()
        return

    print("启动 GUI 控制面板...")
    panel = ServoPanel(ser)
    try:
        panel.run()
    finally:
        ser.close()
        print("串口已关闭。")


if __name__ == '__main__':
    main()