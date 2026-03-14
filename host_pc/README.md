# 轮腿机器人测试上位机

这是一个基于 **Python + PySide6 + pyqtgraph** 的串口测试上位机，用于配合 `wheel-leg-bot.ino` 的 0~9 测试菜单，对机器人硬件进行带图形界面的联机验证。

## 功能

- 串口连接 / 断开
- 一键发送测试命令 `0~9` 和 `h`
- 实时解析固件串口输出
- 实时波形显示：
  - 电池电压
  - IMU 姿态角
  - IMU 陀螺仪 / 加速度
  - 电机速度 / 位置
  - 编码器角度
- 原始日志窗口
- CSV 录制导出

## 已适配的固件输出

当前解析器已适配以下串口输出格式：

- `Battery Raw: ... | Filtered: ...`
- `Angle(XYZ): ... | Gyro(XYZ): ... | Acc(XYZ): ...`
- `Encoder1 Angle: ... Vel: ... | Encoder2 Angle: ... Vel: ...`
- `Torque(V) Target=... | M1 Angle: ... Vel: ... | M2 Angle: ... Vel: ...`
- `FOC Result -> M1:... M2:... | Ready:YES/NO`
- `Angle Target: ... | M1: ... | M2: ...`
- `Velocity Target: ... | M1: ... | M2: ...`
- `Stage:... | M1 Target:... Vel:... | M2 Target:... Vel:...`

## 安装

建议使用 Python 3.10+。

### 1. 安装依赖

在 `host_pc` 目录执行：

```bash
pip install -r requirements.txt
```

### 2. 启动

```bash
python main.py
```

Windows 下也可以直接双击：

```bash
run_gui.bat
```

## 使用流程

1. 烧录并启动 `wheel-leg-bot.ino`
2. 用 USB 将 ESP32 连接到电脑
3. 打开上位机，点击 **刷新串口**
4. 选择正确串口，波特率使用 `115200`
5. 点击 **连接串口**
6. 点击左侧测试按钮：
   - `1` 电池
   - `2` IMU
   - `3` 舵机
   - `4` 编码器
   - `5` FOC基础
   - `6` FOC校准
   - `7` 位置闭环
   - `8` 速度闭环
   - `9` 力矩扫描 / 单电机测试
7. 如需保存测试数据，点击 **开始录制 CSV**

## 注意事项

- `6/7/8/9` 涉及电机动作，测试前请抬起轮子或做好安全固定。
- 当前舵机库只有写入接口，没有回读接口，因此 GUI 不能显示舵机实际到位反馈。
- 当前“力矩测试”是 **电压力矩模式**，不是电流闭环力矩控制。

## 后续可扩展项

- 改成 JSON 串口协议，降低文本解析耦合
- 增加自动判定（PASS/FAIL）
- 增加测试报告导出
- 增加舵机回读支持
- 增加电流采样后实现真实力矩闭环测试
