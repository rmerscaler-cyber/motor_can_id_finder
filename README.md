# H7 电机 ID 读取教程（历史工具）

本目录保存曾在 H7 上使用的电机 ID 识别程序和源码快照，供以后复查协议、调试流程或恢复临时诊断固件时参考。扫描代码已从主工程移出，当前固件不包含扫描任务、未知地址探测接口或扫描专用 CAN 观察器。

## 识别结果

2026-10-03 的调试器读数已写入主工程的 `model/motor/motor_reg.c`，注册顺序为 A、B、C、yaw：

| 注册项 | 电机 | 总线 | 命令 ID | 反馈 ID |
| --- | --- | --- | ---: | ---: |
| A | 达妙 DM4310 | FDCAN1 | `0x001`（ESC_ID） | `0x007`（MST_ID） |
| B | 达妙 DM4310 | FDCAN1 | `0x002`（ESC_ID） | `0x008`（MST_ID） |
| C | 达妙 DM4310 | FDCAN1 | `0x003`（ESC_ID） | `0x009`（MST_ID） |
| yaw | 翎控 4010 v3 | FDCAN1 | `0x142` | `0x142` |

达妙的两个 ID 分别配置；4010 v3 按用户确认沿用 4005 协议，收发共用一个 ID，无需第二个 ID。当前主工程保留以上正常注册数据，PID 仍全部为零。

## 达妙 DM4310：逐台读取两个 ID

DM 测试程序通过达妙只读寄存器协议读取 `ESC_ID` 和 `MST_ID`。它不写参数、不使能电机，也不发送 MIT 控制命令。

### 操作流程

测试时每次只接一台 DM4310。A、B、C 的顺序由接线顺序确定；扫描器不能从未知 ID 判断物理电机在机构中的位置。

1. 将源码快照 `example/source/` 还原到匹配的 H7 工程并按当时的 CMake/RTOS 接线方式加入临时调试固件。
2. 在调试器 Watch 中添加 `robot_dm_scan` 和 `dm_id_scan_debug`。
3. 接入 A，设置 `robot_dm_scan.slot = 0`，再设置 `robot_dm_scan.start = 1`。等待 `dm_id_scan_debug.motor[0].found = 1`。
4. 保持 H7 运行，断开 A 并接入 B；设置 `slot = 1`、`start = 1`，等待 `motor[1].found = 1`。
5. 同样方式接入 C，设置 `slot = 2`、`start = 1`，等待 `motor[2].found = 1`。
6. 记录两个 ID 后停止临时固件，并把读数填入正式注册表。

每次扫描只连接一台电机；同一次调试会话中不要求三个 DM4310 同时接线。结果保存在 RAM，重启会清空。换线期间可保持固件运行，已完成的槽位会保留；重新扫描某槽会清除并更新该槽。

### Watch 字段

| 字段 | 含义 |
| --- | --- |
| `slot` | 下一次扫描槽位：0=A、1=B、2=C |
| `start` | 写 1 开始扫描，程序消费后清零 |
| `enabled` | 暂停/继续扫描 |
| `bus_mask` | 1/2/4 分别扫描 FDCAN1/2/3，7 扫描全部 |
| `first_id` / `last_id` | 候选 ESC_ID 范围 |
| `dm_id_scan_debug.motor[n].cmd_id` | 读出的 ESC_ID，作为正常注册项 `.cmd_id` |
| `dm_id_scan_debug.motor[n].can_id` | 读出的 MST_ID，作为正常注册项 `.can_id` |
| `dm_id_scan_debug.motor[n].found` | 两个寄存器均读到且无冲突时为 1 |
| `reply_can_id` | 参数回复帧实际使用的 CAN ID |
| `command_reply` / `feedback_reply` | 对应寄存器回复的原始 8 字节 |
| `status` | 0 空闲、1 等待稳定、2 扫描中、3 找到、4 配置错误、5 回复冲突 |

`0x7FF` 是参数访问帧的 CAN ID，不是最终电机命令 ID。参数请求数据为 `[ESC_ID低字节, ESC_ID高字节, 0x33, RID, 0, 0, 0, 0]`；RID 8 读取 ESC_ID，RID 7 读取 MST_ID。应答第 4 至 7 字节按小端序给出读回值。仅当两项回复均有效且一致时才确认结果。

如未收到结果，应检查 CAN 初始化、发送/接收计数、供电和接线，以及 1 Mbps 波特率。若仅需验证一个候选，可暂停扫描并令 `first_id = last_id` 后重新开始。

## 翎控 4010 v3：逐个试 Motor ID

当时的 LK 调试程序使用状态读取命令 `9A 00 00 00 00 00 00 00`，对候选地址逐个发帧。标准 CAN ID 为 `0x140 + Motor ID`，Motor ID 范围 1..32，发送与回复使用同一个 CAN ID。主工程不再含此自动扫描功能。

历史 Watch 名称为 `robot_lk_scan`、`lk_id_scan_debug` 和 `bsp_can_trace[0..2]`。可暂停扫描后通过 `manual_bus`、`manual_can_id` 手动试某一路和某个候选 ID；自动模式曾按 CAN1/2/3 和 Motor ID 递增扫描。看到 `found = 1` 后，`first_bus`、`first_motor_id`、`first_can_id` 及 `first_data` 保存首个有效回复。

本次实际读数为 FDCAN1、Motor ID 2，因此共用 ID 为 `0x142`。LK 不需要像 DM4310 那样填写两个 ID。

## 源码快照

`example/source/` 按当时工程相对路径保存了扫描应用、任务接线、motor 中间层探测接口、DM/LK 协议实现、H7 CAN/device 观察路径及对应启动/CMake 文件。它是用于教学和回溯的历史快照，不属于主工程的构建输入；移回另一个固件前应检查它的 CubeMX、HAL 和 FreeRTOS 版本是否匹配。

| 路径 | 内容 |
| --- | --- |
| `example/source/app/dm_id_application.c` 及同名头文件 | DM 三个槽位的扫描状态机和 Watch 结果 |
| `example/source/app/robot_application.c`、`robot_task.c` | LK 地址扫描、应用调度和 FreeRTOS 周期任务 |
| `example/source/model/motor/motor.c`、`motor.h`、`motor_typedef.h` | 协议无关的探测入口与回复收件箱 |
| `example/source/model/motor/motor_reg.c`、`DM_motor.c`、`lk_motor.c` | 扫描驱动绑定、DM 双寄存器读取和 LK `0x9A` 查询 |
| `example/source/device/driver/motor/motor_port.c`、`example/source/bsp/bsp_can.c` | 未知回复 CAN 帧的端口透传和原始观察 |
| `example/source/Core/Src/main.c`、`freertos.c`、CMake 文件 | 当时的初始化、任务和构建接线 |

大致调用关系：

```text
robot_task
  -> robot_application / dm_id_application
  -> motor_probe_* [motor.c]
  -> driver ops [DM_motor.c / lk_motor.c]
  -> motor_port [device]
  -> BSP CAN / 原始帧观察 [bsp]
```

达妙使用独立回复收件箱收集两个寄存器；翎控状态回复通过原始 CAN observer 观察。两者都不写入正常 `motor->fb` 物理反馈区。

协议来源：达妙只读参数帧格式参考 [dmBots motor-sdk 的 DM_CAN.py](https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)；翎控 ID 范围、命令和 CAN 参数按当时提供的电机资料整理。
