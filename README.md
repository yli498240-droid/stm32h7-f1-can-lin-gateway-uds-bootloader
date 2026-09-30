# STM32H7/F1 CAN/LIN 车身网关与车窗 ECU 诊断及 A/B Bootloader 系统

## 项目简介

基于 STM32H743、STM32F103RCT6 和 STM32F103C8T6 构建三节点车身网络，通过真实 LIN 与 CAN 实现实体按键到车窗 ECU 的控制链路，并集成 UDS 诊断和 A/B OTA。H7 负责网关与诊断主机，RC 负责车窗控制和双槽 Bootloader，C8 负责按键采集与 LIN 事件上传。

## 系统拓扑

```text
C8 Button Node (STM32F103C8T6)
  └─ 实体按键 / LIN Slave
          │ 真实 LIN 19.2 kbps，USART3 + TJA1021
          ▼
H7 Gateway (STM32H743)
  ├─ LIN Master / ButtonEvent / Window Lock / NvM
  ├─ Diagnostic & OTA Host
  └─ CAN Gateway
          │ Classic CAN 500 kbps，11-bit
          ▼
RC Window ECU (STM32F103RCT6)
  ├─ AUTO / JOG 电机控制
  ├─ 编码器堵转与防夹
  ├─ UDS / ISO-TP
  └─ A/B Bootloader + 双副本 Metadata
```

RC 通过 CAN `0x101` 回报状态。

## 通信与命令

| CAN ID | 方向 | 用途 |
| --- | --- | --- |
| `0x100` | H7 → RC | Window Command |
| `0x101` | RC → H7 | Window Status |
| `0x7E0` | Tester → RC | UDS Request |
| `0x7E8` | RC → Tester | UDS Response |

`0x100` 命令值：`00` STOP、`01` AUTO OPEN、`02` AUTO CLOSE、`03` RESET、`04` JOG OPEN、`05` JOG CLOSE。

LIN 使用真实 TJA1021 收发器：`0x21` 由 H7 发布控制/ACK，`0x22` 由 H7 发 Header、C8 回传 Button Event + Sequence，`0x23` 由 H7 发 Header、C8 回传状态。H7 约每 20 ms 轮询 `0x22`；事件在 ACK 前由 C8 保留并重传。

## 诊断与 OTA

RC App 实现的 UDS SID：`0x10`、`0x11`、`0x14`、`0x19`、`0x22`、`0x27`、`0x31`、`0x34`、`0x36`、`0x37`。RC Boot 实现：`0x10`、`0x11`、`0x22`、`0x27`、`0x31`、`0x34`、`0x36`、`0x37`。App DID 包括 `F100`～`F103`，Boot DID `F1A0` 用于查询下载目标。OTA 数据经 ISO-TP 传输。

镜像 CRC 例程的 Request 格式为 `31 01 F0 01 CRC32[4 bytes]`（CRC32 按协议顺序传输）；成功响应为 `71 01 F0 01 00`，校验失败响应为 `71 01 F0 01 01`。这与安全解锁不是同一机制。

| RC Flash 区域 | 地址 |
| --- | --- |
| Boot | `0x08000000–0x08007FFF` |
| Slot A | `0x08008000–0x080237FF` |
| Slot B | `0x08023800–0x0803EFFF` |
| Metadata A | `0x0803F000–0x0803F7FF` |
| Metadata B | `0x0803F800–0x0803FFFF` |

新镜像先进入 Pending；Boot 校验 CRC32、Vector 和 Descriptor，记录启动尝试；App Confirm 后成为有效槽。连续未 Confirm 达到三次尝试上限时，回退到已有 confirmed 槽。

SecurityAccess 使用固定 Seed/Key 演示机制，公开仓库已移除具体常量。

## 车窗控制与可靠性

车窗堵转与防夹保护采用编码器判据。

- OPEN 堵转：启动宽限 500 ms；每 50 ms 采样，`encoder delta < 50` 连续 20 次则 STOP，不永久锁存。
- CLOSE 防夹：启动宽限 500 ms；`encoder delta < 80` 连续 3 次则反转 500 ms、STOP 并锁存；需 `RESET` 解除。

系统还实现 CAN Bus-Off 检测与恢复、RC IWDG、LIN Sleep/Remote Wake、镜像/Flash 边界校验。H7 的轻量 NvM 用双 Sector、CRC 与最后 Commit 的记录方式持久化 Window Lock 用户偏好。

软件结构参考 AUTOSAR Classic 的分层与模块职责设计，对通信、诊断、故障管理和非易失数据管理进行模块化拆分。

## 代码目录

- `gateway_h7/`：LIN Master、CAN 网关、UDS/OTA、NvM。
- `window_ecu_rc/`：车窗控制、防夹、堵转、ISO-TP、UDS。
- `bootloader/`：A/B Bootloader、镜像校验、Metadata、Flash 下载。
- `button_node_c8/`：按键采集、LIN Slave、Sleep/Wakeup。

仓库主要保留项目核心业务源码，STM32 HAL、CMSIS、FreeRTOS 等通用依赖未纳入。
