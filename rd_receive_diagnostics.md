# RD 接收诊断

船舶固件的北斗 USART1 使用 1024 字节循环 DMA，并由控制任务每 20 ms 主动读取 DMA 写指针，避免日志等到缓冲事件后成批出现。蓝牙原始命令区支持以下诊断命令：

RD 模式会配置模块每秒输出一次 `BDPWI` 状态，方便直接观察串口接收是否仍在运行。该状态只能证明模块 UART 有输出，不能替代 `$BDTXR/$BDTCI` 下行报文证据。

固件在转发每条 RD 帧时会原子追加 CRLF，确保手机蓝牙端的 `readLine()` 立即交付当前帧，不需要等待下一条消息。

| 命令 | 说明 |
|---|---|
| `rd diag` | 查询北斗 USART1 DMA 接收统计 |
| `rd diag reset` | 清零接收统计，不停止 DMA |

返回示例：

```text
RD diag bytes=328 events=12 frames=8 errors=0 restarts=1 restart_fail=0 overflow=0 drops=0 last_error=0x00000000 pos=328
```

- `bytes`：DMA 已交给帧解析器的字节数。
- `events`：DMA 半满、全满或 UART IDLE 接收事件次数。
- `frames`：成功进入模块消息队列的完整 `$...\r\n` 帧数。
- `errors`：USART1 的 ORE/FE/NE/PE 等错误次数。
- `restarts` / `restart_fail`：DMA 接收启动成功/失败次数。
- `overflow`：超过 512 字节后被整帧丢弃的数量。
- `drops`：模块消息队列已满造成的丢帧数量。
- `last_error`：最近一次 HAL UART 错误位。
- `pos`：当前 DMA 环形缓冲读取位置。

## 断点判定

1. 岸基下发后模块 UART 有 `$BDTXR`/`$BDTCI`，但 `bytes` 不增长：检查模块 TX 到 MCU PA10 的硬件连接。
2. `bytes` 增长而 `frames` 不增长：检查帧是否以 `$` 开头、是否带 CRLF，或查看 `overflow`。
3. `frames` 增长但 App 无原始帧：检查 `drops` 和蓝牙链路。
4. `errors` 增长：记录 `last_error`，确认 DMA 已自动 Abort 并重启，且 `restart_fail=0`。
