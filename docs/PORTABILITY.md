# IgH API Portability Notes

本文档用于判断哪些主站侧代码可以直接复制到客户工程，哪些代码必须依赖 Linux IgH 运行环境。

## 可直接复制的部分

以下文件不依赖 `ecrt.h`、pthread 或 Linux IgH master，可作为纯 C99 的 ECAN-Lite MCAN 动态 PDO 编解码模块复制：

| 文件 | 职责 |
| --- | --- |
| `include/igh_ecan_common.h` | 通用返回值、默认 CAN/CAN FD 参数、位宏 |
| `include/igh_mcan_pdo_codec.h` | 动态 PDO wire format、CAN/CAN FD frame 结构、writer/parser API |
| `src/igh_mcan_pdo_internal.h` | PDO codec 内部共享声明 |
| `src/igh_mcan_pdo_common.c` | 字节序、CRC、DLC、PDO header 和诊断文本 |
| `src/igh_mcan_pdo_writer.c` | 把 CAN/CAN FD frame 编入 RxPDO payload |
| `src/igh_mcan_pdo_parser.c` | 从 TxPDO payload 解析 CAN/CAN FD frame |

本工程也提供独立静态库目标：

```sh
cmake --build build --target ecan_igh_mcan_pdo_codec
```

该目标不链接 IgH EtherCAT 库。客户工程若已有自己的 EtherCAT、共享内存或驱动通道，只需要把 PDO buffer 指针交给 `igh_mcan_pdo_writer_*()` / `igh_mcan_pdo_parse_frames_ex()`。

## 需要 IgH 的部分

| 模块 | 依赖 | 原因 |
| --- | --- | --- |
| `igh_adapter` | `ecrt.h`、pthread | IgH master/domain、PDO 映射、DC Sync0、OP 状态 |
| `igh_mcan_config` | `igh_adapter` | 把 MCAN 配置转换为 IgH 启动 SDO 队列 |
| `igh_mcan_port` | `igh_adapter`、PDO codec | 绑定 IgH raw PDO，提供周期收发 API |

## API 使用边界

- 实时周期线程优先使用 `igh_mcan_port_cycle_try_begin()` 或 checked API，并检查返回值。
- `ecan_igh_wait_op()`、`ecan_igh_wait_op_timeout()`、`ecan_igh_reboot_slaves()` 是 blocking API，不应放入实时周期路径。
- OptionFlags bit0（`IGH_MCAN_OPT_RESERVED_BIT0`）当前为保留位；MCAN port 配置会返回
  `IGH_ECAN_UNSUPPORTED`。协议常量只用于识别并拒绝非当前版本的数据，主路径只生成和解析 `CAN_FRAME`。
- 如果客户只复制 PDO codec，需要自行提供 PDO buffer 的生命周期和同步策略；writer 在 `finish()` 时使用内存屏障发布 header。
