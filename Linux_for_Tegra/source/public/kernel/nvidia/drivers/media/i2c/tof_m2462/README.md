# `tof_m2462` — IRS2975C（M2462）双流 ToF 内核驱动

这是 `kernel/tof_m2453` 的 M2462 版本，功能对应用户态的
`tof_app_irs2975c`：在 **probe 期读模块 SPI flash 的小表 + 每个 use case 的
密寄存器块**，之后切 use case 只做一次 RAM → I2C 的突发写，不再访问 flash。
211928 B 的标定 blob 完全不碰，仍归用户态深度引擎。

## 型号与寄存器差异

`doc/imager_dev_codes.md` 的对应关系：

| 销售型号 | imager |
|---|---|
| IRS2380C / IRS2381C | M2453 |
| IRS2771C | M2455 |
| IRS2877C | M2457 |
| IRS2877A | M2442 |
| IRS2977C | M2458 |
| IRS2875C | M2459 |
| **IRS2975C（本驱动）** | **M2462** |

M2462 使用 **M2459/M2462 寄存器代**，和 M2453 不同：

| 项目 | M2453/M2455/M2457 | **M2459/M2462（IRS2875C/IRS2975C）** |
|---|---|---|
| SPI 控制寄存器 | `0xA087..0xA08C` | **`0xA095..0xA09A`** |
| stop 后轮询的 idle 状态 | `STATUS 0x9403` | **`STATUS0 0x940E`** |
| SPI 时钟极性位 | 不需要（`0x4002`） | **需要（`0x5002`）** |
| SPICFG 写入时机 | 复位后直接写 | **先等 `STATUS0` bit0 = 1 再写** |

`0x9403` 在 M2459/M2462 上是 `CFGCNT_SEQ_RECONFIG`，拿它当 idle 标志会永远
等不到。驱动里用 `tof_regs(tof)->status` 统一取表，M2462 表见
`tof_m2462_regs.c`。

## 双流 use case

IRS2975C 是 Flood + Spot 双流模组：

- 两路流**交替出现在相邻 UVC 帧**；
- 每个 UVC 帧只携带一路的 raw frame；
- `Hybrid_Mode18_15fps` 是整个深度帧 18 个 raw frame，分到每路是
  **9 个/UVC 帧**（1 个无照明强度帧 + 2 个频率 × 4 相位）；
- `HDR_Hybrid_Mode34_15fps` 同理是 **17 个/UVC 帧**；
- 每个 raw 子图是 **240 × 180，RAW12**。

`tof_m2462_zwe.c` 按 `tof_native/module_config.py` 的公式解析：

```text
phases     = Σ (phase_byte == 1 ? 1 : 4)
per_stream = phases / streamCount
raw_frames = streamCount == 1 && declared ? declared : per_stream
```

所以 dmesg 里 use case 行是 `raw=9/frame streams=2`，不是 18。`struct
tof_usecase` 同时保留 `streams` 字段，板级驱动/上层可以据此知道两路流要
交替处理。

## 文件清单

| 文件 | 内容 |
|---|---|
| `tof_m2462.h` | 寄存器定义、imager 表描述、use case 描述、驱动状态、API |
| `tof_m2462_regs.c` | **按 imager 型号的寄存器表**：M2462/IRS2975C、M2459/IRS2875C、M2453 家族、M2442/IRS2877A |
| `tof_m2462_i2c.c` | 寄存器读/写、burst 分组、`poll_until`、`reset_pulse` |
| `tof_m2462_spi.c` | SPI 主控 bring-up、256 B flash 读、**M2462 ready 等待** |
| `tof_m2462_zwe.c` | ToC / `Elena`(usecase) / `eLENA`(寄存器表) 解析、**双流 raw frame 拆分** |
| `tof_m2462_core.c` | probe 期读表 + 缓存 usecase 块、bring-up、start/stop、set_usecase、模块模板 |
| `Makefile` / `Kconfig` | 外部模块构建 |

## 编译

把整个目录放进内核树（例如 `drivers/media/i2c/tof_m2462/`），或作为外部模块：

```bash
make -C <linux> M=$PWD/tof_m2462 modules
```

Kconfig：

```text
CONFIG_TOF_M2462=m
```

生成 `tof_m2462.ko`。`Makefile` 会编译 core/i2c/regs/spi/zwe 五个对象。

**注意**：`tof_m2462_core.c` 末尾的 `i2c_driver` 模板默认仍包在 `#if 0` 里，
因为实际项目通常把这套 engine 集成到板级驱动（例如 `irs2877a`）中，避免
重复注册 i2c driver。要独立编译成模块，删除 `#if 0` / `#endif`，并按目标
内核调整 probe/remove 签名：

- Linux 6.6+：`int probe(struct i2c_client *)`、`void remove(struct i2c_client *)`
- Linux 6.5 及更早：`int probe(struct i2c_client *, const struct i2c_device_id *)`、
  `int remove(struct i2c_client *)`

## 集成点

1. **板级 reset 回调**

   驱动不假设 reset 线怎么接。probe 前通过 platform data 传入：

   ```c
   static const struct tof_m2462_board_ops board_ops = {
           .set_reset = my_board_set_reset,   /* gpiod_set_value() + 时序 */
   };
   ```

   M2462 的 SPI bring-up 会在 reset 之后先等 `STATUS0` ready；这个等待由
   `tof_m2462_spi_wait_ready()` 完成，板级代码不需要重复。

2. **选对寄存器表**

   ```c
   tof->regs = tof_m2462_regs_by_name("irs2975c");
   ```

   或 DT 属性：

   ```dts
   infineon,imager = "irs2975c";
   ```

   不给时回落到 M2462 表并打印警告。

3. **V4L2 / 控制面**

   `tof_m2462_core.c` 末尾的 TODO：把 `tof->use_cases[i]` 的
   `name/width/height/raw_frames/streams/fps_*` 做成 menu control 或私有
   ioctl，setter 调 `tof_m2462_set_usecase()`；`s_stream` 调
   `start_capture/stop_capture`。停止时才能切模式（驱动返回 `-EBUSY`）。

4. **标定 blob**

   驱动只报告它在 flash 里的位置/大小/CRC；211928 B 的读取仍由用户态
   通过 `request_firmware()` 或文件完成。

典型顺序：

```c
ret = tof_m2462_init_state(tof);      /* 分配 xfer_buf/word_buf，选表 */
ret = tof_m2462_probe_config(tof);    /* reset + SPI + 小表 + 缓存密块 */
ret = tof_m2462_bring_up(tof);        /* reset + design step + INIT/FW 表 */
ret = tof_m2462_set_usecase(tof, idx);/* M2462: 写缓存的密块 */
ret = tof_m2462_start_capture(tof);   /* START-MAP */
```

## 与 `kernel/tof_m2453` 的差异

| 差异 | 说明 |
|---|---|
| 新增 M2462 表 | SPI `0xA095..0xA09A`，STATUS0 `0x940E`，polarity + ready |
| SPI 初始化 | 复位后先 `tof_m2462_spi_wait_ready()`，再写 `SPICFG=0x5002` |
| use case 几何 | 解析 `streamCount`/raw frame sets，`raw_frames` 是每个 UVC 帧的数量（9），不是深度帧总数（18） |
| design step | M2462 A11 为 `0x0A11`，已在 `tof_valid_design_steps[]` 中 |
| 默认表 | `tof_regs()` 回落到 M2462，而不是 M2453 |
| 日志 | use case 行打印 `raw=9/frame streams=2`；I2C 读失败打印具体寄存器 |

## 排查

### `cannot read STATUS 0x940E` / `-ENODEV`

说明 imager 没有在 I2C 上应答。检查：

- i2c 总线号、7 位地址；
- 模组是否上电；
- reset 是否已经释放（`tof_m2462_reset_pulse()` 是否真的执行）；
- `dmesg` 里是否有 `no acknowledge from address`。

M2462 的 ready 等待读的是 `0x940E`，不是 M2453 的 `0x9403`。

### SPI 读回全 0

先确认：

- 选的是 M2462 表（`0xA095..`）；
- `SPICFG` 值带 polarity 位（`0x5002`）；
- 复位后确实等待了 `STATUS0` ready。

### use case 切换后配置没下发

按 `kernel/tof_m2453/README.md` 的排查顺序：确认 `active_usecase` 不是开关、
确认块缓存成功、必要时用 `tof_m2462_set_usecase_reg()` 逐寄存器兜底。

## 用户态工具

`tof_app_irs2975c/` 保留为离线/诊断工具：可以用 `.zwe` 文件验证解析器、CRC、
寄存器块缓存和双流 raw frame 计算，不需要 imager 或内核模块。
