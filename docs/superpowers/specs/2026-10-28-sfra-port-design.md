# SPI_LC_iq 接入 SFRA 扫频 + 电流内环改电感电流 —— 设计

日期：2026-10-28
状态：已确认（用户「按你的来」）

## 1. 意图

1. **电流内环换成电感电流**。现在反馈是 `ICap = IL - Iout`，两个 ADC 通道相减，噪声叠加
   （实测空载 RMS 0.29 A）。改成单路 `IL`。
2. **接入 TI SFRA 扫频**，实测环路：受控体两条（电流环、电压环）+ 闭环增益两条。

## 2. 已定的决策

| 决策 | 内容 |
|---|---|
| 构建方式 | 编译开关 `SFRA_ENABLE`：`0` = OLED 版（现状）、`1` = SFRA + SCI 上位机 |
| 方案 | 完整照搬 `Boost_SFRA`：`SciCommsGui_32bit.c` + SFRA GUI 上位机 |
| 程序位置 | SFRA 版**从 Flash 运行**（RAM 代码空间不够，实测缺约 870 字） |
| 扫频工作点 | **参考冻结成直流**：电压环 `Vref = 20 V`、电流环 `Duty = 0.4` |
| 保护 | **只用软件过压/过流**（`Protect_Enable=1`），不依赖硬件 COMP1/COMP2 |
| Flash 版启动 | **只靠调试器 Load**，不要求脱机自启动（boot 引脚不动） |
| 调试配置 | **新增第三个 build configuration**：RAM 运行 + SFRA、无示波器，用于快速迭代 |

## 3. 目标 / 非目标 / 反向目标

**目标**

- `SFRA_ENABLE=0` 时行为与现在**完全一致**（OLED 8 行、三通道示波器、RMS 显示）
- `SFRA_ENABLE=1` 时能扫出四张波特图并存 CSV
- 同一个 `main.c` 服务所有配置，switch 只在宏和 build configuration 层面

**非目标**

- 不追求 SFRA 与 OLED 同时在线（GPIO28/29 冲突）
- 不引入 3P3Z，不换 PR 结构
- 不动功率级和调理电路

**反向目标（做到就算失败）**

- OLED 版被 Flash 改造搞坏
- 扫频时功率级过压/过流
- 改 IL 反馈后闭环振荡

## 4. 代码增量（实测，来自 Boost_SFRA map）

```
SFRA_IQ_BACKGROUND 489 + INIT 142 + COLLECT 69 + INJECT 30 = 730 字
SciCommsGui_32bit.c                                        = 781 字
合计                                                        = 1511 字

现有空闲 208 字；关掉 OLED 后 640 字  ->  必须搬 Flash
```

## 5. 方案

### 5.1 三个 build configuration

| 配置 | 程序位置 | `SFRA_ENABLE` | 示波器 | OLED | 用途 |
|---|---|---|---|---|---|
| `Debug`（现状，保留） | RAM | 0 | 3 通道 | 有 | 日常调试 |
| `Debug_SFRA_RAM`（新增） | RAM | 1 | **无** | 无 | 扫频调参，Load 快 |
| `Release_SFRA_FLASH`（新增） | Flash | 1 | 有 | 无 | 最终验收 |

- 切换 = 选配置 + Rebuild All
- **第三、第四个配置都从 `Debug` 复制**（现在 `Release` 是空模板，缺 include 路径和库，靠它改会编不过）

### 5.2 构建开关

`dcac_cfg.h`：

```c
#ifndef SFRA_ENABLE
#define SFRA_ENABLE   0      /* 0 = OLED 版；1 = SFRA 版 */
#endif
```

必须写成 `#ifndef` 包住，否则在 build configuration 里加 `-dSFRA_ENABLE=1` 只会得到
"redefinition" 告警、头文件里的 0 赢，宏不生效。

`main.c` 里的 OLED 调用（`OLED_Init/OLED_Clear/Oled_Task/Oled_F`）**必须自己用 `#if` 包住**
—— 文件级 exclude 管不到 `main.c`。

`APP/Oled.c` 用配置的 exclude 排除；`APP/SciCommsGui_32bit.c` 反过来。

### 5.3 Flash 运行改造

新增 `28035_FLASH_lnk.cmd`，**以 SFRA 例程的脚本为模板**：

```
C:\ti\controlSUITE\libs\app_libs\SFRA\v1_20_00_00\examples\
    DPSWrkShpKit_SFRA_F28035\F28035_FLASH_DPS Workshop.CMD
```

同款目标芯片、同款 SFRA 库，比按 `F28035.cmd` 手改省事。要点：

- 不要自己重画 bank：`F28035.cmd` 已把 A–H 八个 sector 全部切开
- `ramfuncs : LOAD = FLASHD, RUN = RAMM0`，带 `LOAD_START/LOAD_END/RUN_START`
- **`ramfuncs` 只写一次**（`Boost_SFRA` 的 cmd 里 `.TI.ramfunc` 和裸 `ramfuncs` 各写了一遍，
  会占两份，别照抄）
- `codestart → BEGIN(0x3F7FF6)`
- `SFRA_IQ_Data : > RAM3, PAGE = 1`，`RAM3 : origin = 0x009000, length = 0x001000`
  并把 `SFRA_IQ_Data` 写成该区**第一条**（TI 例程就是这么做的，不靠 `ALIGN`）。
  链接后到 map 里确认实际地址能被 64 整除。

`main()` 在 `InitSysCtrl()` 之后插入（**必须包在 Flash 配置的 `#if` 里**，
RAM 版没有这些符号，共用 `main.c` 会链接失败）：

```c
memcpy(&RamfuncsRunStart, &RamfuncsLoadStart,
       (size_t)&RamfuncsLoadEnd - (size_t)&RamfuncsLoadStart);
InitFlash();
```

### 5.4 SFRA 集成

依赖（写进配置）：

```
头文件  C:\ti\controlSUITE\libs\app_libs\SFRA\v1_20_00_00\IQ\include
库      C:\ti\controlSUITE\libs\app_libs\SFRA\v1_20_00_00\IQ\lib\SFRA_IQ_Lib.lib
上位机  C:\ti\controlSUITE\libs\app_libs\SFRA\v1_20_00_00\GUI\SFRA_GUI.exe
```

参数：

```c
SFRA_FREQ_LENGTH   48          /* 见下：不是 RAM 限制，是 Nyquist 限制 */
SFRA_FREQ_START    100.0f
SFRA_FREQ_STEP     1.059253f   /* 每十倍频程 40 点 */
SFRA_ISR_FREQ      20000.0f    /* = 开关频率；写 40k 会让频率轴整体偏一倍 */
```

**扫频上限不能用 100 点**：从 100 Hz 起、每点 ×1.0593，100 点会扫到 31.6 kHz，
远超 20 kHz 采样的 Nyquist。实用上限取 `fsw/10 = 2 kHz`，对应 **48 点**
（100 Hz ~ 2.1 kHz）。这里数据数组 48×5×2 = 480 字，RAM 也不再紧张。

SCI：`SCIA_Init(15000000, 57600)`，分频 32。`InitSysCtrl` 里已有 `LOSPCP=2`（LSPCLK 15 MHz）
并已开 SCIAENCLK，不用改。

引脚：

```c
GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   /* SCIRXDA */
GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   /* SCITXDA */
GpioCtrlRegs.GPAPUD.bit.GPIO28  = 0;   /* RX 上拉，悬空不乱收噪声 */
GpioCtrlRegs.GPAPUD.bit.GPIO29  = 1;   /* TX 关上拉 */
GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  /* 异步输入 */
```

### 5.5 环路模式与注入/采集点

删掉原来的 `Loop_Mode`（开环/闭环），统一成 `LOOP_MODE`。编号照 `Boost_SFRA`，
**0 和 4 必须分开**——否则"OLED 版行为完全一致"和 SFRA 的"固定占空比"会打架：

| LOOP_MODE | 物理状态 | 注入 | 采集 | 用途 |
|---|---|---|---|---|
| 0 | 正弦开环 + 软启动 | — | — | **现状行为，SFRA_ENABLE=0 时用** |
| 1 | 电压单环 | `Vref` | `Vout` | 测电压环闭环增益 |
| 2 | 电流单环 | `Iref` | `IL` | 测电流环闭环增益 |
| 3 | 双环 | `Vref` | `Vout` | 测电压环闭环增益（双环下） |
| 4 | **固定占空比开环** | `Duty` | `IL` 或 `Vout` | 测受控体（Plant） |

**阶段 2（测电压环受控体）不新增模式**：在 mode 4 上用一个采集目标开关
（照 `Boost_SFRA` 的 `OPEN_LOOP_MEAS_CURRENT` 写法）把 collect 切到 `Vout`，
注入仍是 `Duty`。

### 5.6 扫频时的工作点（参考冻结）

单相逆变器的参考是 50 Hz 正弦，"注入 Vref 采 Vout"没有稳态工作点，必须冻结：

- **电压环扫频**（mode 1/3）：`Vref = _IQ(20.0f)` 直流，`RefRamp` 直接置到位，跳过正弦表
- **电流环扫频**（mode 2/4）：`Mod = _IQ(0.4f)` 固定占空比，电压环不参与

冻结只在 `SFRA_ENABLE=1` 时生效；`=0` 时仍走原来的正弦 + 软启动。

### 5.7 电流内环改 IL

```c
PI_I.Fdb = IL;        /* 原：ICap */
```

连带删掉只为 `ICap` 服务的东西：`RmsICapSq`、`FiltICap`、`Debug_View.ICapRms`、
`ScopeICap`、以及 `ICap = IL - Iout` 这行。OLED 第 6 行显示 IL 的 RMS，不变。

### 5.8 保护

`SFRA_ENABLE=1` 时强制 `Protect_Enable=1`（用 `#if` 互锁 + `#warning`），
只用软件过压（45 V）/过流（25 A）。硬件 COMP Trip 不依赖。

注入前加故障门：

```c
if (FaultLatch == FAULT_NONE) { Duty = SFRA_IQ_INJECT(Duty_DC); }
```

保护一动作就停止注入，否则扫出来的数据全废。

## 6. 怎么验证

| 步骤 | 通过标准 |
|---|---|
| 1. `Debug`（现状）编译 | 与现在完全一致，OLED 正常、示波器三通道有数据 |
| 2. `Debug_SFRA_RAM` 编译 | 链接不溢出，map 能看到 `SFRA_IQ_Data` 段且地址 64 字对齐 |
| 3. `Release_SFRA_FLASH` 编译 + 烧写 | 板子能跑（靠调试器 Load），串口有响应 |
| 4. SFRA GUI 连接 | 能读出 `Vec_Length`、`status`、`FreqIndex` |
| 5. 扫电流环 Plant | 低频增益与 L=1.12 mH 理论值大致吻合，-3dB 点位置合理 |
| 6. 改 IL 反馈后闭环带载 | 不振荡，输出电压稳定 |

## 7. 风险

| 风险 | 影响 | 对策 |
|---|---|---|
| Flash 改动后板子起不来 | 高 | 保留 `Debug` RAM 配置；调试器可重擦 Flash |
| `LOOP_MODE` 重构影响现状 | 高 | `LOOP_MODE=0` 分支保持原样，验收第 1 条卡住 |
| IL 纹波污染扫频 | 中 | 硬件 RC；不在 ISR 里软件滤波（SFRA.md 6.3） |
| 参考冻结后输出超压 | 中 | 软件 OV 保护 + 注入幅值从 0.5% 起 |
