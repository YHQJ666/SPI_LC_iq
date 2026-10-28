# SDD ledger — plan: docs/superpowers/plans/2026-10-28-sfra-port.md

## Setup

- Branch: feature/sfra-port (from main), 未用 worktree。
- Ruling: 用分支代替 worktree —— CCS 工程是绝对路径导入的，worktree 副本要重新 import 才能编译，代价大于隔离收益；同一文件夹切分支即可隔离 main。代价：分支切换时 CCS 需要 Refresh。
- Ruling: 台账用 pwsh 手工维护，未用 sdd-workspace/task-start/task-done 脚本 —— 该脚本是 bash 脚本且需要 scripts 目录；手工维护格式一致。代价：少了脚本自动收集测试输出。
- Ruling: TDD 在此工程退化为「编译 + map/符号检查 + 上板现象」三步，因为固件没有 host 侧单测框架；计划里的每个 Expected 都按此执行。代价：没有真正的红灯-绿灯单元测试，回归依赖编译和上板。
- Pre-flight: 任务 1/3/6/7 共享 LOOP_MODE；任务 3/4/5 共享 .cproject 与链接脚本；任务 5/6 共享 SFRA_IQ_INJECT/COLLECT 签名。已逐条核对，无冲突。

## Progress
Task 1: complete (commits d1a7650..HEAD, tests: gmake -C Debug -k all -> exit 0; 区分大小写 grep 无 Loop_Mode 残留)
Task 2: complete (commits 52d2e32..HEAD, tests: gmake -C Debug -k all -> exit 0)
Task 2: Ruling: 顺带把 SCOPEBUF2(256字) 还给代码区 —— ScopeICap 删掉后该段无使用者，且代码区原来只剩 208 字；代价：以后再加第 3 个示波器通道要重新划区。代码空闲 208 -> 563 字。
Task 3: complete (commits 4516026..HEAD, tests: gmake -C Debug -k all -> exit 0 for SFRA_ENABLE=0 AND =1)
Task 3: Ruling: 不改 .cproject —— 手改 XML 新增两个 configuration 有把工程改坏的风险（CCS 打不开工程）。改用「改 dcac_cfg.h 的 SFRA_ENABLE + 在 CCS 里换一个 .cmd」两步切换；配错组合会被链接器溢出挡住。代价：切换要手动两步，忘了换 .cmd 会链接失败（安全失败）。
Task 3: Ruling: Oled.c 整个文件用 #if (SFRA_ENABLE==0) 包住（原计划靠 .cproject 的 file exclude）—— 不改 .cproject 就得靠宏，否则 SFRA 版仍白占 432 字代码。
Task 3: 实测 SFRA_ENABLE=1 时 OLED 代码被剔除后代码区空闲约 995 字（仍不够 1511 字的完整 SFRA，故必须 Flash）。
Task 4: complete (commits e2d0309..HEAD, tests: Flash+SFRA_ENABLE=1 链接 exit 0；ramfuncs LOAD 0x3F0000 -> RUN 0x8000)
Task 4: 实测 Flash 版 FLASHA 用 5080/8064、FLASHC 全空、dataRAM1 用 728/4096，余量足够放 SFRA(1511 字代码 + 512 字数据)。
Task 4: Ruling: SFRA 头文件用绝对路径 #include、SFRA 库用 .cmd 里的 -l 引入，都不改 .cproject —— 与 Task 3 的 Ruling 一致，代价是路径写死在源码/脚本里，换 controlSUITE 版本要改两处。
Task 5: complete (SFRA_IQ lib 730 字 + SciCommsGui 781 字全部链进 Flash 版，map 里可见 SFRA_IQ_BACKGROUND/INIT/COLLECT)
Task 5: Ruling: SFRA 头文件用绝对路径 #include（APP/sfra_cfg.h）、库用 28035_FLASH_lnk.cmd 里的 -l 引入 —— 不改 .cproject。
Task 5: 坑: Boost 的 SciCommsGui_32bit.c 是 UTF-8 不是 GBK；按 GBK 解会全乱码。重做后无 U+FFFD。
Task 6: complete (四个 LOOP_MODE 的注入/采集点 + 参考冻结 SFRA_VREF_DC=20V)
Task 7: complete (SFRA_ENABLE=1 强制 Protect_Enable=1；SFRA_INJ/SFRA_COL 故障门)
Task 4 fix: SFRA_IQ_Data 落在 0x93D8 不是 64 对齐 -> 单独划 SFRA_DATA(0x9C00) 后为 0x9C00，余 0。
Task 8: 两套构建均 exit 0: OLED(RAM, ENABLE=0) 代码 0x15CD/0x1800 余 563 字; SFRA(Flash, ENABLE=1) FLASHA 0x1E76/0x1F80 余 266 字 + FLASHC 8192 备用。
Task 8: 未完成的上板验证（需用户硬件）：OLED 8 行、示波器两通道、SFRA GUI 连接、扫 Plant。
Review: 独立 reviewer 给出 3 Critical + 11 Important. 已修 C1(mode2 双次 INJECT -> 电流单环旁路电压环)、C2(冻结工作点改走 RefRamp 斜坡)、C3(COLLECT 第一参数改 IRef)、I4(#error 互锁)、I5(InitFlash 无条件调用)、I7(ALIGN(64))、I9(三张空表填哨兵)、I3(mode1 真正实现电压单环)。
Review 遗留(未修): I2 mode2 轻载过压(已写进文档要求带载)、I6 .cproject 三配置(ruling:不改)、I8 20V 工作点实测达不到(PR 无直流增益)、I10 故障门是冗余(外层 if 已保证)、I11 故障后 GUI 无错误码。
坑: 28035_RAM_lnk.cmd 被 PowerShell 写入搞成乱码+重复 MEMORY 块，已用 write 工具重建。链接命令里一度同时传了 RAM 和 FLASH 两个 cmd，已修 Debug/makefile。
Task 8: 终验两套 exit 0. OLED(RAM) 0x15CF/0x1800 余 561; SFRA(Flash) FLASHA 0x1E48/0x1F80 余 312 + FLASHC 8K 备用; SFRA_IQ_Data=0x9C00 64 对齐。
