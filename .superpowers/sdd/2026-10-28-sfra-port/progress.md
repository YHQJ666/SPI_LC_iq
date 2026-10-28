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
