/*
 * 28035_FLASH_lnk.cmd
 *
 * F28035 单相离网逆变 —— 从 Flash 运行版本
 *
 * 用途：SFRA 扫频版必须用这个脚本。
 *   RAM 版（28035_RAM_lnk.cmd）代码全放 SARAM，只有 10K 字；
 *   加了 SFRA 库（730 字）+ 上位机协议（781 字）后放不下，
 *   所以代码改放 Flash（64K），SARAM 全留给数据。
 *
 * 启动流程：
 *   boot ROM -> BEGIN(0x3F7FF6) -> codestart -> _c_int00
 *   -> main() 里先 memcpy 把 ramfuncs 从 Flash 搬到 RAM，再调 InitFlash()
 *   -> 之后才能从 Flash 取指跑（Flash 等待周期要在 InitFlash 里设好）
 *
 * 关键点：
 *   - ramfuncs 必须 LOAD 在 Flash、RUN 在 RAM，否则 InitFlash 自己在 Flash 里
 *     跑时还没设等待周期，会取到错指令
 *   - SFRA_IQ_Data 段是库要求的，必须放 RAM 且 64 字对齐；
 *     这里把它写成 dataRAM1 的第一条，靠 origin 0x9000 天然对齐
 */

/*
 * SFRA_IQ 库（v1.20）。
 * 放在链接脚本里而不是工程设置里，是为了不改 .cproject：
 * 只有引用到 SFRA 符号时链接器才会从这个库里取目标文件，
 * 所以同一套脚本配 OLED 模式也不会白占代码空间。
 */
-l "C:/ti/controlSUITE/libs/app_libs/SFRA/v1_20_00_00/IQ/lib/SFRA_IQ_Lib.lib"

MEMORY
{
PAGE 0 :
   BEGIN      : origin = 0x3F7FF6, length = 0x000002   /* boot-to-Flash 入口 */

   /* 代码区：FLASHA + FLASHC 连续约 16K 字，够放应用 + SFRA 库 + 上位机协议 */
   FLASHA     : origin = 0x3F6000, length = 0x001F80
   FLASHC     : origin = 0x3F2000, length = 0x002000

   /* ramfuncs 的加载区 */
   FLASHD     : origin = 0x3F0000, length = 0x002000

   CSM_RSVD   : origin = 0x3F7F80, length = 0x000076   /* 用 CSM 时填 0 */
   CSM_PWL    : origin = 0x3F7FF8, length = 0x000008

   /* ramfuncs 的运行区（SARAM L0+L1，4K 字） */
   RAML0L1    : origin = 0x008000, length = 0x001000

   IQTABLES   : origin = 0x3FE000, length = 0x000B50
   IQTABLES2  : origin = 0x3FEB50, length = 0x00008C
   IQTABLES3  : origin = 0x3FEBDC, length = 0x0000AA
   BOOTROM    : origin = 0x3FF27C, length = 0x000D44
   RESET      : origin = 0x3FFFC0, length = 0x000002
   VECTORS    : origin = 0x3FFFC2, length = 0x00003E

PAGE 1 :
   BOOT_RSVD  : origin = 0x000000, length = 0x000050
   RAMM0      : origin = 0x000050, length = 0x0003B0   /* 栈 */
   RAMM1      : origin = 0x000400, length = 0x000400

   /* 数据区：L3，4K 字，起始 0x9000 是 64 字对齐的 */
   dataRAM1   : origin = 0x009000, length = 0x000C00

   /* SFRA 库内部数据段单独一块：库要求 64 字对齐，
      放独立区才能保证起点是 64 的整数倍（0x9C00 / 64 = 624）。
      混在 dataRAM1 里跟着别的段走的话，实测会落到 0x93D8 这种地址上。 */
   SFRA_DATA  : origin = 0x009C00, length = 0x000400
}

SECTIONS
{
   /* ---- 程序 ---- */
   .cinit           : > FLASHA | FLASHC,  PAGE = 0
   .pinit           : > FLASHA | FLASHC,  PAGE = 0
   .text            : > FLASHA | FLASHC,  PAGE = 0
   .econst          : > FLASHA | FLASHC,  PAGE = 0
   .switch          : > FLASHA | FLASHC,  PAGE = 0
   IQmath           : > FLASHA | FLASHC,  PAGE = 0

   codestart        : > BEGIN,           PAGE = 0

   ramfuncs         : LOAD = FLASHD,
                      RUN  = RAML0L1,
                      LOAD_START(_RamfuncsLoadStart),
                      LOAD_END(_RamfuncsLoadEnd),
                      RUN_START(_RamfuncsRunStart),
                      PAGE = 0

   csmpasswds       : > CSM_PWL,         PAGE = 0
   csm_rsvd         : > CSM_RSVD,        PAGE = 0

   IQmathTables     : > IQTABLES,        PAGE = 0, TYPE = NOLOAD
   .reset           : > RESET,           PAGE = 0, TYPE = DSECT
   vectors          : > VECTORS,         PAGE = 0, TYPE = DSECT

   /* ---- 数据 ---- */
   .stack           : > RAMM0,           PAGE = 1
   .esysmem         : > RAMM1,           PAGE = 1

   /* SFRA 库要求的段：必须 64 字对齐，单独放 SFRA_DATA（0x9C00 起） */
   SFRA_IQ_Data     : > SFRA_DATA,       PAGE = 1

   /* 示波器缓冲区：地址随本脚本走，Flash 版下和 RAM 版不同，用数组名访问 */
   scopeBufs        : > dataRAM1,        PAGE = 1

   .ebss            : > dataRAM1,        PAGE = 1
}
