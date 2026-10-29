/*
 * 28035_RAM_lnk.cmd
 *
 * F28035 单相离网逆变 —— 纯 RAM 运行版（不接 SFRA GUI 时用这个）
 *
 * 与 28035_FLASH_lnk.cmd 的区别：
 *   - 代码不再放 Flash，全部在 SARAM（L0~L2，0x8000~0x95FF），没有 Flash 等待周期
 *   - 不需要 memcpy 搬 ramfuncs（LOAD=RUN），所以 dcac_cfg.h 里
 *     SFRA_FLASH_BUILD 必须改成 0
 *   - 掉电/复位后程序丢失，必须用 CCS 重新 Load & Run
 *
 * 地址分配（F28035 10K 字 SARAM）：
 *   PAGE 0  CODERAM   0x8000..0x95FF (0x1600)  .text/.cinit/.econst/IQmath/ramfuncs
 *   PAGE 1  dataRAM1  0x9600..0x9BFF (0x0600)  .ebss/scopeBufs
 *   PAGE 1  SFRA_DATA 0x9C00..0x9FFF (0x0400)  SFRA 库内部段（要求 64 字对齐）
 *   PAGE 1  RAMM0     0x0050..0x03FF (0x0300)  栈
 *   PAGE 1  RAMM1     0x0480..0x07FF (0x0380)  堆(.esysmem)
 */

-l "C:/ti/controlSUITE/libs/app_libs/SFRA/v1_20_00_00/IQ/lib/SFRA_IQ_Lib.lib"

MEMORY
{
PAGE 0 :

   BEGIN      : origin = 0x000000, length = 0x000002   /* codestart */

   CODERAM    : origin = 0x008000, length = 0x001A00   /* 代码 0x8000~0x99FF */

   RESET      : origin = 0x3FFFC0, length = 0x000002
   VECTORS    : origin = 0x3FFFC2, length = 0x00003E

   IQTABLES   : origin = 0x3FE000, length = 0x000B50
   IQTABLES2  : origin = 0x3FEB50, length = 0x00008C
   IQTABLES3  : origin = 0x3FEBDC, length = 0x0000AA

   BOOTROM    : origin = 0x3FF27C, length = 0x000D44


PAGE 1 :

   BOOT_RSVD  : origin = 0x000002, length = 0x00004E

   RAMM0      : origin = 0x000050, length = 0x0003B0   /* 栈 */

   RAMM1      : origin = 0x000400, length = 0x000400   /* 堆 */

   dataRAM1   : origin = 0x009A00, length = 0x0005C0   /* 变量 0x9A00~0x9FBF */

   SFRA_DATA  : origin = 0x009FC0, length = 0x000040   /* 库要求 64 字对齐 (0x9FC0/64=639) */
}


SECTIONS
{
   /* ---- 程序（全部在 SARAM） ---- */
   codestart        : > BEGIN,           PAGE = 0

   .text            : > CODERAM,         PAGE = 0
   .cinit           : > CODERAM,         PAGE = 0
   .pinit           : > CODERAM,         PAGE = 0
   .switch          : > CODERAM,         PAGE = 0
   .econst          : > CODERAM,         PAGE = 0
   IQmath           : > CODERAM,         PAGE = 0

   /* RAM 版里 ramfuncs 就是普通 RAM 代码：LOAD=RUN，main() 不再搬移。
      ADC_ISR 的 #pragma CODE_SECTION(...,"ramfuncs") 照样放这里。 */
   ramfuncs         : > CODERAM,         PAGE = 0

   .reset           : > RESET,           PAGE = 0, TYPE = DSECT
   vectors          : > VECTORS,         PAGE = 0, TYPE = DSECT

   IQmathTables     : > IQTABLES,        PAGE = 0, TYPE = NOLOAD


   /* ---- 数据 ---- */
   .stack           : > RAMM0,           PAGE = 1
   .esysmem         : > RAMM1,           PAGE = 1

   .ebss            : > dataRAM1,        PAGE = 1
   scopeBufs        : > dataRAM1,        PAGE = 1

   SFRA_IQ_Data     : > SFRA_DATA, ALIGN(64), PAGE = 1
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
