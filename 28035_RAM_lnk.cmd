/*
 * 28035_RAM_lnk.cmd
 *
 * F28035 单相离网逆变（全部在 CPU 上运行）RAM 链接命令文件
 *
 * CLA 已经不再使用，所以 L1/L2/L3 全部归还给 CPU，代码放在 L1+L2+L3，
 * 数据放在 L0 和 M1，M0 放 ramfuncs 和初始化表。整片 SARAM 都可以用。
 *
 * 存储器分配：
 *   M0  0x000050~0x0003FF  944 word   ramfuncs / .pinit / .switch
 *   M1  0x000480~0x0007FF  896 word   栈
 *   L0  0x008000~0x0087FF  2048 word  数据（.ebss / .econst）
 *   L1  0x008800~0x008BFF  1024 word  \
 *   L2  0x008C00~0x008FFF  1024 word   > 连起来放代码（.text / .cinit / IQmath）
 *   L3  0x009000~0x009FFF  4096 word  /
 *
 * 起始地址 BEGIN = 0x000000，对应 boot-to-SARAM 模式，
 * 用调试器直接 Load Program 跑，不需要烧 Flash
 */

MEMORY
{
PAGE 0 :
   BEGIN      : origin = 0x000000, length = 0x000002
   RAMM0      : origin = 0x000050, length = 0x0003B0

   // L1+L2+L3 连续 6K word，CLA 不用了全部给代码
   RAML1L2L3  : origin = 0x008800, length = 0x001800

   RESET      : origin = 0x3FFFC0, length = 0x000002

   IQTABLES   : origin = 0x3FE000, length = 0x000B50
   IQTABLES2  : origin = 0x3FEB50, length = 0x00008C
   IQTABLES3  : origin = 0x3FEBDC, length = 0x0000AA

   BOOTROM    : origin = 0x3FF27C, length = 0x000D44

PAGE 1 :
   BOOT_RSVD  : origin = 0x000002, length = 0x00004E
   RAMM1      : origin = 0x000480, length = 0x000380

   // L0 专门给数据
   RAML0      : origin = 0x008000, length = 0x000800
}

SECTIONS
{
   // 程序
   codestart        : > BEGIN,      PAGE = 0
   ramfuncs         : > RAMM0,      PAGE = 0
   .pinit           : > RAMM0,      PAGE = 0
   .switch          : > RAMM0,      PAGE = 0

   .text            : > RAML1L2L3,  PAGE = 0
   .cinit           : > RAML1L2L3,  PAGE = 0
   IQmath           : > RAML1L2L3,  PAGE = 0

   IQmathTables     : > IQTABLES,   PAGE = 0, TYPE = NOLOAD

   // 数据
   .stack           : > RAMM1,      PAGE = 1
   .esysmem         : > RAMM1,      PAGE = 1

   .ebss            : > RAML0,      PAGE = 1
   .econst          : > RAML0,      PAGE = 1
   // 示波器数据缓冲区：单独一段，避免挤占 .ebss（见 main.c 的 scopeBufs）
   scopeBufs        : > RAML1L2L3,  PAGE = 0
   // 电感电流数组单独放 L0（L1L2L3 剩余空间不够第 4 个数组）
   scopeBufs2       : > RAML0,      PAGE = 1

   .reset           : > RESET,      PAGE = 0, TYPE = DSECT
}
