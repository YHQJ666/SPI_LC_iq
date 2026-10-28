/*
 * 28035_RAM_lnk.cmd
 *
 * F28035 单相离网逆变 —— 从 SARAM 运行版本（调试用，下载最快）
 *
 * 注意：F2803x 的 L0/L1/L2/L3 在 PAGE0/PAGE1 上是同一块物理内存，
 *       不能同时出现在两个 PAGE 里，否则链接器会报 overlap。
 *       本脚本按 CPU 侧统一使用：代码 L1+L2+L3，数据 L0 和 M1。
 *
 * 内存分配：
 *   M0  0x000050~0x0003FF  944 word   ramfuncs / .pinit / .switch
 *   M1  0x000480~0x0007FF  896 word   栈
 *   L0  0x008000~0x0087FF  2048 word   .ebss / .econst（其中 0x8600 起划给示波器）
 *   L1  0x008800~0x008BFF  1024 word  \
 *   L2  0x008C00~0x008FFF  1024 word   > 代码 .text / .cinit / IQmath
 *   L3  0x009000~0x009FFF  4096 word  /
 *
 * 入口 BEGIN = 0x000000：boot-to-SARAM 引导，
 * 由调试器 Load Program 把程序装进 SARAM，不写 Flash。
 *
 * SFRA 扫频版必须改用 28035_FLASH_lnk.cmd（代码放 Flash，SARAM 全留给数据）。
 */

MEMORY
{
PAGE 0 :
   // 示波器缓冲固定区（不随代码变化）
   //   SCOPEBUF : 0x8600 起 512 字，放 ScopeIout / ScopeIL（各 256 字）
   SCOPEBUF   : origin = 0x008600, length = 0x000200
   BEGIN      : origin = 0x000000, length = 0x000002
   RAMM0      : origin = 0x000050, length = 0x0003B0

   // 代码区：整个 L1L2L3 6144 字
   RAML1L2L3  : origin = 0x008800, length = 0x001800


   RESET      : origin = 0x3FFFC0, length = 0x000002

   IQTABLES   : origin = 0x3FE000, length = 0x000B50
   IQTABLES2  : origin = 0x3FEB50, length = 0x00008C
   IQTABLES3  : origin = 0x3FEBDC, length = 0x0000AA

   BOOTROM    : origin = 0x3FF27C, length = 0x000D44

PAGE 1 :
   BOOT_RSVD  : origin = 0x000002, length = 0x00004E
   RAMM1      : origin = 0x000480, length = 0x000380

   // L0 数据区（0x8600~0x87FF 已经划给 SCOPEBUF，所以这里只到 0x85FF）
   RAML0      : origin = 0x008000, length = 0x000600
}

SECTIONS
{
   codestart        : > BEGIN,      PAGE = 0
   ramfuncs         : > RAMM0,      PAGE = 0
   .pinit           : > RAMM0,      PAGE = 0
   .switch          : > RAMM0,      PAGE = 0

   .text            : > RAML1L2L3,  PAGE = 0
   .cinit           : > RAML1L2L3,  PAGE = 0
   IQmath           : > RAML1L2L3,  PAGE = 0

   IQmathTables     : > IQTABLES,   PAGE = 0, TYPE = NOLOAD

   .stack           : > RAMM1,      PAGE = 1
   .esysmem         : > RAMM1,      PAGE = 1

   .ebss            : > RAML0,      PAGE = 1
   .econst          : > RAML0,      PAGE = 1
   // 示波器数据缓冲区：固定在 SCOPEBUF，代码长度变化不影响
   scopeBufs        : > SCOPEBUF,   PAGE = 0

   .reset           : > RESET,      PAGE = 0, TYPE = DSECT
}
