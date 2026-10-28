/*
 * 28035_RAM_lnk.cmd
 *
 * F28035 闁告娲滃ù澶岀矉閼姐倗绉归梺顐㈡瑜板鏁嶉崼婵嗗伎闂侇喓鍔屽﹢?CPU 濞戞挸锕ㄧ换宥囨偘瀹€瀣RAM 闂佸墽鍋撶敮鎾川閹存帗濮㈤柡鍌氭矗濞?
 *
 * CLA 鐎规瓕灏欑划鈩冪▔瀹ュ懎鏅欏ù锝堟硶閺併倝鏁嶇仦鎯ь暡濞?L1/L2/L3 闁稿繈鍔戦崕纾嬨亹閹烘洜绠风紓?CPU闁挎稑濂旈崬顒勬儘娴ｈ鏉归柛?L1+L2+L3闁?
 * 闁轰胶澧楀畵渚€寮ㄩ幆褎韬?L0 闁?M1闁挎稑顒? 闁衡偓?ramfuncs 闁告粌鑻崹鍨叏鐎ｎ亜顕ч悶娑栧妸閳ь剙鍊归弳锝夋偋?SARAM 闂侇喛妫勮ぐ鍙夌閵壯勬殢闁?
 *
 * 閻庢稒锚閸嬪秹宕抽妸銉ョ€婚梺鏉跨▌缁?
 *   M0  0x000050~0x0003FF  944 word   ramfuncs / .pinit / .switch
 *   M1  0x000480~0x0007FF  896 word   闁?
 *   L0  0x008000~0x0087FF  2048 word  闁轰胶澧楀畵渚€鏁?ebss / .econst闁?
 *   L1  0x008800~0x008BFF  1024 word  \
 *   L2  0x008C00~0x008FFF  1024 word   > 閺夆晝鍋犻幑锝夊级閵夛附鏉瑰ù鐙呯悼閻栨粓鏁?text / .cinit / IQmath闁?
 *   L3  0x009000~0x009FFF  4096 word  /
 *
 * 閻犙冨槻椤劙宕烽弶鎸庣祷 BEGIN = 0x000000闁挎稑鑻顔芥償?boot-to-SARAM 婵☆垪鈧磭纭€闁?
 * 闁活潿鍔忛惃鐔烘嫚閺囩偞鐝ら柣鈺佺摠鐢?Load Program 閻犵儤鍩婄槐婵囩▔瀹ュ浠橀悷鏇氳兌閸?Flash
 */

MEMORY
{
PAGE 0 :
   // 示波器缓冲固定区（不随代码变化）
   //   SCOPEBUF  : 0x8600 起 512 字，放 ScopeIout / ScopeICap（各 256 字）
   //   SCOPEBUF2 : L1L2L3 尾部 256 字，放 ScopeIL
   SCOPEBUF   : origin = 0x008600, length = 0x000200
   BEGIN      : origin = 0x000000, length = 0x000002
   RAMM0      : origin = 0x000050, length = 0x0003B0

   // 代码区：5812 字，L1L2L3 剩下的 256 字给 SCOPEBUF2
   RAML1L2L3  : origin = 0x008800, length = 0x001700
   SCOPEBUF2  : origin = 0x009F00, length = 0x000100


   RESET      : origin = 0x3FFFC0, length = 0x000002

   IQTABLES   : origin = 0x3FE000, length = 0x000B50
   IQTABLES2  : origin = 0x3FEB50, length = 0x00008C
   IQTABLES3  : origin = 0x3FEBDC, length = 0x0000AA

   BOOTROM    : origin = 0x3FF27C, length = 0x000D44

PAGE 1 :
   BOOT_RSVD  : origin = 0x000002, length = 0x00004E
   RAMM1      : origin = 0x000480, length = 0x000380

   // L0 濞戞挻鎹囧Λ顒傜磼濞嗘劖娈堕柟?
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
   // 示波器数据缓冲区：固定在 SCOPEBUF / SCOPEBUF2，代码长度变化不影响
   scopeBufs        : > SCOPEBUF,   PAGE = 0
   scopeBufs2       : > SCOPEBUF2,  PAGE = 0

   .reset           : > RESET,      PAGE = 0, TYPE = DSECT
}
