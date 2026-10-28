	SciaRegs.SCIFFTX.all = 0x8040;		/* 关闭FIFO增强(与TI原始工程一致)。
	   注意: 改成 0xE040 打开16级FIFO会导致连不上，已实测，别改回去。 */