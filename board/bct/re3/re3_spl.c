/*
 * Copyright (C) 2014 Gateworks Corporation
 * Author: Tim Harvey <tharvey@gateworks.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include <i2c.h>
#include <asm/io.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-ddr.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/clock.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/gpio.h>
#include <asm/spl.h>
#include <spl.h>
#include <asm/u-boot.h>
#include <mmc.h>
#include <image.h>
#include "mmdc.h"

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_SPL_NAND_SUPPORT

#define GPMI_PAD_CTRL0 (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP)
#define GPMI_PAD_CTRL1 (PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_MED | \
                        PAD_CTL_SRE_FAST)
#define GPMI_PAD_CTRL2 (GPMI_PAD_CTRL0 | GPMI_PAD_CTRL1)

static iomux_v3_cfg_t gpmi_pads[] = {
        MX6_PAD_NANDF_CLE__NAND_CLE             | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
        MX6_PAD_NANDF_ALE__NAND_ALE             | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
        MX6_PAD_NANDF_WP_B__NAND_WP_B   | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
        MX6_PAD_NANDF_RB0__NAND_READY_B | MUX_PAD_CTRL(GPMI_PAD_CTRL0),
        MX6_PAD_NANDF_CS0__NAND_CE0_B   | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
        MX6_PAD_SD4_CMD__NAND_RE_B              | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
        MX6_PAD_SD4_CLK__NAND_WE_B              | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
        MX6_PAD_NANDF_D0__NAND_DATA00   | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
        MX6_PAD_NANDF_D1__NAND_DATA01   | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
        MX6_PAD_NANDF_D2__NAND_DATA02   | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
        MX6_PAD_NANDF_D3__NAND_DATA03   | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
        MX6_PAD_NANDF_D4__NAND_DATA04   | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
        MX6_PAD_NANDF_D5__NAND_DATA05   | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
        MX6_PAD_NANDF_D6__NAND_DATA06   | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
        MX6_PAD_NANDF_D7__NAND_DATA07   | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
        MX6_PAD_SD4_DAT0__NAND_DQS              | MUX_PAD_CTRL(GPMI_PAD_CTRL1),
};

static void setup_gpmi_nand(void)
{
        struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

        /* config gpmi nand iomux */
        imx_iomux_v3_setup_multiple_pads(gpmi_pads, ARRAY_SIZE(gpmi_pads));

        /* gate ENFC_CLK_ROOT clock first,before clk source switch */
        clrbits_le32(&mxc_ccm->CCGR2, MXC_CCM_CCGR2_IOMUX_IPT_CLK_IO_MASK);

        /* config gpmi and bch clock to 100 MHz */
        clrsetbits_le32(&mxc_ccm->cs2cdr,
                        MXC_CCM_CS2CDR_ENFC_CLK_PODF_MASK |
                        MXC_CCM_CS2CDR_ENFC_CLK_PRED_MASK |
                        MXC_CCM_CS2CDR_ENFC_CLK_SEL_MASK,
                        MXC_CCM_CS2CDR_ENFC_CLK_PODF(0) |
                        MXC_CCM_CS2CDR_ENFC_CLK_PRED(3) |
                        MXC_CCM_CS2CDR_ENFC_CLK_SEL(3));

        /* enable ENFC_CLK_ROOT clock */
        setbits_le32(&mxc_ccm->CCGR2, MXC_CCM_CCGR2_IOMUX_IPT_CLK_IO_MASK);

        /* enable gpmi and bch clock gating */
        setbits_le32(&mxc_ccm->CCGR4,
                     MXC_CCM_CCGR4_RAWNAND_U_BCH_INPUT_APB_MASK |
                     MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_BCH_MASK |
                     MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_GPMI_IO_MASK |
                     MXC_CCM_CCGR4_RAWNAND_U_GPMI_INPUT_APB_MASK |
                     MXC_CCM_CCGR4_PL301_MX6QPER1_BCH_OFFSET);

        /* enable apbh clock gating */
        setbits_le32(&mxc_ccm->CCGR0, MXC_CCM_CCGR0_APBHDMA_MASK);
}
#endif

/* Run a pattern over the entire memory range, fill it up first
 * then checking the actual content in a second pass.
 * Location N should hold the value "PATTERN+N".
 * This is done three times, accessing memory by 32/16/8 bits.
 * Can either stop at the first occurrence of an error, or keep
 * going [which is useful to find *which* lines are actually shorted]
 */

#define MEMTESTPASS		0
#define MEMTESTFAIL		1

u32 MemoryTest1(u32 Start, u32 nBytes, u32 Pattern, u32 StopAtFirstError)
{

	u32 *pW32, A32;
    u8 *pW8, A8;
    u32 i;
	u32 starttime = 0;
	u32 currenttime = 0;

	//Test by words, write then readback

#define pattern(_i_)  (u32)((Pattern + (_i_)))

    printf("Testing in 32 bits at [x%x..x%x] with base pattern %x\r\n", Start,Start+nBytes,Pattern);
    
	
	pW32 = (u32 *) Start;
    for (i = 0; i < nBytes/4; i++) 
	{
        pW32[i] = pattern(i);
    }

    for (i = 0; i < nBytes/4; i++) 
	{
        A32 = pW32[i];
        if (A32 != pattern(i)) {
            printf("DWORD[%d] mismatch, got 0x%x (!= 0x%x)\r\n",i, A32, pattern(i));
            if (StopAtFirstError)
			{
                return MEMTESTFAIL;
			}
        }
    }
#undef pattern

    return MEMTESTPASS;
}


/*
 * called from C runtime startup code (arch/arm/lib/crt0.S:_main)
 * - we have a stack and a place to store GD, both in SRAM
 * - no variable global data is available
 */
void board_init_f(ulong dummy)
{
	/*
	 * Zero out global data:
	 *  - this shoudl be done by crt0.S
	 *  - failure to zero it will cause i2c_setup to fail
	 */
	memset((void *)gd, 0, sizeof(struct global_data));

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	mmdc_do_write_level_calibration();
	mmdc_do_dqs_calibration();

	MemoryTest1(0x10000000, 0x00100000, 0x00000000, 0);

	printf("Memory Test Complete\r\n");

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);


	get_clocks();

#ifdef CONFIG_SPL_NAND_SUPPORT
        setup_gpmi_nand();
#endif

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}

void reset_cpu(ulong addr)
{

}

int mmc_load_image(void)
{
	struct mmc *mmc;
	int err;

	mmc_initialize(gd->bd);
	mmc = find_mmc_device(0);
	if (!mmc)
		return -1;

	err = mmc_init(mmc);
	if (err) 
                return -1;

	return spl_load_image_fat(&mmc->block_dev,
				CONFIG_SYS_MMCSD_FS_BOOT_PARTITION,
				CONFIG_SPL_FS_LOAD_PAYLOAD_NAME);
}

void spl_board_load_image(void)
{
        /* attempt to load image from SD card first */
        /* otherwise attempt to load image from SPI NOR */
	if (mmc_load_image() == 0)
		printf("spl: booting from uSD card\n");
	else {
        	spl_spi_load_image();
		printf("spl: booting from SPI NOR\n");
        }

}

u32 spl_boot_device(void)
{
	return BOOT_DEVICE_BOARD;
}
