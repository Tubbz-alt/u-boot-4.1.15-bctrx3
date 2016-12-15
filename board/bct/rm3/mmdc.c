#include <common.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>

#define MMDC_P0_IPS_BASE_ADDR 0x021b0000
#define MMDC_P1_IPS_BASE_ADDR 0x021b4000

#define MDCTL_OFFSET		0x000
#define MDPDC_OFFSET		0x004
#define MDSCR_OFFSET		0x01c
#define MDMISC_OFFSET		0x018
#define MDREF_OFFSET		0x020
#define MAPSR_OFFSET		0x404
#define MPZQHWCTRL_OFFSET	0x800
#define MPWLGCR_OFFSET		0x808
#define MPWLDECTRL0_OFFSET	0x80c
#define MPWLDECTRL1_OFFSET	0x810
#define MPPDCMPR1_OFFSET	0x88c
#define MPSWDAR_OFFSET		0x894
#define MPRDDLCTL_OFFSET	0x848
#define MPMUR_OFFSET		0x8b8
#define MPDGCTRL0_OFFSET	0x83c
#define MPDGHWST0_OFFSET	0x87c
#define MPDGHWST1_OFFSET	0x880
#define MPDGHWST2_OFFSET	0x884
#define MPDGHWST3_OFFSET	0x888
#define MPDGCTRL1_OFFSET	0x840
#define MPRDDLHWCTL_OFFSET	0x860
#define MPWRDLCTL_OFFSET	0x850
#define MPWRDLHWCTL_OFFSET	0x864

#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0	(IOMUXC_BASE_ADDR + 0x5a8)
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1	(IOMUXC_BASE_ADDR + 0x5b0)
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2	(IOMUXC_BASE_ADDR + 0x524)
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3	(IOMUXC_BASE_ADDR + 0x51c)
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4	(IOMUXC_BASE_ADDR + 0x518)
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5	(IOMUXC_BASE_ADDR + 0x50c)
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6	(IOMUXC_BASE_ADDR + 0x5b8)
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7	(IOMUXC_BASE_ADDR + 0x5c0)

static inline u32 reg32_read(u32 reg)
{
	return readl(reg);
}

static inline void reg32_write(u32 reg, u32 val)
{
	writel(val, reg);
}

static inline void reg32setbit(u32 reg, int bit)
{
	u32 val;

	val = readl(reg);
	val |= 1 << bit;
	writel(val, reg);
}

static inline void reg32clrbit(u32 reg, int bit)
{
	u32 val;

	val = readl(reg);
	val &= ~(1 << bit);
	writel(val, reg);
}

int mmdc_do_write_level_calibration(void)
{
	u32 esdmisc_val, zq_val;
	int errorcount = 0;
	u32 val;
	u32 ddr_mr1 = 0x4;

	/* disable DDR logic power down timer */
	val = readl((MMDC_P0_IPS_BASE_ADDR + MDPDC_OFFSET));
	val &= 0xffff00ff;
	writel(val, (MMDC_P0_IPS_BASE_ADDR + MDPDC_OFFSET)),

	/* disable Adopt power down timer */
	val = readl((MMDC_P0_IPS_BASE_ADDR + MAPSR_OFFSET));
	val |= 0x1;
	writel(val, (MMDC_P0_IPS_BASE_ADDR + MAPSR_OFFSET));

	printf("Start write leveling calibration \n");

	/*
	 * disable auto refresh and ZQ calibration
	 * before proceeding with Write Leveling calibration
	 */
	esdmisc_val = readl(MMDC_P0_IPS_BASE_ADDR + MDREF_OFFSET);
	writel(0x0000C000, (MMDC_P0_IPS_BASE_ADDR + MDREF_OFFSET));
	zq_val = readl(MMDC_P0_IPS_BASE_ADDR + MPZQHWCTRL_OFFSET);
	writel(zq_val & ~(0x3), (MMDC_P0_IPS_BASE_ADDR + MPZQHWCTRL_OFFSET));

	/*
	 * Configure the external DDR device to enter write leveling mode
	 * through Load Mode Register command
	 * Register setting:
	 * Bits[31:16] MR1 value (0x0080 write leveling enable)
	 * Bit[9] set WL_EN to enable MMDC DQS output
	 * Bits[6:4] set CMD bits for Load Mode Register programming
	 * Bits[2:0] set CMD_BA to 0x1 for DDR MR1 programming
	 */
	writel(0x00808231, MMDC_P0_IPS_BASE_ADDR + MDSCR_OFFSET);

	/* Activate automatic calibration by setting MPWLGCR[HW_WL_EN] */
	writel(0x00000001, MMDC_P0_IPS_BASE_ADDR + MPWLGCR_OFFSET);

	/* Upon completion of this process the MMDC de-asserts the MPWLGCR[HW_WL_EN] */
	while (readl(MMDC_P0_IPS_BASE_ADDR + MPWLGCR_OFFSET) & 0x00000001);

	/* check for any errors: check both PHYs for x64 configuration, if x32, check only PHY0 */
	if ((readl(MMDC_P0_IPS_BASE_ADDR + MPWLGCR_OFFSET) & 0x00000F00) ||
			(readl(MMDC_P1_IPS_BASE_ADDR + MPWLGCR_OFFSET) & 0x00000F00)) {
		errorcount++;
	}

	printf("Write leveling calibration completed\n");

	/*
	 * User should issue MRS command to exit write leveling mode
	 * through Load Mode Register command
	 * Register setting:
	 * Bits[31:16] MR1 value "ddr_mr1" value from initialization
	 * Bit[9] clear WL_EN to disable MMDC DQS output
	 * Bits[6:4] set CMD bits for Load Mode Register programming
	 * Bits[2:0] set CMD_BA to 0x1 for DDR MR1 programming
	 */
	writel(((ddr_mr1 << 16)+0x8031), MMDC_P0_IPS_BASE_ADDR + MDSCR_OFFSET);

	/* re-enable to auto refresh and zq cal */
	writel(esdmisc_val, (MMDC_P0_IPS_BASE_ADDR + MDREF_OFFSET));
	writel(zq_val, (MMDC_P0_IPS_BASE_ADDR + MPZQHWCTRL_OFFSET));

	printf("MMDC_MPWLDECTRL0 after write level cal: 0x%08X\n",
			readl(MMDC_P0_IPS_BASE_ADDR + MPWLDECTRL0_OFFSET));
	printf("MMDC_MPWLDECTRL1 after write level cal: 0x%08X\n",
			readl(MMDC_P0_IPS_BASE_ADDR + MPWLDECTRL1_OFFSET));
	printf("MMDC_MPWLDECTRL0 after write level cal: 0x%08X\n",
			readl(MMDC_P1_IPS_BASE_ADDR + MPWLDECTRL0_OFFSET));
	printf("MMDC_MPWLDECTRL1 after write level cal: 0x%08X\n",
			readl(MMDC_P1_IPS_BASE_ADDR + MPWLDECTRL1_OFFSET));

	/* enable DDR logic power down timer */
	val = readl((MMDC_P0_IPS_BASE_ADDR + MDPDC_OFFSET));
	val |= 0x00005500;
	writel(val, (MMDC_P0_IPS_BASE_ADDR + MDPDC_OFFSET));

	/* enable Adopt power down timer: */
	val = readl(MMDC_P0_IPS_BASE_ADDR + MAPSR_OFFSET);
	val &= 0xfffffff7;
	writel(val, (MMDC_P0_IPS_BASE_ADDR + MAPSR_OFFSET));
	/* clear CON_REQ */
	writel(0, (MMDC_P0_IPS_BASE_ADDR + MDSCR_OFFSET));

	return 0;
}

static void modify_dg_result(int reg_st0, int reg_st1, int reg_ctrl)
{
	u32 dg_tmp_val, dg_dl_abs_offset, dg_hc_del, val_ctrl;

	/*
	 * DQS gating absolute offset should be modified from reflecting (HW_DG_LOWx + HW_DG_UPx)/2
	 * to reflecting (HW_DG_UPx - 0x80)
	 */

	val_ctrl = reg32_read(reg_ctrl);
	val_ctrl &= 0xf0000000;

	dg_tmp_val = ((reg32_read(reg_st0) & 0x07ff0000) >> 16) - 0xc0;
	dg_dl_abs_offset = dg_tmp_val & 0x7f;
	dg_hc_del = (dg_tmp_val & 0x780) << 1;

	val_ctrl |= dg_dl_abs_offset + dg_hc_del;

	dg_tmp_val = ((reg32_read(reg_st1) & 0x07ff0000) >> 16) - 0xc0;
	dg_dl_abs_offset = dg_tmp_val & 0x7f;
	dg_hc_del = (dg_tmp_val & 0x780) << 1;

	val_ctrl |= (dg_dl_abs_offset + dg_hc_del) << 16;

	reg32_write(reg_ctrl, val_ctrl);
}

static void mmdc_precharge_all(int cs0_enable, int cs1_enable)
{
	/*
	 * Issue the Precharge-All command to the DDR device for both chip selects
	 * Note, CON_REQ bit should also remain set
	 * If only using one chip select, then precharge only the desired chip select
	 */
	if (cs0_enable)
		reg32_write((MMDC_P0_IPS_BASE_ADDR + MDSCR_OFFSET), 0x04008050);
	if (cs1_enable)
		reg32_write((MMDC_P0_IPS_BASE_ADDR + MDSCR_OFFSET), 0x04008058);
}

static void mmdc_force_delay_measurement(int data_bus_size)
{
	reg32_write(MMDC_P0_IPS_BASE_ADDR + MPMUR_OFFSET, 0x800);

	if (data_bus_size == 0x2)
		reg32_write(MMDC_P1_IPS_BASE_ADDR + MPMUR_OFFSET, 0x800);
}

static void mmdc_reset_read_data_fifos(void)
{
	/*
	 * Reset the read data FIFOs (two resets); only need to issue reset to PHY0 since in x64
	 * mode, the reset will also go to PHY1
	 * read data FIFOs reset1
	 */
	reg32_write((MMDC_P0_IPS_BASE_ADDR + MPDGCTRL0_OFFSET),
			reg32_read((MMDC_P0_IPS_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);

	while (reg32_read((MMDC_P0_IPS_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000);

	/* read data FIFOs reset2 */
	reg32_write((MMDC_P0_IPS_BASE_ADDR + MPDGCTRL0_OFFSET),
	reg32_read((MMDC_P0_IPS_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);

	while (reg32_read((MMDC_P0_IPS_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000);
}

int mmdc_do_dqs_calibration(void)
{
	u32 esdmisc_val;
	int g_error_write_cal;
	int temp_ref;
	int cs0_enable_initial;
	int cs1_enable_initial;
	int PDDWord = 0x00ffff00;
	int errorcount = 0;
	int cs0_enable;
	int cs1_enable;
	int data_bus_size;

	/* check to see which chip selects are enabled */
	cs0_enable_initial = (reg32_read(MMDC_P0_IPS_BASE_ADDR + MDCTL_OFFSET) & 0x80000000) >> 31;
	cs1_enable_initial = (reg32_read(MMDC_P0_IPS_BASE_ADDR + MDCTL_OFFSET) & 0x40000000) >> 30;

	/* disable DDR logic power down timer */
	reg32_write(MMDC_P0_IPS_BASE_ADDR + MDPDC_OFFSET,
			reg32_read((MMDC_P0_IPS_BASE_ADDR + MDPDC_OFFSET)) & 0xffff00ff);

	/* disable Adopt power down timer */
	reg32_write(MMDC_P0_IPS_BASE_ADDR + MAPSR_OFFSET,
			reg32_read((MMDC_P0_IPS_BASE_ADDR + MAPSR_OFFSET)) | 0x1);

	/* set DQS pull ups */
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0, reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0) | 0x7000);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1, reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1) | 0x7000);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2, reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2) | 0x7000);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3, reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3) | 0x7000);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4, reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4) | 0x7000);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5, reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5) | 0x7000);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6, reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6) | 0x7000);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7, reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7) | 0x7000);

	esdmisc_val = reg32_read(MMDC_P0_IPS_BASE_ADDR + MDMISC_OFFSET);

	reg32setbit(MMDC_P0_IPS_BASE_ADDR + MDMISC_OFFSET, 6); /* set RALAT to max */
	reg32setbit(MMDC_P0_IPS_BASE_ADDR + MDMISC_OFFSET, 7);
	reg32setbit(MMDC_P0_IPS_BASE_ADDR + MDMISC_OFFSET, 8);
	reg32setbit(MMDC_P0_IPS_BASE_ADDR + MDMISC_OFFSET, 16); /* set WALAT to max */
	reg32setbit(MMDC_P0_IPS_BASE_ADDR + MDMISC_OFFSET, 17);

	/*
	 * disable auto refresh
	 * before proceeding with calibration
	 */
	temp_ref = reg32_read(MMDC_P0_IPS_BASE_ADDR + MDREF_OFFSET);
	reg32_write(MMDC_P0_IPS_BASE_ADDR + MDREF_OFFSET, 0x0000C000);

	/*
	 * per the ref manual, issue one refresh cycle MDSCR[CMD]= 0x2,
	 * this also sets the CON_REQ bit.
	 */
	if (cs0_enable_initial)
		reg32_write(MMDC_P0_IPS_BASE_ADDR + MDSCR_OFFSET, 0x00008020);
	if (cs1_enable_initial)
		reg32_write(MMDC_P0_IPS_BASE_ADDR + MDSCR_OFFSET, 0x00008028);

	/* poll to make sure the con_ack bit was asserted */
	while (!(reg32_read(MMDC_P0_IPS_BASE_ADDR + MDSCR_OFFSET) & 0x00004000)) ;

	/*
	 * check MDMISC register CALIB_PER_CS to see which CS calibration is	 * targeted to (under normal cases, it should be cleared as this is the
	 * default value, indicating calibration is directed to CS0). Disable
	 * the other chip select not being target for calibration to avoid any
	 * potential issues This will get re-enabled at end of calibration.
	 */
	if ((reg32_read(MMDC_P0_IPS_BASE_ADDR + MDMISC_OFFSET) & 0x00100000) == 0)
		reg32clrbit(MMDC_P0_IPS_BASE_ADDR + MDCTL_OFFSET, 30); /* clear SDE_1 */
	else
		reg32clrbit(MMDC_P0_IPS_BASE_ADDR + MDCTL_OFFSET, 31); /* clear SDE_0 */

	/*
	 * check to see which chip selects are now enabled for the remainder
	 * of the calibration.
	 */
	cs0_enable = (reg32_read(MMDC_P0_IPS_BASE_ADDR + MDCTL_OFFSET) & 0x80000000) >> 31;
	cs1_enable = (reg32_read(MMDC_P0_IPS_BASE_ADDR + MDCTL_OFFSET) & 0x40000000) >> 30;

	/* check to see what is the data bus size */
	data_bus_size = (reg32_read(MMDC_P0_IPS_BASE_ADDR + MDCTL_OFFSET) & 0x30000) >> 16;

	mmdc_precharge_all(cs0_enable, cs1_enable);

	/* Write the pre-defined value into MPPDCMPR1 */
	reg32_write((MMDC_P0_IPS_BASE_ADDR + MPPDCMPR1_OFFSET), PDDWord);

	/*
	 * Issue a write access to the external DDR device by setting the bit SW_DUMMY_WR (bit 0)
	 * in the MPSWDAR0 and then poll this bit until it clears to indicate completion of the
	 * write access.
	 */
	reg32setbit(MMDC_P0_IPS_BASE_ADDR + MPSWDAR_OFFSET, 0);

	while (reg32_read(MMDC_P0_IPS_BASE_ADDR + MPSWDAR_OFFSET) & 0x00000001);

	/*
	 * Set the RD_DL_ABS_OFFSET# bits to their default values (will be calibrated later in
	 * the read delay-line calibration)
	 * Both PHYs for x64 configuration, if x32, do only PHY0
	 */
	reg32_write(MMDC_P0_IPS_BASE_ADDR + MPRDDLCTL_OFFSET, 0x40404040);
	if (data_bus_size == 0x2)
		reg32_write(MMDC_P1_IPS_BASE_ADDR + MPRDDLCTL_OFFSET, 0x40404040);

	/* Force a measurement, for previous delay setup to take effect */
	mmdc_force_delay_measurement(data_bus_size);

	/*
	 * Read DQS Gating calibration
	 */

	printf("Starting DQS gating calibration...\n");

	mmdc_reset_read_data_fifos();

	/*
	 * Start the automatic read DQS gating calibration process by asserting
	 * MPDGCTRL0[HW_DG_EN] and MPDGCTRL0[DG_CMP_CYC] and then poll
	 * MPDGCTRL0[HW_DG_EN]] until this bit clears to indicate completion.
	 * Also, ensure that MPDGCTRL0[HW_DG_ERR] is clear to indicate no errors
	 * were seen during calibration. Set bit 30: chooses option to wait 32
	 * cycles instead of 16 before comparing read data
	 */
	reg32setbit(MMDC_P0_IPS_BASE_ADDR + MPDGCTRL0_OFFSET, 30);

	/* Set bit 28 to start automatic read DQS gating calibration */
	reg32setbit(MMDC_P0_IPS_BASE_ADDR + MPDGCTRL0_OFFSET, 28);

	/*
	 * Poll for completion
	 * MPDGCTRL0[HW_DG_EN] should be 0
	 */
	while (reg32_read(MMDC_P0_IPS_BASE_ADDR + MPDGCTRL0_OFFSET) & 0x10000000);

	/*
	 * Check to see if any errors were encountered during calibration
	 * (check MPDGCTRL0[HW_DG_ERR])
	 * check both PHYs for x64 configuration, if x32, check only PHY0
	 */
	if (data_bus_size == 0x2) {
		if ((reg32_read(MMDC_P0_IPS_BASE_ADDR + MPDGCTRL0_OFFSET) & 0x00001000) ||
				(reg32_read(MMDC_P1_IPS_BASE_ADDR + MPDGCTRL0_OFFSET) & 0x00001000))
			errorcount++;
	} else {
		if (reg32_read(MMDC_P0_IPS_BASE_ADDR + MPDGCTRL0_OFFSET) & 0x00001000)
			errorcount++;
	}

	/*
	 * DQS gating absolute offset should be modified from reflecting
	 * (HW_DG_LOWx + HW_DG_UPx)/2 to reflecting (HW_DG_UPx - 0x80)
	 */
	modify_dg_result(MMDC_P0_IPS_BASE_ADDR + MPDGHWST0_OFFSET,
			MMDC_P0_IPS_BASE_ADDR + MPDGHWST1_OFFSET,
			MMDC_P0_IPS_BASE_ADDR + MPDGCTRL0_OFFSET);

	modify_dg_result(MMDC_P0_IPS_BASE_ADDR + MPDGHWST2_OFFSET,
			MMDC_P0_IPS_BASE_ADDR + MPDGHWST3_OFFSET,
			MMDC_P0_IPS_BASE_ADDR + MPDGCTRL1_OFFSET);

	if (data_bus_size == 0x2) {
		modify_dg_result(MMDC_P1_IPS_BASE_ADDR + MPDGHWST0_OFFSET,
				MMDC_P1_IPS_BASE_ADDR + MPDGHWST1_OFFSET,
				MMDC_P1_IPS_BASE_ADDR + MPDGCTRL0_OFFSET);
		modify_dg_result(MMDC_P1_IPS_BASE_ADDR + MPDGHWST2_OFFSET,
				MMDC_P1_IPS_BASE_ADDR + MPDGHWST3_OFFSET,
				MMDC_P1_IPS_BASE_ADDR + MPDGCTRL1_OFFSET);
	}

	printf("DQS gating calibration completed.\n");

	/*
	 * Read delay Calibration
	 */

	printf("Starting read calibration...\n");

	mmdc_reset_read_data_fifos();

	mmdc_precharge_all(cs0_enable, cs1_enable);

	/*
	 * Read delay-line calibration
	 * Start the automatic read calibration process by asserting MPRDDLHWCTL[ HW_RD_DL_EN]
	 */
	reg32_write(MMDC_P0_IPS_BASE_ADDR + MPRDDLHWCTL_OFFSET, 0x00000030);

	/*
	 * poll for completion
	 * MMDC indicates that the write data calibration had finished by setting
	 * MPRDDLHWCTL[HW_RD_DL_EN] = 0
	 * Also, ensure that no error bits were set
	 */
	while (reg32_read(MMDC_P0_IPS_BASE_ADDR + MPRDDLHWCTL_OFFSET) & 0x00000010) ;

	/* check both PHYs for x64 configuration, if x32, check only PHY0 */
	if (data_bus_size == 0x2) {
		if ((reg32_read(MMDC_P0_IPS_BASE_ADDR + MPRDDLHWCTL_OFFSET) & 0x0000000f) ||
				(reg32_read(MMDC_P1_IPS_BASE_ADDR + MPRDDLHWCTL_OFFSET) & 0x0000000f)) {
			errorcount++;
		}
	} else {
		if (reg32_read(MMDC_P0_IPS_BASE_ADDR + MPRDDLHWCTL_OFFSET) & 0x0000000f) {
			errorcount++;
		}
	}

	printf("Read calibration completed\n");

	/*
	 * Write delay Calibration
	 */

	printf("Starting write calibration...\n");

	mmdc_reset_read_data_fifos();

	mmdc_precharge_all(cs0_enable, cs1_enable);

	/*
	 * Set the WR_DL_ABS_OFFSET# bits to their default values
	 * Both PHYs for x64 configuration, if x32, do only PHY0
	 */
	reg32_write(MMDC_P0_IPS_BASE_ADDR + MPWRDLCTL_OFFSET, 0x40404040);
	if (data_bus_size == 0x2)
		reg32_write(MMDC_P1_IPS_BASE_ADDR + MPWRDLCTL_OFFSET, 0x40404040);

	mmdc_force_delay_measurement(data_bus_size);

	/* Start the automatic write calibration process by asserting MPWRDLHWCTL0[HW_WR_DL_EN] */
	reg32_write(MMDC_P0_IPS_BASE_ADDR + MPWRDLHWCTL_OFFSET, 0x00000030);

	/*
	 * poll for completion
	 * MMDC indicates that the write data calibration had finished by setting
	 * MPWRDLHWCTL[HW_WR_DL_EN] = 0
	 * Also, ensure that no error bits were set
	 */
	while (reg32_read(MMDC_P0_IPS_BASE_ADDR + MPWRDLHWCTL_OFFSET) & 0x00000010) ;

	/* check both PHYs for x64 configuration, if x32, check only PHY0 */
	if (data_bus_size == 0x2) {
		if ((reg32_read(MMDC_P0_IPS_BASE_ADDR + MPWRDLHWCTL_OFFSET) & 0x0000000f) ||
				(reg32_read(MMDC_P1_IPS_BASE_ADDR + MPWRDLHWCTL_OFFSET) & 0x0000000f)) {
			errorcount++;
			g_error_write_cal = 1; // set the g_error_write_cal
		}
	} else {
		if (reg32_read(MMDC_P0_IPS_BASE_ADDR + MPWRDLHWCTL_OFFSET) & 0x0000000f) {
			errorcount++;
			g_error_write_cal = 1; // set the g_error_write_cal
		}
	}

	printf("Write calibration completed\n");

	mmdc_reset_read_data_fifos();

	printf("\n");

	/* enable DDR logic power down timer */
	reg32_write(MMDC_P0_IPS_BASE_ADDR + MDPDC_OFFSET,
			reg32_read(MMDC_P0_IPS_BASE_ADDR + MDPDC_OFFSET) | 0x00005500);

	/* enable Adopt power down timer */
	reg32_write(MMDC_P0_IPS_BASE_ADDR + MAPSR_OFFSET,
		reg32_read(MMDC_P0_IPS_BASE_ADDR + MAPSR_OFFSET) & 0xfffffff7);

	/* restore MDMISC value (RALAT, WALAT) */
	reg32_write(MMDC_P1_IPS_BASE_ADDR + MDMISC_OFFSET, esdmisc_val);

	/* clear DQS pull ups */
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0, reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0) & 0xffff0fff);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1, reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1) & 0xffff0fff);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2, reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2) & 0xffff0fff);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3, reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3) & 0xffff0fff);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4, reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4) & 0xffff0fff);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5, reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5) & 0xffff0fff);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6, reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6) & 0xffff0fff);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7, reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7) & 0xffff0fff);

	/* re-enable SDE (chip selects) if they were set initially */
	if (cs1_enable_initial == 1)
		reg32setbit(MMDC_P0_IPS_BASE_ADDR + MDCTL_OFFSET, 30); /* set SDE_1 */

	if (cs0_enable_initial == 1)
		reg32setbit(MMDC_P0_IPS_BASE_ADDR + MDCTL_OFFSET, 31); /* set SDE_0 */

	/* re-enable to auto refresh */
	reg32_write(MMDC_P0_IPS_BASE_ADDR + MDREF_OFFSET, temp_ref);

	/* clear the MDSCR (including the con_req bit) */
	reg32_write(MMDC_P0_IPS_BASE_ADDR + MDSCR_OFFSET, 0x0); /* CS0 */

	/* poll to make sure the con_ack bit is clear */
	while (reg32_read(MMDC_P0_IPS_BASE_ADDR + MDSCR_OFFSET) & 0x00004000) ;

	printf("MMDC registers updated from calibration \n");
	printf("\nRead DQS Gating calibration\n");
	printf("MPDGCTRL0 PHY0 (0x021b083c) = 0x%08X\n", reg32_read(0x021b083c));
	printf("MPDGCTRL1 PHY0 (0x021b0840) = 0x%08X\n", reg32_read(0x021b0840));
	printf("MPDGCTRL0 PHY1 (0x021b483c) = 0x%08X\n", reg32_read(0x021b483c));
	printf("MPDGCTRL1 PHY1 (0x021b4840) = 0x%08X\n", reg32_read(0x021b4840));
	printf("\nRead calibration\n");
	printf("MPRDDLCTL PHY0 (0x021b0848) = 0x%08X\n", reg32_read(0x021b0848));
	printf("MPRDDLCTL PHY1 (0x021b4848) = 0x%08X\n", reg32_read(0x021b4848));
	printf("\nWrite calibration\n");
	printf("MPWRDLCTL PHY0 (0x021b0850) = 0x%08X\n", reg32_read(0x021b0850));
	printf("MPWRDLCTL PHY1 (0x021b4850) = 0x%08X\n", reg32_read(0x021b4850));
	printf("\n");
	/*
	 * registers below are for debugging purposes
	 * these print out the upper and lower boundaries captured during read DQS gating calibration
	 */
	printf("Status registers, upper and lower bounds, for read DQS gating. \n");
	printf("MPDGHWST0 PHY0 (0x021b087c) = 0x%08X\n", reg32_read(0x021b087c));
	printf("MPDGHWST1 PHY0 (0x021b0880) = 0x%08X\n", reg32_read(0x021b0880));
	printf("MPDGHWST2 PHY0 (0x021b0884) = 0x%08X\n", reg32_read(0x021b0884));
	printf("MPDGHWST3 PHY0 (0x021b0888) = 0x%08X\n", reg32_read(0x021b0888));
	printf("MPDGHWST0 PHY1 (0x021b487c) = 0x%08X\n", reg32_read(0x021b487c));
	printf("MPDGHWST1 PHY1 (0x021b4880) = 0x%08X\n", reg32_read(0x021b4880));
	printf("MPDGHWST2 PHY1 (0x021b4884) = 0x%08X\n", reg32_read(0x021b4884));
	printf("MPDGHWST3 PHY1 (0x021b4888) = 0x%08X\n", reg32_read(0x021b4888));

	return errorcount;
}

#ifdef MMDC_SOFTWARE_CALIBRATION

static void mmdc_set_dqs(u32 value)
{
	u32 data_bus_size;

	value |= value << 8 | value << 16 | value << 24;

	reg32_write(MMDC_P0_IPS_BASE_ADDR + MPRDDLCTL_OFFSET, value);


	/* check to see what is the data bus size */
	data_bus_size = (reg32_read(MMDC_P0_IPS_BASE_ADDR + MDCTL_OFFSET) & 0x30000) >> 16;

	if (data_bus_size == 0x2)
		reg32_write(MMDC_P1_IPS_BASE_ADDR + MPRDDLCTL_OFFSET, value);
}

static void mmdc_set_wr_delay(u32 value)
{
	u32 data_bus_size;

	value |= value << 8 | value << 16 | value << 24;

	reg32_write(MMDC_P0_IPS_BASE_ADDR + MPWRDLCTL_OFFSET, value);

	/* check to see what is the data bus size */
	data_bus_size = (reg32_read(MMDC_P0_IPS_BASE_ADDR + MDCTL_OFFSET) & 0x30000) >> 16;

	if (data_bus_size == 0x2)
		reg32_write(MMDC_P1_IPS_BASE_ADDR + MPWRDLCTL_OFFSET, value);
}

static void mmdc_issue_write_access(void __iomem *base)
{
	/*
	 * Issue a write access to the external DDR device by setting the bit SW_DUMMY_WR (bit 0)
	 * in the MPSWDAR0 and then poll this bit until it clears to indicate completion of the
	 * write access.
	 */

	reg32setbit(MMDC_P0_IPS_BASE_ADDR + MPSWDAR_OFFSET, 0);

	while (reg32_read(MMDC_P0_IPS_BASE_ADDR + MPSWDAR_OFFSET) & 0x00000001);
}

static void mmdc_issue_read_access(void __iomem *base)
{
	/*
	 * Issue a write access to the external DDR device by setting the bit SW_DUMMY_WR (bit 0)
	 * in the MPSWDAR0 and then poll this bit until it clears to indicate completion of the
	 * write access.
	 */

	reg32setbit(MMDC_P0_IPS_BASE_ADDR + MPSWDAR_OFFSET, 1);

	while (reg32_read(MMDC_P0_IPS_BASE_ADDR + MPSWDAR_OFFSET) & 0x00000002);
}

static int total_lower[2] = { 0x0, 0x0 };
static int total_upper[2] = { 0xff, 0xff };

static int mmdc_is_valid(void __iomem *base, int delay, int rd)
{
	u32 val;
	u32 data_bus_size;

	if (rd)
		mmdc_set_dqs(delay);
	else
		mmdc_set_wr_delay(delay);

	/* check to see what is the data bus size */
	data_bus_size = (reg32_read(MMDC_P0_IPS_BASE_ADDR + MDCTL_OFFSET) & 0x30000) >> 16;

	mmdc_force_delay_measurement(data_bus_size);

	mdelay(1);

	if (!rd)
		mmdc_issue_write_access(base);

	mmdc_issue_read_access(base);

	val = readl(base + MPSWDAR_OFFSET);

	if ((val & 0x3c) == 0x3c)
		return 1;
	else
		return 0;
#ifdef MMDC_SOFWARE_CALIB_COMPARE_RESULTS
	if ((val & 0x3c) == 0x3c) {
		if (lower < 0)
			lower = i;
	}

	if ((val & 0x3c) != 0x3c) {
		if (lower > 0 && upper < 0)
			upper = i;
	}

	printf("0x%02x: compare: 0x%08x ", i, readl(MMDC_P0_IPS_BASE_ADDR + MPSWDAR_OFFSET));
	for (j = 0; j < 8; j++) {
		printf("0x%08x ", readl(MMDC_P0_IPS_BASE_ADDR + 0x898 + j * 4));
	}
	printf("\n");
#endif
}

static void mmdc_sw_read_calib(int ch, u32 wr_value)
{
	int rd = 1;
	void __iomem *base;
	int i;
	int lower = 0x0, upper = 0x7f;
	int cs0_enable_initial;
	int cs1_enable_initial;

	if (ch)
		base = (void *)MMDC_P1_IPS_BASE_ADDR;
	else
		base = (void *)MMDC_P0_IPS_BASE_ADDR;

	/* check to see which chip selects are enabled */
	cs0_enable_initial = (reg32_read(base + MDCTL_OFFSET) & 0x80000000) >> 31;
	cs1_enable_initial = (reg32_read(base + MDCTL_OFFSET) & 0x40000000) >> 30;

	/* 1. Precharge */
	mmdc_precharge_all(cs0_enable_initial, cs1_enable_initial);

	/* 2. Configure pre-defined value */
	reg32_write(MMDC_P0_IPS_BASE_ADDR + MPPDCMPR1_OFFSET, wr_value);

	/* 3. Issue write access */
	mmdc_issue_write_access(base);

	for (i = 0; i < 100; i++) {
		if (mmdc_is_valid(base, 0x40, rd)) {
			goto middle_passed;
		}
	}

	printf("ch: %d value: 0x%08x middle test failed\n", ch, wr_value);
	return;

middle_passed:
	for (i = 0x40; i < 0x7f; i++) {
		if (!mmdc_is_valid(base, i, rd)) {
			upper = i;
			break;
		}
	}

	for (i = 0; i < 100; i++) {
		if (mmdc_is_valid(base, 0x40, rd)) {
			goto go_on;
		}
	}

	printf("ch: %d value: 0x%08x middle test 1 failed\n", ch, wr_value);
	return;

go_on:
	for (i = 0x40; i >= 0; i--) {
		if (!mmdc_is_valid(base, i, rd)) {
			lower = i;
			break;
		}
	}

	if (lower > total_lower[ch])
		total_lower[ch] = lower;

	if (upper < total_upper[ch])
		total_upper[ch] = upper;

	printf("ch: %d value: 0x%08x lower: %-3d upper: %-3d\n", ch, wr_value, lower, upper);
	}

static void mmdc_sw_write_calib(int ch, u32 wr_value)
{
	int rd = 0;
	void __iomem *base;
	int i;
	int lower = 0x0, upper = 0x7f;
	int cs0_enable_initial;
	int cs1_enable_initial;

	if (ch)
		base = (void *)MMDC_P1_IPS_BASE_ADDR;
	else
		base = (void *)MMDC_P0_IPS_BASE_ADDR;


	/* check to see which chip selects are enabled */
	cs0_enable_initial = (reg32_read(base + MDCTL_OFFSET) & 0x80000000) >> 31;
	cs1_enable_initial = (reg32_read(base + MDCTL_OFFSET) & 0x40000000) >> 30;

	/* 1. Precharge */
	mmdc_precharge_all(cs0_enable_initial, cs1_enable_initial);

	/* 2. Configure pre-defined value */
	reg32_write(MMDC_P0_IPS_BASE_ADDR + MPPDCMPR1_OFFSET, wr_value);

	/* 3. Issue write access */
	mmdc_issue_write_access(base);

	for (i = 0; i < 100; i++) {
		if (mmdc_is_valid(base, 0x40, rd)) {
			goto middle_passed;
		}
	}

	printf("ch: %d value: 0x%08x middle test failed\n", ch, wr_value);
	return;

middle_passed:
	for (i = 0x40; i < 0x7f; i++) {
		if (!mmdc_is_valid(base, i, rd)) {
			upper = i;
			break;
		}
	}

	for (i = 0; i < 100; i++) {
		if (mmdc_is_valid(base, 0x40, rd)) {
			goto go_on;
		}
	}

	printf("ch: %d value: 0x%08x middle test 1 failed\n", ch, wr_value);
	return;

go_on:
	for (i = 0x40; i >= 0; i--) {
		if (!mmdc_is_valid(base, i, rd)) {
			lower = i;
			break;
		}
	}

	if (lower > total_lower[ch])
		total_lower[ch] = lower;

	if (upper < total_upper[ch])
		total_upper[ch] = upper;

	printf("ch: %d value: 0x%08x lower: %-3d upper: %-3d\n", ch, wr_value, lower, upper);
}

int mmdc_do_software_calibration(void)
{
	u32 s;
	int ch;

	printf("mmdc_do_software_calibration\r\n");

	for (ch = 0; ch < 2; ch++) {
		mmdc_sw_read_calib(ch, 0x00000055);
		mmdc_sw_read_calib(ch, 0x00005500);
		mmdc_sw_read_calib(ch, 0x00550000);
		mmdc_sw_read_calib(ch, 0x55000000);
		mmdc_sw_read_calib(ch, 0x00ffff00);
		mmdc_sw_read_calib(ch, 0xff0000ff);
		mmdc_sw_read_calib(ch, 0x55aaaa55);
		mmdc_sw_read_calib(ch, 0xaa5555aa);

		for (s = 1; s; s <<= 1)
			mmdc_sw_read_calib(ch, s);
	}

	printf("ch0 total lower: %d upper: %d avg: 0x%02x\n",
			total_lower[0], total_upper[0],
			(total_lower[0] + total_upper[0]) / 2);
	printf("ch1 total lower: %d upper: %d avg: 0x%02x\n",
			total_lower[1], total_upper[1],
			(total_lower[1] + total_upper[1]) / 2);

	mmdc_set_dqs(0x40);

	total_lower[0] = 0;
	total_lower[1] = 0;
	total_upper[0] = 0xff;
	total_upper[1] = 0xff;

	for (ch = 0; ch < 2; ch++) {
		mmdc_sw_write_calib(ch, 0x00000055);
		mmdc_sw_write_calib(ch, 0x00005500);
		mmdc_sw_write_calib(ch, 0x00550000);
		mmdc_sw_write_calib(ch, 0x55000000);
		mmdc_sw_write_calib(ch, 0x00ffff00);
		mmdc_sw_write_calib(ch, 0xff0000ff);
		mmdc_sw_write_calib(ch, 0x55aaaa55);
		mmdc_sw_write_calib(ch, 0xaa5555aa);

		for (s = 1; s; s <<= 1)
			mmdc_sw_write_calib(ch, s);
	}

	printf("ch0 total lower: %d upper: %d avg: 0x%02x\n",
			total_lower[0], total_upper[0],
			(total_lower[0] + total_upper[0]) / 2);
	printf("ch1 total lower: %d upper: %d avg: 0x%02x\n",
			total_lower[1], total_upper[1],
			(total_lower[1] + total_upper[1]) / 2);

	return 0;
}
#endif /* MMDC_SOFTWARE_CALIBRATION */