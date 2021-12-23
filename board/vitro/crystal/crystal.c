/*
 * Copyright (C) 2017 Vitro Technology Corporation
 *
 * Configuration settings for the Vitro Crystal i.MX6 platform
 *
 * Initialization based on similar i.MX6-based platforms (mostly mx6cuboxi
 * and mx6sabresd targets)
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <linux/types.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/mxc_hdmi.h>
#include <linux/errno.h>
#include <asm/u-boot.h>
#include <asm/global_data.h>
#include <asm/arch/crm_regs.h>
#include <asm/mach-imx/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/video.h>
#include <mmc.h>
#include <fsl_esdhc_imx.h>
#include <malloc.h>
#include <netdev.h>
#include <miiphy.h>
#include <linux/delay.h>

/* It can be used before RAM is initialized
 * It is general purpose register, used in code with "gd" variable (r9 for
 * ARM32)
 */
DECLARE_GLOBAL_DATA_PTR;

/*** UART ***/
#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

static iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart1_pads);
}

/*** uSD card slot ***/
#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

static iomux_v3_cfg_t const usdhc2_pads[] = {
	IOMUX_PADS(PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT0__SD2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_4__GPIO1_IO04	| MUX_PAD_CTRL(NO_PAD_CTRL)), /* CD */
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	IOMUX_PADS(PAD_SD3_CLK__SD3_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_CMD__SD3_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT1__SD3_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT2__SD3_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT3__SD3_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT4__SD3_DATA4	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT5__SD3_DATA5	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT6__SD3_DATA6	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT7__SD3_DATA7	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};


#define CRYSTAL_USDHC_USD_IDX	0
#define CRYSTAL_USDHC_EMMC_IDX	1
#define CRYSTAL_USDHC_USD_BASE	USDHC2_BASE_ADDR
#define CRYSTAL_USDHC_EMMC_BASE	USDHC3_BASE_ADDR

static struct fsl_esdhc_cfg usdhc_cfg[CONFIG_SYS_FSL_USDHC_NUM] = {
       {.esdhc_base = CRYSTAL_USDHC_USD_BASE},
       {.esdhc_base = CRYSTAL_USDHC_EMMC_BASE},
};

/* port 1 pad 4 */
#define USDHC2_CD_GPIO	IMX_GPIO_NR(1, 4)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case CRYSTAL_USDHC_USD_BASE:
		/* signal is low if card is inserted */
		ret = !gpio_get_value(USDHC2_CD_GPIO);
		break;
	case CRYSTAL_USDHC_EMMC_BASE :
		/* Assume eMMC is always present */
		ret = 1;
		break;
	}

	return ret;

	/* return 0 if card not inserted or unknown esdhc_base */
	return ret;
}

int board_mmc_init(struct bd_info *bis)
{
	int i, ret;

	/*
	 * (U-Boot device node)    (device)
	 * mmc1                    USDHC2 (uSD card)
	 * mmc2                    USDHC3 (eMMC)
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case CRYSTAL_USDHC_USD_IDX:
			SETUP_IOMUX_PADS(usdhc2_pads);
			usdhc_cfg[CRYSTAL_USDHC_USD_IDX].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;
		case CRYSTAL_USDHC_EMMC_IDX:
			SETUP_IOMUX_PADS(usdhc3_pads);
			usdhc_cfg[CRYSTAL_USDHC_EMMC_IDX].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret) {
			printf("Warning: failed to initialize mmc dev %d\n", i);
			return ret;
		}
	}

	return 0;
}

/*** Ethernet ***/
#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_PAD_CTRL_PD  (PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_PAD_CTRL_CLK  ((PAD_CTL_PUS_100K_UP & ~PAD_CTL_PKE) | \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define ETH_PHY_RESET	IMX_GPIO_NR(4, 15)

static iomux_v3_cfg_t const enet_pads[] = {
	IOMUX_PADS(PAD_ENET_MDIO__ENET_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_MDC__ENET_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	/* AR8035 reset */
	IOMUX_PADS(PAD_KEY_ROW4__GPIO4_IO15 | MUX_PAD_CTRL(ENET_PAD_CTRL_PD)),
	/* AR8035 interrupt */
	IOMUX_PADS(PAD_DI0_PIN2__GPIO4_IO18 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* GPIO16 -> AR8035 25MHz */
	IOMUX_PADS(PAD_GPIO_16__ENET_REF_CLK	  | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TXC__RGMII_TXC	  | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD0__RGMII_TD0 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD1__RGMII_TD1 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD2__RGMII_TD2 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD3__RGMII_TD3 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	/* AR8035 CLK_25M --> ENET_REF_CLK (V22) */
	IOMUX_PADS(PAD_ENET_REF_CLK__ENET_TX_CLK | MUX_PAD_CTRL(ENET_PAD_CTRL_CLK)),
	IOMUX_PADS(PAD_RGMII_RXC__RGMII_RXC | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD0__RGMII_RD0 | MUX_PAD_CTRL(ENET_PAD_CTRL_PD)),
	IOMUX_PADS(PAD_RGMII_RD1__RGMII_RD1 | MUX_PAD_CTRL(ENET_PAD_CTRL_PD)),
	IOMUX_PADS(PAD_RGMII_RD2__RGMII_RD2 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD3__RGMII_RD3 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL_PD)),
	IOMUX_PADS(PAD_ENET_RXD0__GPIO1_IO27 | MUX_PAD_CTRL(ENET_PAD_CTRL_PD)),
	IOMUX_PADS(PAD_ENET_RXD1__GPIO1_IO26 | MUX_PAD_CTRL(ENET_PAD_CTRL_PD)),
};

static void setup_iomux_enet(void)
{
	SETUP_IOMUX_PADS(enet_pads);

	gpio_direction_output(ETH_PHY_RESET, 0);
	mdelay(10);
	gpio_set_value(ETH_PHY_RESET, 1);
	udelay(100);
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

/* On Vitro Crystal board Ethernet PHY is located at address 0x0 */
#define ETH_PHY_MASK	(1 << 0x0)

int board_eth_init(struct bd_info *bis)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	struct mii_dev *bus;
	struct phy_device *phydev;

	int ret = enable_fec_anatop_clock(0, ENET_25MHZ);
	if (ret)
		return ret;

	/* set gpr1[ENET_CLK_SEL] */
	setbits_le32(&iomuxc_regs->gpr[1], IOMUXC_GPR1_ENET_CLK_SEL_MASK);

	setup_iomux_enet();

	bus = fec_get_miibus(IMX_FEC_BASE, -1);
	if (!bus)
		return -EINVAL;

	phydev = phy_find_by_mask(bus, ETH_PHY_MASK, PHY_INTERFACE_MODE_RGMII);
	if (!phydev) {
		ret = -EINVAL;
		goto free_bus;
	}

	debug("using phy at address %d\n", phydev->addr);
	ret = fec_probe(bis, -1, IMX_FEC_BASE, bus, phydev);
	if (ret)
		goto free_phydev;

	return 0;

free_phydev:
	free(phydev);
free_bus:
	free(bus);
	return ret;
}

/*** Video ***/
#ifdef CONFIG_VIDEO_IPUV3
static void do_enable_hdmi(struct display_info_t const *dev)
{
	imx_enable_hdmi_phy();
}

struct display_info_t const displays[] = {
	{
		.bus	= -1,
		.addr	= 0,
		.pixfmt	= IPU_PIX_FMT_RGB24,
		.detect	= detect_hdmi,
		.enable	= do_enable_hdmi,
		.mode	= {
			.name           = "HDMI",
			/* 1024x768@60Hz (VESA)*/
			.refresh        = 60,
			.xres           = 1024,
			.yres           = 768,
			.pixclock       = 15384,
			.left_margin    = 160,
			.right_margin   = 24,
			.upper_margin   = 29,
			.lower_margin   = 3,
			.hsync_len      = 136,
			.vsync_len      = 6,
			.sync           = FB_SYNC_EXT,
			.vmode          = FB_VMODE_NONINTERLACED
		}
	}
};

size_t display_count = ARRAY_SIZE(displays);

static int setup_display(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int reg;
	int timeout = 100000;

	enable_ipu_clock();
	imx_setup_hdmi();

	/* set video pll to 455MHz (24MHz * (37+11/12) / 2) */
	setbits_le32(&ccm->analog_pll_video, BM_ANADIG_PLL_VIDEO_POWERDOWN);

	reg = readl(&ccm->analog_pll_video);
	reg &= ~BM_ANADIG_PLL_VIDEO_DIV_SELECT;
	reg |= BF_ANADIG_PLL_VIDEO_DIV_SELECT(37);
	reg &= ~BM_ANADIG_PLL_VIDEO_POST_DIV_SELECT;
	reg |= BF_ANADIG_PLL_VIDEO_POST_DIV_SELECT(1);
	writel(reg, &ccm->analog_pll_video);

	writel(BF_ANADIG_PLL_VIDEO_NUM_A(11), &ccm->analog_pll_video_num);
	writel(BF_ANADIG_PLL_VIDEO_DENOM_B(12), &ccm->analog_pll_video_denom);

	reg &= ~BM_ANADIG_PLL_VIDEO_POWERDOWN;
	writel(reg, &ccm->analog_pll_video);

	while (timeout--)
		if (readl(&ccm->analog_pll_video) & BM_ANADIG_PLL_VIDEO_LOCK)
			break;
	if (timeout < 0) {
		printf("Warning: video pll lock timeout!\n");
		return -ETIMEDOUT;
	}

	reg = readl(&ccm->analog_pll_video);
	reg |= BM_ANADIG_PLL_VIDEO_ENABLE;
	reg &= ~BM_ANADIG_PLL_VIDEO_BYPASS;
	writel(reg, &ccm->analog_pll_video);

	/* gate ipu1_di0_clk */
	clrbits_le32(&ccm->CCGR3, MXC_CCM_CCGR3_LDB_DI0_MASK);

	/* select video_pll clock / 7  for ipu1_di0_clk -> 65MHz pixclock */
	reg = readl(&ccm->chsccdr);
	reg &= ~(MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_MASK |
		 MXC_CCM_CHSCCDR_IPU1_DI0_PODF_MASK |
		 MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_MASK);
	reg |= (2 << MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_OFFSET) |
	       (6 << MXC_CCM_CHSCCDR_IPU1_DI0_PODF_OFFSET) |
	       (0 << MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &ccm->chsccdr);

	/* enable ipu1_di0_clk */
	setbits_le32(&ccm->CCGR3, MXC_CCM_CCGR3_LDB_DI0_MASK);

	return 0;
}
#endif /* CONFIG_VIDEO_IPUV3 */

/*** USB ***/
#ifdef CONFIG_USB_EHCI_MX6
static void setup_usb(void)
{
	/* There is USB hub (U22) but it cannot be controlled by the firmware.
	* It is enabled by default and can only be turned off by the USB
	* current limiter IC (U23).
	* It seems that we do not need any more initialization */
}

#endif /* CONFIG_USB_EHCI_MX6 */

int board_early_init_f(void)
{
	int ret = 0;
	setup_iomux_uart();

#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif
	return ret;
}

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

// Mandatory board_init function to put configuration code
int board_init(void)
{
	int ret = 0;

	/* address of boot parameters */
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;

#ifdef CONFIG_VIDEO_IPUV3
	ret = setup_display();
#endif

	return ret;
}

int board_late_init(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
       /* Do we have a hardware way of distinguishing between 2.x and 3.x ? */
	env_set("board_rev", "3");
#endif
	return 0;
}

#ifdef CONFIG_SPL_BUILD

#include <asm/arch/mx6-ddr.h>
#include <spl.h>

/* DCD - Device Configuration Data */
/* MMDC - Multi Mode DDR Controller */

/* Clock Control Module */
static void ccgr_init(void)
{
	/* register definition in arch/arm/include/asm/arch-mx6/imx-regs.h */
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

/* output from NXP DDR initialization spreadsheet */
static const struct mx6sdl_iomux_ddr_regs mx6dl_ddr_ioregs = {
	.dram_sdclk_0 = 0x00000028,
	.dram_sdclk_1 = 0x00000028,
	.dram_cas =	0x00000028,
	.dram_ras =	0x00000028,
	.dram_reset =	0x000c0028,
	.dram_sdcke0 =	0x00003000,
	.dram_sdcke1 =	0x00003000,
	.dram_sdba2 =	0x00000000,
	.dram_sdodt0 =	0x00000028,
	.dram_sdodt1 =	0x00000028,
	.dram_sdqs0 =	0x00000028,
	.dram_sdqs1 =	0x00000028,
	.dram_sdqs2 =	0x00000028,
	.dram_sdqs3 =	0x00000028,
	.dram_sdqs4 =	0x00000028,
	.dram_sdqs5 =	0x00000028,
	.dram_sdqs6 =	0x00000028,
	.dram_sdqs7 =	0x00000028,
	.dram_dqm0 =	0x00000028,
	.dram_dqm1 =	0x00000028,
	.dram_dqm2 =	0x00000028,
	.dram_dqm3 =	0x00000028,
	.dram_dqm4 =	0x00000028,
	.dram_dqm5 =	0x00000028,
	.dram_dqm6 =	0x00000028,
	.dram_dqm7 =	0x00000028,
};

/* output from NXP DDR initialization spreadsheet */
static const struct mx6sdl_iomux_grp_regs mx6dl_grp_ioregs = {
	.grp_ddr_type =	0x000c0000,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_ddrpke =	0x00000000,
	.grp_addds =	0x00000028,
	.grp_ctlds =	0x00000028,
	.grp_ddrmode =	0x00020000,
	.grp_b0ds =	0x00000028,
	.grp_b1ds =	0x00000028,
	.grp_b2ds =	0x00000028,
	.grp_b3ds =	0x00000028,
	.grp_b4ds =	0x00000028,
	.grp_b5ds =	0x00000028,
	.grp_b6ds =	0x00000028,
	.grp_b7ds =	0x00000028,
};

/* DDR calibration output */
static const struct mx6_mmdc_calibration mx6dl_2g_mmcd_calib = {
       .p0_mpwldectrl0	= 0x0044004c,
       .p0_mpwldectrl1	= 0x00380043,
       .p1_mpwldectrl0	= 0x00250025,
       .p1_mpwldectrl1	= 0x002b0046,
       .p0_mpdgctrl0	= 0x020b020b,
       .p0_mpdgctrl1	= 0x01750175,
       .p1_mpdgctrl0	= 0x0177017c,
       .p1_mpdgctrl1	= 0x016d0179,
       .p0_mprddlctl	= 0x4b494b4b,
       .p1_mprddlctl	= 0x4a4b4c4a,
       .p0_mpwrdlctl	= 0x3f3f3336,
       .p1_mpwrdlctl	= 0x35393530,
};

/* MT41K256M16HA-125:E DDR parameters */
static struct mx6_ddr3_cfg mem_ddr_4g = {
	.mem_speed = 1600,
	.density = 4,
	.width = 16,
	.banks = 8,
	.rowaddr = 15,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

#define CRYSTAL_DRAM_BUS_WIDTH	64

static void spl_dram_init(void)
{
	struct mx6_ddr_sysinfo sysinfo = {
		/* width of data bus: 0=16, 1=32, 2=64 */
		.dsize = CRYSTAL_DRAM_BUS_WIDTH / 32,
		/* config for full 4GB range so that get_mem_size() works */
		.cs_density = 32,	/* 32Gb per CS */
		.ncs = 1,		/* single chip select */
		.cs1_mirror = 0,
		.rtt_wr = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Wr = RZQ/4 */
		.rtt_nom = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Nom = RZQ/4 */
		.walat = 1,	/* Write additional latency */
		.ralat = 5,	/* Read additional latency */
		.mif3_mode = 3,	/* Command prediction working mode */
		.bi_on = 1,	/* Bank interleaving enabled */
		.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
		.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
		.ddr_type = DDR_TYPE_DDR3,
		.refsel = 1,	/* Refresh cycles at 32KHz */
		.refr = 7,	/* 8 refresh commands per refresh cycle */
	};

	/* for i.MX6DL SoC */
	mx6sdl_dram_iocfg(CRYSTAL_DRAM_BUS_WIDTH, &mx6dl_ddr_ioregs, &mx6dl_grp_ioregs);

	mx6_dram_cfg(&sysinfo, &mx6dl_2g_mmcd_calib, &mem_ddr_4g);
}

void board_init_f(ulong dummy)
{
	/* setup AIPS and disable watchdog */
	/* from arch/arm/mach-imx/mx6/soc.c */
	arch_cpu_init();

	ccgr_init();

	/* from arch/arm/mach-imx/mx6/soc.c */
	gpr_init();

	/* iomux */
	board_early_init_f();

	/* setup GP timer */
	/* from arch/arm/mach-imx/syscounter.c */
	timer_init();

	printf("Serial console enabled\n\r");

	/* DDR initialization */
	spl_dram_init();

	printf("DDR initialized\n\r");

	/* There is no need to clear BSS - it will be done by crt0.S */

	/* Board_init_f must return normally
	 * Don't call board_init_r() directly) */
}

void spl_board_init()
{
	/* UART clocks enabled and gd valid - init serial console */
	/* common/spl/spl.c */
	preloader_console_init();
}

void board_boot_order(u32 *spl_boot_list)
{
	switch (spl_boot_device()) {
	case BOOT_DEVICE_MMC2:
	case BOOT_DEVICE_MMC1:
		/* Pririotize eMMC boot */
		spl_boot_list[0] = BOOT_DEVICE_MMC2;
		spl_boot_list[1] = BOOT_DEVICE_MMC1;
		break;
	default:
		spl_boot_list[0] = spl_boot_device();
	}
}

#endif /* CONFIG_SPL_BUILD */
