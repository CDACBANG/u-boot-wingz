/*
 * (C) Copyright 2004-2011
 * Texas Instruments, <www.ti.com>
 *
 * Author :
 *	rangineni balasubramanyam rangineni.balu@cdac.in
 *	thajudheenk <thajudheenk@cdac.in>
 *
 * Derived from Beagle Board and 3430 SDP code by
 *	Richard Woodruff <r-woodruff2@ti.com>
 *	Syed Mohammed Khasim <khasim@ti.com>
 *
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <dm.h>
#include <ns16550.h>
#ifdef CONFIG_STATUS_LED
#include <status_led.h>
#endif
#include <twl4030.h>
#include <linux/mtd/nand.h>
#include <asm/io.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/mux.h>
#include <asm/arch/mem.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>
#include <asm/omap_musb.h>
#include <asm/errno.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/musb.h>
#include "wingz.h"
#include <command.h>

#ifdef CONFIG_USB_EHCI
#include <usb.h>
#include <asm/ehci-omap.h>
#endif

#define TWL4030_I2C_BUS			0
#define EXPANSION_EEPROM_I2C_BUS	1
#define EXPANSION_EEPROM_I2C_ADDRESS	0x50


DECLARE_GLOBAL_DATA_PTR;

static struct {
	unsigned int device_vendor;
	unsigned char revision;
	unsigned char content;
	char fab_revision[8];
	char env_var[16];
	char env_setting[64];
} expansion_config;

static const struct ns16550_platdata wingz_serial = {
	OMAP34XX_UART3,
	2,
	V_NS16550_CLK
};

U_BOOT_DEVICE(beagle_uart) = {
	"serial_omap",
	&wingz_serial
};

/*
 * Routine: board_init
 * Description: Early hardware init.
 */
int board_init(void)
{
	gpmc_init(); /* in SRAM or SDRAM, finish GPMC */
	/* board id for Linux */
	gd->bd->bi_arch_number = MACH_TYPE_WINGZ_CDAC_BLR;
	/* boot param addr */
	gd->bd->bi_boot_params = (OMAP34XX_SDRC_CS0 + 0x100);

#if defined(CONFIG_STATUS_LED) && defined(STATUS_LED_BOOT)
	status_led_set (STATUS_LED_BOOT, STATUS_LED_ON);
#endif

	return 0;
}


static int get_board_revision(void)
{
	static int revision = -1;

	return revision;
}

#ifdef CONFIG_SPL_BUILD
/*
 * Routine: get_board_mem_timings
 * Description: If we use SPL then there is no x-loader nor config header
 * so we have to setup the DDR timings ourself on both banks.
 */
void get_board_mem_timings(struct board_sdrc_timings *timings)
{
	int pop_mfr, pop_id;

	/*
	 * We need to identify what PoP memory is on the board so that
	 * we know what timings to use.  If we can't identify it then
	 * we know it's an xM.  To map the ID values please see nand_ids.c
	 */
	identify_nand_chip(&pop_mfr, &pop_id);

	timings->mr = MICRON_V_MR_165;
	if (pop_mfr == NAND_MFR_STMICRO && pop_id == 0xba) {
			/* 512MB DDR */
			timings->mcfg = NUMONYX_V_MCFG_165(512 << 20);
			timings->ctrla = NUMONYX_V_ACTIMA_165;
			timings->ctrlb = NUMONYX_V_ACTIMB_165;
			timings->rfr_ctrl = SDP_3430_SDRC_RFR_CTRL_165MHz;			
		} else if (pop_mfr == NAND_MFR_MICRON && pop_id == 0xba) {
			timings->mcfg = MICRON_V_MCFG_165(128 << 20);
			timings->ctrla = MICRON_V_ACTIMA_165;
			timings->ctrlb = MICRON_V_ACTIMB_165;
			timings->rfr_ctrl = SDP_3430_SDRC_RFR_CTRL_165MHz;			
		} else if (pop_mfr == NAND_MFR_MICRON && pop_id == 0xbc) {
			timings->mcfg = MICRON_V_MCFG_200(256 << 20);
			timings->ctrla = MICRON_V_ACTIMA_200;
			timings->ctrlb = MICRON_V_ACTIMB_200;
			timings->rfr_ctrl = SDP_3430_SDRC_RFR_CTRL_200MHz;			
		}
	
}
#endif


#ifdef CONFIG_VIDEO_OMAP3


#endif

#ifdef CONFIG_USB_MUSB_OMAP2PLUS
static struct musb_hdrc_config musb_config = {
	.multipoint     = 1,
	.dyn_fifo       = 1,
	.num_eps        = 16,
	.ram_bits       = 12,
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
};

static struct musb_hdrc_platform_data musb_plat = {
#if defined(CONFIG_MUSB_HOST)
	.mode           = MUSB_HOST,
#elif defined(CONFIG_MUSB_GADGET)
	.mode		= MUSB_PERIPHERAL,
#else
#error "Please define either CONFIG_MUSB_HOST or CONFIG_MUSB_GADGET"
#endif
	.config         = &musb_config,
	.power          = 100,
	.platform_ops	= &omap2430_ops,
	.board_data	= &musb_board_data,
};
#endif

/*
 * Routine: misc_init_r
 * Description: Configure board specific parts
 */
int misc_init_r(void)
{
	struct gpio *gpio5_base = (struct gpio *)OMAP34XX_GPIO5_BASE;
	struct gpio *gpio6_base = (struct gpio *)OMAP34XX_GPIO6_BASE;
	struct control_prog_io *prog_io_base = (struct control_prog_io *)OMAP34XX_CTRL_BASE;
	bool generate_fake_mac = false;
	u32 value;

	/* Enable i2c2 pullup resisters */
	value = readl(&prog_io_base->io1);
	value &= ~(PRG_I2C2_PULLUPRESX);
	writel(value, &prog_io_base->io1);

	printf("WINGZ energy \nIoT group \n CDAC Bangalore\n");
	setenv("musb", "musb_hdrc.fifo_mode=5");
	MUX_WINGZ_C();
	/* Set VAUX2 to 1.8V for EHCI PHY */
	twl4030_pmrecv_vsel_cfg(TWL4030_PM_RECEIVER_VAUX2_DEDICATED,
					TWL4030_PM_RECEIVER_VAUX2_VSEL_18,
					TWL4030_PM_RECEIVER_VAUX2_DEV_GRP,
					TWL4030_PM_RECEIVER_DEV_GRP_P1);
	

	
	if (expansion_config.content == 1)
		setenv(expansion_config.env_var, expansion_config.env_setting);

	twl4030_power_init();
	twl4030_led_init(TWL4030_LED_LEDEN_LEDAON | TWL4030_LED_LEDEN_LEDBON);
	

	/* Set GPIO states before they are made outputs */
	writel(GPIO23 | GPIO10 | GPIO8 | GPIO2 | GPIO1,
		&gpio6_base->setdataout);
	writel(GPIO31 | GPIO30 | GPIO29 | GPIO28 | GPIO22 | GPIO21 |
		GPIO15 | GPIO14 | GPIO13 | GPIO12, &gpio5_base->setdataout);

	/* Configure GPIOs to output */
	writel(~(GPIO23 | GPIO10 | GPIO8 | GPIO2 | GPIO1), &gpio6_base->oe);
	writel(~(GPIO31 | GPIO30 | GPIO29 | GPIO28 | GPIO22 | GPIO21 |
		GPIO15 | GPIO14 | GPIO13 | GPIO12), &gpio5_base->oe);

	dieid_num_r();

#ifdef CONFIG_VIDEO_OMAP3
	gpio_request(170, "dvi");
	gpio_direction_output(170, 0);
	gpio_set_value(170, 1);

	//beagle_display_init();
	omap3_dss_venc_config(&venc_config_std_tv, VENC_HEIGHT, VENC_WIDTH);
	omap3_dss_panel_config(&dvid_cfg);


	omap3_dss_enable();
#endif

#ifdef CONFIG_USB_MUSB_OMAP2PLUS
	musb_register(&musb_plat, &musb_board_data, (void *)MUSB_BASE);
#endif

	if (generate_fake_mac) {
		u32 id[4];

		get_dieid(id);
		usb_fake_mac_from_die_id(id);
	}

	return 0;
}

/*
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers specific to the
 *		hardware. Many pins need to be moved from protect to primary
 *		mode.
 */
void set_muxconf_regs(void)
{
	MUX_WINGZ();
}

#if defined(CONFIG_GENERIC_MMC) && !defined(CONFIG_SPL_BUILD)
int board_mmc_init(bd_t *bis)
{
	return omap_mmc_init(0, 0, 0, -1, -1);
}
#endif

#if defined(CONFIG_USB_EHCI) && !defined(CONFIG_SPL_BUILD)
/* Call usb_stop() before starting the kernel */
void show_boot_progress(int val)
{
	if (val == BOOTSTAGE_ID_RUN_OS)
		usb_stop();
}

static struct omap_usbhs_board_data usbhs_bdata = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED
};

int ehci_hcd_init(int index, enum usb_init_type init,
		struct ehci_hccr **hccr, struct ehci_hcor **hcor)
{
	return omap_ehci_hcd_init(index, &usbhs_bdata, hccr, hcor);
}

int ehci_hcd_stop(int index)
{
	return omap_ehci_hcd_stop();
}

#endif /* CONFIG_USB_EHCI */

#if defined(CONFIG_USB_ETHER) && defined(CONFIG_MUSB_GADGET)
int board_eth_init(bd_t *bis)
{
	return usb_eth_initialize(bis);
}
#endif
