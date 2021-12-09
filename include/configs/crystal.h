/*
 * Copyright (C) 2017 Vitro Technology Corporation
 *
 * Configuration settings for the Vitro Crystal i.MX6 platform
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CRYSTAL_BOARD_CONFIG_H__
#define __CRYSTAL_BOARD_CONFIG_H__

#include "mx6_common.h"
#include "imx6_spl.h"

/* UART */
#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONSOLE_DEV	"ttymxc0"

/* MMC */
#define CONFIG_SYS_FSL_USDHC_NUM	1
#define CONFIG_SYS_MMC_ENV_DEV		0	/* SDHC2 */
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR

/* Ethernet Configuration */
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_FEC_MXC_PHYADDR		0

/* Framebuffer */
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP

/* USB */
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2

#define CONFIG_PREBOOT \
	"if hdmidet; then " \
		"usb start; "		       \
		"setenv stdin  serial,usbkbd; "\
		"setenv stdout serial,vga; "   \
		"setenv stderr serial,vga; "   \
	"else " \
		"setenv stdin  serial; " \
		"setenv stdout serial; " \
		"setenv stderr serial; " \
	"fi;"

#define CONFIG_EXTRA_ENV_SETTINGS \
	"fdtfile=undefined\0" \
	"fdt_addr_r=0x18000000\0" \
	"fdt_addr=0x18000000\0" \
	"kernel_addr_r=" __stringify(CONFIG_LOADADDR) "\0"  \
	"pxefile_addr_r=" __stringify(CONFIG_LOADADDR) "\0" \
	"scriptaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"ramdisk_addr_r=0x13000000\0" \
	"ramdiskaddr=0x13000000\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_high=0xffffffff\0" \
	"ip_dyn=yes\0" \
	"console=" CONSOLE_DEV ",115200\0" \
	"bootm_size=0x10000000\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"finduuid=part uuid mmc 0:1 uuid\0" \
	"update_sd_firmware=" \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"if mmc dev ${mmcdev}; then "	\
			"if ${get_cmd} ${update_sd_firmware_filename}; then " \
				"setexpr fw_sz ${filesize} / 0x200; " \
				"setexpr fw_sz ${fw_sz} + 1; "	\
				"mmc write ${loadaddr} 0x2 ${fw_sz}; " \
			"fi; "	\
		"fi\0" \
	"findfdt="\
		"if test $board_name = crystal && test $board_rev = 2; then " \
			"setenv fdtfile imx6dl-crystal2.dtb; " \
		"elif test $board_name = crystal && test $board_rev = 3; then " \
			"setenv fdtfile imx6dl-crystal3.dtb; " \
		"else " \
			"echo WARNING: Could not determine dtb to use; " \
		"fi; \0" \
	BOOTENV

#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 0) \
	func(USB, usb, 0) \
	func(PXE, pxe, na) \
	func(DHCP, dhcp, na)

#include <config_distro_bootcmd.h>


/* Physical Memory Map */
#define CONFIG_SYS_MAX_FLASH_BANKS	1
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE
#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)

#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */
#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_ENV_OFFSET_REDUND CONFIG_ENV_OFFSET + CONFIG_ENV_SIZE

#endif /* __CRYSTAL_BOARD_CONFIG_H__ */
