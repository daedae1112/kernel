/*
 * linux/arch/arm/mach-omap2/board-mapphone-camera.c
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * Derived from mach-omap3/board-3430sdp.c
 *
 * Copyright (C) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-generic.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <plat/mux.h>
#include <plat/board-mapphone.h>
#include <plat/omap-pm.h>
#include <plat/control.h>
#include <linux/string.h>
#include <plat/resource.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#if defined(CONFIG_VIDEO_OMAP3)
#include <media/v4l2-int-device.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>
#include <../drivers/media/video/isp/isp.h>
#include <../drivers/media/video/isp/ispcsi2.h>
#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
#include <media/mt9p012.h>
#define MT9P012_XCLK_48MHZ		48000000
#endif
#if defined(CONFIG_VIDEO_OV8810) || defined(CONFIG_VIDEO_OV8810_MODULE)
#include <media/ov8810.h>
#if defined(CONFIG_LEDS_FLASH_RESET)
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#endif
#define OV8810_CSI2_CLOCK_POLARITY	0	/* +/- pin order */
#define OV8810_CSI2_DATA0_POLARITY	0	/* +/- pin order */
#define OV8810_CSI2_DATA1_POLARITY	0	/* +/- pin order */
#define OV8810_CSI2_CLOCK_LANE		1	 /* Clock lane position: 1 */
#define OV8810_CSI2_DATA0_LANE		2	 /* Data0 lane position: 2 */
#define OV8810_CSI2_DATA1_LANE		3	 /* Data1 lane position: 3 */
#define OV8810_CSI2_PHY_THS_TERM	1  /* GVH */
#define OV8810_CSI2_PHY_THS_SETTLE	21  /* GVH */
#define OV8810_CSI2_PHY_TCLK_TERM	0
#define OV8810_CSI2_PHY_TCLK_MISS	1
#define OV8810_CSI2_PHY_TCLK_SETTLE	14
#define OV8810_XCLK_27MHZ			27000000

#endif
#ifdef CONFIG_VIDEO_OMAP3_HPLENS
#include <../drivers/media/video/hplens.h>
#endif
#endif



#define CAM_IOMUX_SAFE_MODE (OMAP343X_PADCONF_PULL_UP | \
				OMAP343X_PADCONF_PUD_ENABLED | \
				OMAP343X_PADCONF_MUXMODE7)
#define CAM_IOMUX_SAFE_MODE_INPUT (OMAP343X_PADCONF_INPUT_ENABLED | \
				OMAP343X_PADCONF_PULL_UP | \
				OMAP343X_PADCONF_PUD_ENABLED | \
				OMAP343X_PADCONF_MUXMODE7)
#define CAM_IOMUX_FUNC_MODE (OMAP343X_PADCONF_INPUT_ENABLED | \
				OMAP343X_PADCONF_MUXMODE0)

#define CAM_MAX_REGS 5
#define CAM_MAX_REG_NAME_LEN 8

#define MAPPHONE_CPU_CLK_LOCK    1
#define MAPPHONE_CPU_CLK_UNLOCK  0

static void mapphone_camera_lines_safe_mode(void);
static void mapphone_camera_lines_func_mode(void);
/* devtree regulator support */
static void mapphone_lock_dev_cpufreq(struct device *, int);

static char regulator_list[CAM_MAX_REGS][CAM_MAX_REG_NAME_LEN];
/* devtree flash */
static u8 bd7885_available;
static enum v4l2_power previous_power = V4L2_POWER_OFF;

#ifdef CONFIG_VIDEO_OMAP3_HPLENS
static int hplens_power_set(enum v4l2_power power)
{
	(void)power;

	return 0;
}

static int hplens_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_LENS;

	return 0;
}

struct hplens_platform_data mapphone_hplens_platform_data = {
	.power_set = hplens_power_set,
	.priv_data_set = hplens_set_prv_data,
};
#endif

#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
static struct omap34xxcam_sensor_config mt9p012_cam_hwc = {
	.sensor_isp = 0,
	.xclk = OMAP34XXCAM_XCLK_A,
	.capture_mem = PAGE_ALIGN(2592 * 1944 * 2) * 4,
};

static int mt9p012_sensor_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.xclk = mt9p012_cam_hwc.xclk;
	hwc->u.sensor.sensor_isp = mt9p012_cam_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = mt9p012_cam_hwc.capture_mem;
	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	return 0;
}

static struct isp_interface_config mt9p012_if_config = {
	.ccdc_par_ser = ISP_PARLL,
	.dataline_shift = 0x1,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.wenlog = ISPCCDC_CFG_WENLOG_OR,
	.wait_bayer_frame = 0,
	.wait_yuv_frame = 1,
	.dcsub = 42,
	.cam_mclk = 432000000,
	.cam_mclk_src_div = OMAP_MCAM_SRC_DIV,
	.raw_fmt_in = ISPCCDC_INPUT_FMT_GR_BG,
	.u.par.par_bridge = 0x0,
	.u.par.par_clk_pol = 0x0,
};

u32 mt9p012_set_xclk(u32 xclkfreq)
{
	return isp_set_xclk(xclkfreq, OMAP34XXCAM_XCLK_A);
}

static int mt9p012_sensor_power_set(struct device* dev, enum v4l2_power power)
{
	static struct regulator *regulator;
	int error = 0;

	switch (power) {
	case V4L2_POWER_OFF:
		mapphone_lock_dev_cpufreq(dev, MAPPHONE_CPU_CLK_UNLOCK);


//##Dongseok : MI need to set gpio_direction_output(GPIO_MT9P012_RESET, 0); 
		gpio_direction_output(GPIO_MT9P012_RESET, 0);
		mt9p012_set_xclk(0);		
		gpio_free(GPIO_MT9P012_RESET);
		msleep(1);
		/* Turn off power */
		if (regulator != NULL) {
			regulator_disable(regulator);
			regulator_put(regulator);
			regulator = NULL;
		} else {
			mapphone_camera_lines_safe_mode();
			pr_err("%s: Regulator for vcam is not "\
					"initialized\n", __func__);
			return -EIO;
		}

		/* Release pm constraints */
		omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 0);
		omap_pm_set_max_mpu_wakeup_lat(dev, -1);
		mapphone_camera_lines_safe_mode();
	break;
	case V4L2_POWER_ON:
		if (previous_power == V4L2_POWER_OFF) {
			/* Power Up Sequence */
			mapphone_camera_lines_func_mode();
			/* Set min throughput to:
			 *  2592 x 1944 x 2bpp x 30fps x 3 L3 accesses */
			omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 885735);
			/* Hold a constraint to keep MPU in C1 */
			omap_pm_set_max_mpu_wakeup_lat(dev, MPU_LATENCY_C1);

			/* Configure ISP */
			isp_configure_interface(&mt9p012_if_config);

//TTU & MI Froyo need to gipo_request(GPIO_MT9P012_RESET,XXXX)
			/* Request and configure gpio pins */
			if (gpio_request(GPIO_MT9P012_RESET,
						"mt9p012 camera reset") != 0) {
				error = -EIO;
				goto out;
			}

			/* set to output mode */
			gpio_direction_output(GPIO_MT9P012_RESET, 0);

			
			/* nRESET is active LOW. set HIGH to release reset */
			gpio_set_value(GPIO_MT9P012_RESET, 1);			
			

			/* turn on digital power */
			if (regulator != NULL) {
				pr_warning("%s: Already have "\
						"regulator\n", __func__);
			} else {
				regulator = regulator_get(NULL, "vcam");
				if (IS_ERR(regulator)) {
					pr_err("%s: Cannot get vcam "\
						"regulator, err=%ld\n",
						__func__, PTR_ERR(regulator));
					error = PTR_ERR(regulator);
					goto out;
				}
			}

			if (regulator_enable(regulator) != 0) {
				pr_err("%s: Cannot enable vcam regulator\n",
						__func__);
				error = -EIO;
				goto out;
			}
			msleep(5);
		}

		mt9p012_set_xclk(MT9P012_XCLK_48MHZ);
		msleep(3);

		if (previous_power == V4L2_POWER_OFF) {
//##Dongseok : MI need to set gpio_direction_output(GPIO_MT9P012_RESET, 0); 
			gpio_direction_output(GPIO_MT9P012_RESET, 0); //Adding dongseok
			udelay(1500);
			/* RESET is active LOW. set HIGH to release reset */
			gpio_set_value(GPIO_MT9P012_RESET, 1);

			/* give sensor sometime to get out of the reset.
			 * Datasheet says 2400 xclks.
			 */
			msleep(3);
		}
		mapphone_lock_dev_cpufreq(dev, MAPPHONE_CPU_CLK_LOCK);
		break;
out:
		mt9p012_set_xclk(0);
		omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 0);
		omap_pm_set_max_mpu_wakeup_lat(dev, -1);
		mapphone_camera_lines_safe_mode();
		return error;
	case V4L2_POWER_STANDBY:
		mapphone_lock_dev_cpufreq(dev, MAPPHONE_CPU_CLK_UNLOCK);
		/* Stand By Sequence */
		mt9p012_set_xclk(0);
		break;
	}
	/* Save powerstate to know what was before calling POWER_ON. */
	previous_power = power;
	return 0;
}

struct mt9p012_platform_data mapphone_mt9p012_platform_data = {
	.power_set = mt9p012_sensor_power_set,
	.priv_data_set = mt9p012_sensor_set_prv_data,
	.csi2_lane_count = isp_csi2_complexio_lanes_count,
	.csi2_cfg_vp_out_ctrl = isp_csi2_ctrl_config_vp_out_ctrl,
	.csi2_ctrl_update = isp_csi2_ctrl_update,
	.csi2_cfg_virtual_id = isp_csi2_ctx_config_virtual_id,
	.csi2_ctx_update = isp_csi2_ctx_update,
	.csi2_calc_phy_cfg0  = isp_csi2_calc_phy_cfg0,
};

#endif /* #ifdef CONFIG_VIDEO_MT9P012 || CONFIG_VIDEO_MT9P012_MODULE */


static void mapphone_lock_dev_cpufreq(struct device *dev, int lock)
{
	static int flag;

	if (MAPPHONE_CPU_CLK_LOCK == lock) {
		resource_request("vdd1_opp", dev, VDD1_OPP5);
		flag = 1;
	} else {
		if (flag == 1) {
			resource_release("vdd1_opp", dev);
			flag = 0;
		}
	}
}
/* We can't change the IOMUX config after bootup
 * with the current pad configuration architecture,
 * the next two functions are hack to configure the
 * camera pads at runtime to save power in standby.
 * For phones don't have MIPI camera support, like
 * Ruth, Tablet P2,P3 */

void mapphone_camera_lines_safe_mode(void)
{
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE_INPUT, 0x011a);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE_INPUT, 0x011c);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE_INPUT, 0x011e);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE_INPUT, 0x0120);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0122);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0124);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0126);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0128);
}

void mapphone_camera_lines_func_mode(void)
{
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x011a);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x011c);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x011e);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0120);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0122);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0124);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0126);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0128);
}




void __init mapphone_camera_init(void)
{
    printk(KERN_INFO "mapphone_camera_init: smart camera\n");
	omap_cfg_reg(A24_34XX_CAM_HS);
    omap_cfg_reg(A23_34XX_CAM_VS);
    omap_cfg_reg(C27_34XX_CAM_PCLK);
    omap_cfg_reg(B24_34XX_CAM_D2);
    omap_cfg_reg(C24_34XX_CAM_D3);
    omap_cfg_reg(D24_34XX_CAM_D4);
    omap_cfg_reg(A25_34XX_CAM_D5);
    omap_cfg_reg(K28_34XX_CAM_D6);
    omap_cfg_reg(L28_34XX_CAM_D7);
    omap_cfg_reg(K27_34XX_CAM_D8);
    omap_cfg_reg(L27_34XX_CAM_D9);
	omap_cfg_reg(C25_34XX_CAM_XCLKA);
    omap_cfg_reg(K8_34XX_GPMC_WAIT2);
    omap_cfg_reg(C23_34XX_CAM_FLD);
}
