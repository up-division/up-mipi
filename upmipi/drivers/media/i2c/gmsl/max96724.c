// SPDX-License-Identifier: GPL-2.0
#include <linux/bitops.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/limits.h>

#include "max_des.h"
#include "mipi-upboard.h"

#define MAX96724_MULTI_LINKS 4

#define DES_TEST_DPLL_REG        0x19
#define DES_TEST_LINK_FREQ_HZ    DES_TEST_DPLL_REG * 50000000ULL
#define DES_TEST_RATE_NAME       "1.6Gbps/lane, link_freq=800MHz"

#define XTREMEI14_PHY_MAP 0x4b
#define MAX96724_UP_GPIO_BASE 512


#define DESKEW_INIT 0x87
#define DESKEW_PRO  0x00


struct max96724_chip_info {
	unsigned int num_phys;
	unsigned int num_pipes;
	unsigned int num_links;
	unsigned int num_remaps_per_pipe;
};

struct max96724_priv {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct max_des des;
	const struct max96724_chip_info *info;
	bool des_hw_inited;
};


static inline struct max96724_priv *des_to_priv(struct max_des *des)
{
	return container_of(des, struct max96724_priv, des);
}

static int max96724_upboard_pwren_gpio(struct i2c_client *client, bool enable)
{
	struct device *dev = &client->dev;
	struct upmipi_gpios *gpio_info;
	unsigned int idx;
	int gpio;
	int ret;

	gpio_info = mipi_upboard_gpios();
	if (!gpio_info) {
		dev_info(dev, "[MAX96724-PWR] no UP MIPI GPIO table, skip\n");
		return 0;
	}

	dev_info(dev, "[MAX96724-PWR] i2c adaptor nr:%d\n", client->adapter->nr);

	/*
	 * Follow UP MIPI convention:
	 *   adapter 0/1/2 -> CAM1 reset / power enable path
	 *   adapter 3/4/5 -> CAM2 reset / power enable path
	 *
	 * For UPX-MTL01 / UPX-ARL01:
	 *   CAM1_RST offset = 397
	 *   CAM2_RST offset = 113
	 * Legacy GPIO number = gpiochip0 base 512 + offset.
	 */
	switch (client->adapter->nr) {
	case 0:
	case 1:
	case 2:
		idx = 0; /* UPBOARD_CAM1_RESET */
		break;
	case 3:
	case 4:
	case 5:
		idx = 2; /* UPBOARD_CAM2_RESET */
		break;
	default:
		dev_info(dev,
			 "[MAX96724-PWR] adapter %d does not map to CAM1/CAM2 GPIO, skip\n",
			 client->adapter->nr);
		return 0;
	}

	if (idx >= gpio_info->ngpio) {
		dev_warn(dev, "[MAX96724-PWR] GPIO index %u out of range, ngpio=%d\n",
			 idx, gpio_info->ngpio);
		return 0;
	}

	gpio = gpio_info->gpios[idx].offset + MAX96724_UP_GPIO_BASE;

	ret = gpio_request(gpio, gpio_info->gpios[idx].name);
	if (ret && ret != -EBUSY) {
		dev_warn(dev, "[MAX96724-PWR] gpio_request %s GPIO%d failed: %d\n",
			 gpio_info->gpios[idx].name, gpio, ret);
		return ret;
	}

	ret = gpio_direction_output(gpio, enable ? 1 : 0);
	if (ret) {
		dev_warn(dev, "[MAX96724-PWR] set %s GPIO%d %s failed: %d\n",
			 gpio_info->gpios[idx].name, gpio,
			 enable ? "HIGH" : "LOW", ret);
		return ret;
	}

	dev_info(dev, "[MAX96724-PWR] %s GPIO%d offset=%d set %s\n",
		 gpio_info->gpios[idx].name, gpio, gpio_info->gpios[idx].offset,
		 enable ? "HIGH" : "LOW");

	if (enable)
		msleep(10);

	return 0;
}

static const struct regmap_config max96724_i2c_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x1fff,
};

static const struct max96724_chip_info max96724_info = {
	.num_phys = 4,
	.num_pipes = 4,
	.num_links = 4,
	.num_remaps_per_pipe = 16,
};

static int max96724_write(struct max96724_priv *priv, u16 reg, u8 val)
{
	int ret = regmap_write(priv->regmap, reg, val);

	if (ret)
		dev_err(priv->dev, "write reg 0x%04x = 0x%02x failed: %d\n",
			reg, val, ret);
	return ret;
}

static __maybe_unused int max96724_update_bits(struct max96724_priv *priv, u16 reg, u8 mask, u8 val)
{
	int ret = regmap_update_bits(priv->regmap, reg, mask, val);

	if (ret)
		dev_err(priv->dev,
			"update reg 0x%04x mask 0x%02x val 0x%02x failed: %d\n",
			reg, mask, val, ret);
	return ret;
}

static int max96724_reset(struct max96724_priv *priv)
{
	usleep_range(1000, 2000);
	return 0;
}

/********************************** LINK SETTING  ***********************************/
static int max96724_link_enable(struct max96724_priv *priv, u16 val)
{
	int ret = 0;
	ret = max96724_write(priv, 0x0006, val);
	if (ret)
		return ret;

	return 0;
}

static int max96724_set_link_rate_field(struct max96724_priv *priv,
					 unsigned int link_id,
					 enum cam_gmsl2_rx_rate rate)
{
	u16 reg;
	u8 shift;
	u8 mask;
	u8 val;

	if (link_id >= MAX96724_MULTI_LINKS)
		return -EINVAL;

	if (rate != CAM_GMSL2_RX_RATE_3GBPS &&
	    rate != CAM_GMSL2_RX_RATE_6GBPS)
		return -EINVAL;

	/*
	 * 0x0010:
	 *   Link A RX rate [1:0], Link B RX rate [5:4]
	 * 0x0011:
	 *   Link C RX rate [1:0], Link D RX rate [5:4]
	 */
	reg = (link_id < 2) ? 0x0010 : 0x0011;
	shift = (link_id & 0x1) ? 4 : 0;
	mask = 0x3 << shift;
	val = ((u8)rate & 0x3) << shift;

	return max96724_update_bits(priv, reg, mask, val);
}

static int max96724_des_set_link_rate(struct max_des *des,
				      unsigned int link_id,
				      enum cam_gmsl2_rx_rate rate)
{
	struct max96724_priv *priv = des_to_priv(des);
	int ret;

	ret = max96724_set_link_rate_field(priv, link_id, rate);
	if (ret)
		return ret;

	/* Apply the new receive rate to this link. */
	ret = max96724_write(priv, 0x0018, BIT(link_id));
	if (ret)
		return ret;

	msleep(20);

	dev_info(priv->dev,
		 "[DES-SCAN] link%u RX rate set to %uGbps\n",
		 link_id,
		 rate == CAM_GMSL2_RX_RATE_6GBPS ? 6 : 3);

	return 0;
}

static int max96724_link_rate(struct max96724_priv *priv)
{
	unsigned int link;
	int ret;

	/*
	 * Temporary bring-up default.  max_des_scan_func() will subsequently
	 * test 3Gbps and 6Gbps and apply each detected camera profile's rate.
	 */
	for (link = 0; link < MAX96724_MULTI_LINKS; link++) {
		ret = max96724_set_link_rate_field(priv, link,
						  CAM_GMSL2_RX_RATE_6GBPS);
		if (ret)
			return ret;
	}

	return 0;
}

static int max96724_link_reset(struct max96724_priv *priv)
{
	int ret = 0;

	/* one-shot reset link0-3 */
	ret = max96724_write(priv, 0x0018, 0x0f);
	if (ret)
		return ret;

	return 0;
}

static int max96724_link_init(struct max96724_priv *priv)
{
	int ret = 0;

	/* Disable link && Setting all link to "GMSL2"*/
	ret = max96724_link_enable(priv, 0xf0);
	if(ret) 
		return ret;

	ret = max96724_link_rate(priv);
	if(ret) 
		return ret;

	ret = max96724_link_reset(priv);
	if(ret) 
		return ret;
	
	/* Enable all link && Setting all link to "GMSL2"*/
	ret = max96724_link_enable(priv, 0xff);
	if(ret) 
		return ret;

	return 0;
}

/********************************** PIPE SETTING  ***********************************/
static int max96724_pipe_enable(struct max96724_priv *priv, u16 val)
{
	int ret = 0;
	ret = max96724_write(priv, 0x00F4, val);
	if (ret)
		return ret;

	return 0;
}

static int max96724_pipe_select_id(struct max96724_priv *priv)
{
	int ret = 0;

	/*
	*	link0 -> pipe0 -> stream id = 2
	* 	link1 -> pipe1 -> stream id = 2
	*	link2 -> pipe2 -> stream id = 2
	*	link3 -> pipe3 -> stream id = 2
	*/

	/* [7:6]select link for pipe1, [5:4]select stream id for pipe1 */
	/* [3:2]select link for pipe0, [1:0]select stream id for pipe0 */
	ret = max96724_write(priv, 0x00F0, 0x62);
	if (ret)
		return ret;

	ret = max96724_write(priv, 0x00F1, 0xEA);
	if (ret)
		return ret;

	return 0;
}

static int max96724_pipe_lim_heart(struct max96724_priv *priv)
{
	int ret = 0;
	u16 base = 0x0106;

	for (int pipe = 0; pipe < 4; pipe++)
	{
		ret = max96724_write(priv, base + (pipe * 0x12), 0x0a);
		if (ret)
			return ret;
	}
	
	return 0;
}

static int max96724_config_pipe_remap_to_vc(struct max96724_priv *priv, unsigned int pipe_idx, unsigned int to_vc)
{
	u16 base = 0x0900 + pipe_idx * 0x40;
	u8 dst_yuv = ((to_vc & 0x3) << 6) | 0x1E;
	u8 dst_fs  = ((to_vc & 0x3) << 6) | 0x00;
	u8 dst_fe  = ((to_vc & 0x3) << 6) | 0x01;
	int ret;

	if (pipe_idx >= 4 || to_vc >= 4)
		return -EINVAL;

	ret = max96724_write(priv, base + 0x0B, 0x07);
	if (ret)
		return ret;

	ret = max96724_write(priv, base + 0x0C, 0x00);
	if (ret)
		return ret;

	/* map0/map1/map2 -> Controller1 */
	ret = max96724_write(priv, base + 0x2D, 0x15);
	if (ret)
		return ret;

	/* map0: YUV422 */
	ret = max96724_write(priv, base + 0x0D, 0x1E);
	if (ret)
		return ret;

	ret = max96724_write(priv, base + 0x0E, dst_yuv);
	if (ret)
		return ret;

	/* map1: FS */
	ret = max96724_write(priv, base + 0x0F, 0x00);
	if (ret)
		return ret;

	ret = max96724_write(priv, base + 0x10, dst_fs);
	if (ret)
		return ret;

	/* map2: FE */
	ret = max96724_write(priv, base + 0x11, 0x01);
	if (ret)
		return ret;

	ret = max96724_write(priv, base + 0x12, dst_fe);
	if (ret)
		return ret;

	dev_info(priv->dev,
		 "[DES] pipe%u remap initialized: incoming VC0 YUV/FS/FE -> VC%u, Controller1\n",
		 pipe_idx, to_vc);

	return 0;
}

static int max96724_init_all_pipe_maps(struct max96724_priv *priv)
{
	unsigned int pipe;
	int ret;

	for (pipe = 0; pipe < 4; pipe++) {
		ret = max96724_config_pipe_remap_to_vc(priv, pipe, pipe);
		if (ret)
			return ret;
	}

	dev_info(priv->dev,
		 "[DES] all pipe maps initialized: pipe0->VC0 pipe1->VC1 pipe2->VC2 pipe3->VC3\n");

	return 0;
}

static int max96724_pipe_controller_mapping(struct max96724_priv *priv)
{
	int ret = 0;

	/* pipe0-3 mapping to controller1*/
	ret = max96724_write(priv, 0x08CA, 0x55);
	if (ret)
		return ret;

	return 0;
}

static int max96724_pipe_init(struct max96724_priv *priv)
{
	int ret = 0;

	/* disable all pipe */
	ret = max96724_pipe_enable(priv, 0x00);
	if (ret)
		return ret;

	/* pipe source & stream id setting */
	ret = max96724_pipe_select_id(priv);
	if (ret)
		return ret;

	/* pipe efficiency / LIM_HEART  */
	ret = max96724_pipe_lim_heart(priv);
	if (ret)
		return ret;

	/* pipe vc/dt mapping */
	ret = max96724_init_all_pipe_maps(priv);
	if (ret)
		return ret;

	/* pipe-controller mapping */
	ret = max96724_pipe_controller_mapping(priv);
	if (ret)
		return ret;
	
	msleep(300);

	ret = max96724_pipe_enable(priv, 0x0F);
	if (ret)
		return ret;

	return 0;
}

/********************************** PHY SETTING  ***********************************/
static int max96724_phy_enable(struct max96724_priv *priv, u16 val)
{
	int ret = 0;

	ret = max96724_write(priv, 0x08A2, val);
	if (ret)
		return ret;

	return 0;
}

static int max96724_lane_mode(struct max96724_priv *priv, u16 mode)
{
	int ret = 0;

	ret = max96724_write(priv, 0x08A0, mode);
	if (ret)
		return ret;

	return 0;
}

static int max96724_lane_mapping(struct max96724_priv *priv, u16 mapping)
{
	int ret = 0;

	ret = max96724_write(priv, 0x08A3, mapping);
	if (ret)
		return ret;

	ret = max96724_write(priv, 0x08A4, mapping);
	if (ret)
		return ret;

	return 0;
}

static int max96724_lane_count(struct max96724_priv *priv, u16 count)
{
	int ret = 0;

	ret = max96724_write(priv, 0x090A, count);
	if (ret)
		return ret;

	ret = max96724_write(priv, 0x094A, count);
	if (ret)
		return ret;

	ret = max96724_write(priv, 0x098A, count);
	if (ret)
		return ret;

	ret = max96724_write(priv, 0x09CA, count);
	if (ret)
		return ret;

	return 0;
}

static int max96724_lane_deskew(struct max96724_priv *priv)
{
	int ret = 0;

	ret = max96724_write(priv, 0x0903, DESKEW_INIT);
	if (ret)
		return ret;
	ret = max96724_write(priv, 0x0904, DESKEW_PRO);
	if (ret)
		return ret;
	
	ret = max96724_write(priv, 0x0943, DESKEW_INIT);
	if (ret)
		return ret;
	ret = max96724_write(priv, 0x0944, DESKEW_PRO);
	if (ret)
		return ret;

	ret = max96724_write(priv, 0x0983, 0x00);
	if (ret)
		return ret;
	ret = max96724_write(priv, 0x0984, 0x00);
	if (ret)
		return ret;
	
	ret = max96724_write(priv, 0x09C3, 0x00);
	if (ret)
		return ret;
	ret = max96724_write(priv, 0x09C4, 0x00);
	if (ret)
		return ret;

	return 0;
}

static int max96724_lane_dpll(struct max96724_priv *priv)
{
	int ret = 0;
	ret = max96724_write(priv, 0x0415, DES_TEST_DPLL_REG);
	if (ret)
		return ret;

	ret = max96724_write(priv, 0x0418, DES_TEST_DPLL_REG);
	if (ret)
		return ret;

	ret = max96724_write(priv, 0x041B, DES_TEST_DPLL_REG);
	if (ret)
		return ret;

	ret = max96724_write(priv, 0x041E, DES_TEST_DPLL_REG);
	if (ret)
		return ret;

	return 0;
}

static int max96724_phy_init(struct max96724_priv *priv)
{
	int ret = 0;

	/* phy disable */
    ret = max96724_phy_enable(priv, 0x00);
	if (ret)
		return ret;

	/* lane mode 2*4 */
	ret = max96724_lane_mode(priv, 0x04);
	if (ret)
		return ret;

	/* lane mapping */
	ret = max96724_lane_mapping(priv, XTREMEI14_PHY_MAP);
	if (ret)
		return ret;

	/* lane count */
	ret = max96724_lane_count(priv, 0xC0);
	if (ret)
		return ret;

	/* phy deskew */
	ret = max96724_lane_deskew(priv);
	if (ret)
		return ret;

	/* lane dpll */
	ret = max96724_lane_dpll(priv);
	if (ret)
		return ret;

	/* phy enable */
	ret = max96724_phy_enable(priv, 0x30);
	if (ret)
		return ret;

	return 0;
}

static void max96724_dump_reg_group(struct max96724_priv *priv,
				    const char *tag,
				    const char *group,
				    const u16 *regs,
				    unsigned int nregs)
{
	unsigned int val;
	unsigned int i;

	dev_info(priv->dev,
		 "[DUMP][%s] ---------- %s ----------\n",
		 tag, group);

	for (i = 0; i < nregs; i++) {
		if (!regmap_read(priv->regmap, regs[i], &val))
			dev_info(priv->dev,
				 "[DUMP][%s] reg 0x%04x = 0x%02x\n",
				 tag, regs[i], val & 0xff);
		else
			dev_warn(priv->dev,
				 "[DUMP][%s] reg 0x%04x read failed\n",
				 tag, regs[i]);
	}
}

static void max96724_dump_regs(struct max96724_priv *priv, const char *tag)
{
	static const u16 link_regs[] = {
		0x0006, /* Link enable / GMSL1/2 mode */
		0x0010, /* Link A/B rate */
		0x0011, /* Link C/D rate */
		0x0018, /* Link reset */
		0x001A, /* Link A lock */
		0x000A, /* Link B lock */
		0x000B, /* Link C lock */
		0x000C, /* Link D lock */
	};

	static const u16 pipe_select_regs[] = {
		0x00F0, /* Pipe0/1 link + stream select */
		0x00F1, /* Pipe2/3 link + stream select */
		0x00F4, /* Pipe enable */
	};

	static const u16 video_lock_regs[] = {
		0x01DC, /* Pipe0 VIDEO_LOCK */
		0x01FC, /* Pipe1 VIDEO_LOCK */
		0x021C, /* Pipe2 VIDEO_LOCK */
		0x023C, /* Pipe3 VIDEO_LOCK */
	};

	static const u16 hvd_detect_regs[] = {
		0x11F0, /* DE_DET pipe0~3 */
		0x11F1, /* HS_DET pipe0~3 */
		0x11F2, /* VS_DET pipe0~3 */
		0x11F3, /* HS_POL pipe0~3 */
		0x11F4, /* VS_POL pipe0~3 */
	};

	static const u16 hvd_counter_ctrl_regs[] = {
		0x11F9, /* HVD counter control */
		0x11FA, /* HVD one-shot control */
	};

	static const u16 pipe0_hvd_counter_regs[] = {
		0x1207, /* Pipe0 VS count */
		0x1208, /* Pipe0 HS count MSB */
		0x1209, /* Pipe0 HS count LSB */
		0x120A, /* Pipe0 DE count MSB */
		0x120B, /* Pipe0 DE count LSB */
	};

	static const u16 pipe1_hvd_counter_regs[] = {
		0x1217, /* Pipe1 VS count */
		0x1218, /* Pipe1 HS count MSB */
		0x1219, /* Pipe1 HS count LSB */
		0x121A, /* Pipe1 DE count MSB */
		0x121B, /* Pipe1 DE count LSB */
	};

	static const u16 pipe2_hvd_counter_regs[] = {
		0x1227, /* Pipe2 VS count */
		0x1228, /* Pipe2 HS count MSB */
		0x1229, /* Pipe2 HS count LSB */
		0x122A, /* Pipe2 DE count MSB */
		0x122B, /* Pipe2 DE count LSB */
	};

	static const u16 pipe3_hvd_counter_regs[] = {
		0x1237, /* Pipe3 VS count */
		0x1238, /* Pipe3 HS count MSB */
		0x1239, /* Pipe3 HS count LSB */
		0x123A, /* Pipe3 DE count MSB */
		0x123B, /* Pipe3 DE count LSB */
	};

	static const u16 efficiency_regs[] = {
		0x0100, 0x0112, 0x0124, 0x0136,
		0x0106, 0x0118, 0x012A, 0x013C,
	};

	static const u16 mipi_global_regs[] = {
		0x08A0, /* MIPI PHY mode */
		0x08A2, /* MIPI PHY enable */
		0x08A3, /* PHY0/1 lane map */
		0x08A4, /* PHY2/3 lane map */
		0x08A5, /* PHY0/1 lane polarity */
		0x08A6, /* PHY2/3 lane polarity */
		0x08CA, /* Pipe -> controller mapping */
	};

	static const u16 mipi_lane_count_regs[] = {
		0x090A, /* Controller0 lane count */
		0x094A, /* Controller1 lane count */
		0x098A, /* Controller2 lane count */
		0x09CA, /* Controller3 lane count */
	};

	static const u16 mipi_dpll_regs[] = {
		0x0415, /* PHY0 DPLL */
		0x0418, /* PHY1 DPLL */
		0x041B, /* PHY2 DPLL */
		0x041E, /* PHY3 DPLL */
	};

	static const u16 mipi_deskew_regs[] = {
		0x0903, 0x0904, /* TX0 deskew init/per */
		0x0943, 0x0944, /* TX1 deskew init/per */
		0x0983, 0x0984, /* TX2 deskew init/per */
		0x09C3, 0x09C4, /* TX3 deskew init/per */
	};

	static const u16 mipi_tx_status_regs[] = {
		0x0902, /* MIPI TX0 status */
		0x0942, /* MIPI TX1 status */
		0x0982, /* MIPI TX2 status */
		0x09C2, /* MIPI TX3 status */
	};

	static const u16 pipe0_map_regs[] = {
		0x090B, 0x090C,
		0x090D, 0x090E,
		0x090F, 0x0910,
		0x0911, 0x0912,
		0x092D,
	};

	static const u16 pipe1_map_regs[] = {
		0x094B, 0x094C,
		0x094D, 0x094E,
		0x094F, 0x0950,
		0x0951, 0x0952,
		0x096D,
	};

	static const u16 pipe2_map_regs[] = {
		0x098B, 0x098C,
		0x098D, 0x098E,
		0x098F, 0x0990,
		0x0991, 0x0992,
		0x09AD,
	};

	static const u16 pipe3_map_regs[] = {
		0x09CB, 0x09CC,
		0x09CD, 0x09CE,
		0x09CF, 0x09D0,
		0x09D1, 0x09D2,
		0x09ED,
	};

	static const u16 concat_regs[] = {
		0x0931, /* Controller0 concat */
		0x0971, /* Controller1 concat */
		0x09B1, /* Controller2 concat */
		0x09F1, /* Controller3 concat */
	};

	static const u16 mipi_packet_counter_regs[] = {
		0x08D0, /* CSI2 TX0/TX1 packet count */
		0x08D1, /* CSI2 TX2/TX3 packet count */
		0x08D2, /* PHY0/PHY1 packet count */
		0x08D3, /* PHY2/PHY3 packet count */
	};

	static const u16 error_flag_regs[] = {
		0x0026, /* PHY_INT_DEC_ERR / INTR3 */
		0x0028, /* EOM_ERR / LFLT / INTR5 */
		0x002A, /* GMSL / LCRC / VPRBS / remote / FS err / INTR7 */
		0x002C, /* Packet count / idle packet err / INTR9 */
		0x002E, /* Retransmission / max retransmission / INTR11 */

		0x0035, /* Decode error PHY A */
		0x0036, /* Decode error PHY B */
		0x0037, /* Decode error PHY C */
		0x0038, /* Decode error PHY D */

		0x0039, /* Idle error PHY A */
		0x003A, /* Idle error PHY B */
		0x003B, /* Idle error PHY C */
		0x003C, /* Idle error PHY D */
	};

	dev_info(priv->dev,
		 "\n[DUMP][%s] ========================================\n",
		 tag);
	dev_info(priv->dev,
		 "[DUMP][%s] ===== MAX96724 register dump start =====\n",
		 tag);

	max96724_dump_reg_group(priv, tag, "LINK / RATE / RESET / LOCK",
				link_regs, ARRAY_SIZE(link_regs));

	max96724_dump_reg_group(priv, tag, "PIPE SELECT / PIPE ENABLE",
				pipe_select_regs, ARRAY_SIZE(pipe_select_regs));

	max96724_dump_reg_group(priv, tag, "VIDEO LOCK",
				video_lock_regs, ARRAY_SIZE(video_lock_regs));

	max96724_dump_reg_group(priv, tag, "HVD DETECT / POLARITY",
				hvd_detect_regs, ARRAY_SIZE(hvd_detect_regs));

	max96724_dump_reg_group(priv, tag, "HVD COUNTER CONTROL",
				hvd_counter_ctrl_regs,
				ARRAY_SIZE(hvd_counter_ctrl_regs));

	max96724_dump_reg_group(priv, tag, "PIPE0 HVD COUNTERS",
				pipe0_hvd_counter_regs,
				ARRAY_SIZE(pipe0_hvd_counter_regs));

	max96724_dump_reg_group(priv, tag, "PIPE1 HVD COUNTERS",
				pipe1_hvd_counter_regs,
				ARRAY_SIZE(pipe1_hvd_counter_regs));

	max96724_dump_reg_group(priv, tag, "PIPE2 HVD COUNTERS",
				pipe2_hvd_counter_regs,
				ARRAY_SIZE(pipe2_hvd_counter_regs));

	max96724_dump_reg_group(priv, tag, "PIPE3 HVD COUNTERS",
				pipe3_hvd_counter_regs,
				ARRAY_SIZE(pipe3_hvd_counter_regs));

	max96724_dump_reg_group(priv, tag, "EFFICIENCY / HEARTBEAT",
				efficiency_regs, ARRAY_SIZE(efficiency_regs));

	max96724_dump_reg_group(priv, tag, "MIPI GLOBAL / LANE MAP / POLARITY",
				mipi_global_regs, ARRAY_SIZE(mipi_global_regs));

	max96724_dump_reg_group(priv, tag, "MIPI CONTROLLER LANE COUNT",
				mipi_lane_count_regs,
				ARRAY_SIZE(mipi_lane_count_regs));

	max96724_dump_reg_group(priv, tag, "MIPI DPLL",
				mipi_dpll_regs, ARRAY_SIZE(mipi_dpll_regs));

	max96724_dump_reg_group(priv, tag, "MIPI D-PHY DESKEW",
				mipi_deskew_regs, ARRAY_SIZE(mipi_deskew_regs));

	max96724_dump_reg_group(priv, tag, "MIPI TX STATUS",
				mipi_tx_status_regs,
				ARRAY_SIZE(mipi_tx_status_regs));

	max96724_dump_reg_group(priv, tag, "PIPE0 VC/DT MAP",
				pipe0_map_regs, ARRAY_SIZE(pipe0_map_regs));

	max96724_dump_reg_group(priv, tag, "PIPE1 VC/DT MAP",
				pipe1_map_regs, ARRAY_SIZE(pipe1_map_regs));

	max96724_dump_reg_group(priv, tag, "PIPE2 VC/DT MAP",
				pipe2_map_regs, ARRAY_SIZE(pipe2_map_regs));

	max96724_dump_reg_group(priv, tag, "PIPE3 VC/DT MAP",
				pipe3_map_regs, ARRAY_SIZE(pipe3_map_regs));

	max96724_dump_reg_group(priv, tag, "CONCATENATION",
				concat_regs, ARRAY_SIZE(concat_regs));

	max96724_dump_reg_group(priv, tag, "MIPI PACKET COUNTERS",
				mipi_packet_counter_regs,
				ARRAY_SIZE(mipi_packet_counter_regs));

	max96724_dump_reg_group(priv, tag, "ERROR FLAGS",
				error_flag_regs, ARRAY_SIZE(error_flag_regs));

	dev_info(priv->dev,
		 "[DUMP][%s] ===== MAX96724 register dump end =====\n",
		 tag);
	dev_info(priv->dev,
		 "[DUMP][%s] ========================================\n\n",
		 tag);
}

static int max96724_reg_dump_ops(struct max_des *des, const char *tag)
{
	struct max96724_priv *priv = des_to_priv(des);
	max96724_dump_regs(priv, tag);
	return 0;
}

static int max96724_des_init(struct max_des *des)
{
	struct max96724_priv *priv = des_to_priv(des);
	int ret;

	if (priv->des_hw_inited)
		return 0;

	/* Link Setting */
	ret = max96724_link_init(priv);
	if (ret)
		return ret;

	/* Pipe Setting */
	ret = max96724_pipe_init(priv);
	if (ret)
		return ret;

	/* PHY Seting */
	ret = max96724_phy_init(priv);
	if (ret)
		return ret;

	max96724_dump_regs(priv, "after_init_pre");

	priv->des_hw_inited = true;

	dev_info(priv->dev, "MAX96724 init setting complete. \n");

	return 0;
}

static int max96724_des_select_links(struct max_des *des, unsigned int mask)
{
	struct max96724_priv *priv = des_to_priv(des);
	u8 val = 0xF0 | (mask & 0x0F);

	return max96724_write(priv, 0x0006, val);
}









static void max96724_decode_pkt_regs(struct device *dev,
				     unsigned int reg8d0,
				     unsigned int reg8d1,
				     unsigned int reg8d2,
				     unsigned int reg8d3,
				     const char *tag)
{
	unsigned int csi0 = reg8d0 & 0x0f;
	unsigned int csi1 = (reg8d0 >> 4) & 0x0f;
	unsigned int csi2 = reg8d1 & 0x0f;
	unsigned int csi3 = (reg8d1 >> 4) & 0x0f;
	unsigned int phy0 = reg8d2 & 0x0f;
	unsigned int phy1 = (reg8d2 >> 4) & 0x0f;
	unsigned int phy2 = reg8d3 & 0x0f;
	unsigned int phy3 = (reg8d3 >> 4) & 0x0f;

	dev_info(dev,
		 "[%s] PKT raw: 0x08D0=0x%02x 0x08D1=0x%02x 0x08D2=0x%02x 0x08D3=0x%02x\n",
		 tag, reg8d0, reg8d1, reg8d2, reg8d3);

	dev_info(dev,
		 "[%s] CSI_TX pkt nibble: ctrl0=%u ctrl1=%u ctrl2=%u ctrl3=%u\n",
		 tag, csi0, csi1, csi2, csi3);

	dev_info(dev,
		 "[%s] PHY pkt nibble: phy0=%u phy1=%u phy2=%u phy3=%u\n",
		 tag, phy0, phy1, phy2, phy3);
}


static void max96724_log_pkt_delta(struct device *dev,
				   unsigned int prev8d0,
				   unsigned int prev8d1,
				   unsigned int prev8d2,
				   unsigned int prev8d3,
				   unsigned int cur8d0,
				   unsigned int cur8d1,
				   unsigned int cur8d2,
				   unsigned int cur8d3,
				   const char *tag)
{
	unsigned int prev_csi0 = prev8d0 & 0x0f;
	unsigned int prev_csi1 = (prev8d0 >> 4) & 0x0f;
	unsigned int prev_csi2 = prev8d1 & 0x0f;
	unsigned int prev_csi3 = (prev8d1 >> 4) & 0x0f;
	unsigned int prev_phy0 = prev8d2 & 0x0f;
	unsigned int prev_phy1 = (prev8d2 >> 4) & 0x0f;
	unsigned int prev_phy2 = prev8d3 & 0x0f;
	unsigned int prev_phy3 = (prev8d3 >> 4) & 0x0f;

	unsigned int cur_csi0 = cur8d0 & 0x0f;
	unsigned int cur_csi1 = (cur8d0 >> 4) & 0x0f;
	unsigned int cur_csi2 = cur8d1 & 0x0f;
	unsigned int cur_csi3 = (cur8d1 >> 4) & 0x0f;
	unsigned int cur_phy0 = cur8d2 & 0x0f;
	unsigned int cur_phy1 = (cur8d2 >> 4) & 0x0f;
	unsigned int cur_phy2 = cur8d3 & 0x0f;
	unsigned int cur_phy3 = (cur8d3 >> 4) & 0x0f;

	dev_info(dev,
		 "[%s] PKT delta: "
		 "ctrl0 %u->%u ctrl1 %u->%u ctrl2 %u->%u ctrl3 %u->%u | "
		 "phy0 %u->%u phy1 %u->%u phy2 %u->%u phy3 %u->%u\n",
		 tag,
		 prev_csi0, cur_csi0,
		 prev_csi1, cur_csi1,
		 prev_csi2, cur_csi2,
		 prev_csi3, cur_csi3,
		 prev_phy0, cur_phy0,
		 prev_phy1, cur_phy1,
		 prev_phy2, cur_phy2,
		 prev_phy3, cur_phy3);
}


static int max96724_des_log_pipe_status(struct max_des *des,
					struct max_des_pipe *pipe)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int val;
	unsigned int map_src0 = 0, map_dst0 = 0;
	unsigned int map_src1 = 0, map_dst1 = 0;
	unsigned int map_src2 = 0, map_dst2 = 0;
	unsigned int map_dest = 0;
	u16 base = 0x0900 + (pipe->index * 0x40);

	regmap_read(priv->regmap, 0x00F0 + (pipe->index / 2), &val);
	dev_info(priv->dev, "pipe%u VIDEO_PIPE_SEL=0x%02x\n", pipe->index, val);

	regmap_read(priv->regmap, 0x00F4, &val);
	dev_info(priv->dev, "pipe%u VIDEO_PIPE_EN=0x%02x\n", pipe->index, val);

	regmap_read(priv->regmap, base + 0x0A, &val);
	dev_info(priv->dev, "pipe%u MIPI_TX_LANE_CNT=0x%02x\n", pipe->index, val);

	regmap_read(priv->regmap, base + 0x0B, &val);
	dev_info(priv->dev, "pipe%u MAP_EN_L=0x%02x\n", pipe->index, val);

	regmap_read(priv->regmap, base + 0x0C, &val);
	dev_info(priv->dev, "pipe%u MAP_EN_H=0x%02x\n", pipe->index, val);

	regmap_read(priv->regmap, base + 0x0D, &map_src0);
	regmap_read(priv->regmap, base + 0x0E, &map_dst0);
	regmap_read(priv->regmap, base + 0x0F, &map_src1);
	regmap_read(priv->regmap, base + 0x10, &map_dst1);
	regmap_read(priv->regmap, base + 0x11, &map_src2);
	regmap_read(priv->regmap, base + 0x12, &map_dst2);
	regmap_read(priv->regmap, base + 0x2D, &map_dest);

	dev_info(priv->dev, "pipe%u MAP0_SRC=0x%02x MAP0_DST=0x%02x\n",
		 pipe->index, map_src0, map_dst0);
	dev_info(priv->dev, "pipe%u MAP1_SRC=0x%02x MAP1_DST=0x%02x\n",
		 pipe->index, map_src1, map_dst1);
	dev_info(priv->dev, "pipe%u MAP2_SRC=0x%02x MAP2_DST=0x%02x\n",
		 pipe->index, map_src2, map_dst2);
	dev_info(priv->dev, "pipe%u MAP_DPHY_DEST=0x%02x\n",
		 pipe->index, map_dest);

	dev_info(priv->dev,
		 "pipe%u decode: MAP0 image VC=%u DT=0x%02x -> VC=%u DT=0x%02x\n",
		 pipe->index,
		 (map_src0 >> 6) & 0x3, map_src0 & 0x3f,
		 (map_dst0 >> 6) & 0x3, map_dst0 & 0x3f);

	dev_info(priv->dev,
		 "pipe%u decode: MAP1 FS    VC=%u DT=0x%02x -> VC=%u DT=0x%02x\n",
		 pipe->index,
		 (map_src1 >> 6) & 0x3, map_src1 & 0x3f,
		 (map_dst1 >> 6) & 0x3, map_dst1 & 0x3f);

	dev_info(priv->dev,
		 "pipe%u decode: MAP2 FE    VC=%u DT=0x%02x -> VC=%u DT=0x%02x\n",
		 pipe->index,
		 (map_src2 >> 6) & 0x3, map_src2 & 0x3f,
		 (map_dst2 >> 6) & 0x3, map_dst2 & 0x3f);

	dev_info(priv->dev,
		 "pipe%u decode: map0->ctrl%u map1->ctrl%u map2->ctrl%u map3->ctrl%u\n",
		 pipe->index,
		 (map_dest >> 0) & 0x3,
		 (map_dest >> 2) & 0x3,
		 (map_dest >> 4) & 0x3,
		 (map_dest >> 6) & 0x3);

	return 0;
}

static int max96724_des_log_phy_status(struct max_des *des,
				       struct max_des_phy *phy)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int lock = 0;
	unsigned int phy_mode = 0, phy_en = 0;
	unsigned int lane_map01 = 0, lane_map23 = 0;
	unsigned int ctrl0_lane_cnt = 0, ctrl1_lane_cnt = 0;
	unsigned int ctrl2_lane_cnt = 0, ctrl3_lane_cnt = 0;
	unsigned int dpll0 = 0, dpll1 = 0, dpll2 = 0, dpll3 = 0;
	unsigned int reg8d0 = 0, reg8d1 = 0, reg8d2 = 0, reg8d3 = 0;

	regmap_read(priv->regmap, 0x001A, &lock);
	regmap_read(priv->regmap, 0x08A0, &phy_mode);
	regmap_read(priv->regmap, 0x08A2, &phy_en);
	regmap_read(priv->regmap, 0x08A3, &lane_map01);
	regmap_read(priv->regmap, 0x08A4, &lane_map23);

	regmap_read(priv->regmap, 0x090A, &ctrl0_lane_cnt);
	regmap_read(priv->regmap, 0x094A, &ctrl1_lane_cnt);
	regmap_read(priv->regmap, 0x098A, &ctrl2_lane_cnt);
	regmap_read(priv->regmap, 0x09CA, &ctrl3_lane_cnt);

	regmap_read(priv->regmap, 0x0415, &dpll0);
	regmap_read(priv->regmap, 0x0418, &dpll1);
	regmap_read(priv->regmap, 0x041B, &dpll2);
	regmap_read(priv->regmap, 0x041E, &dpll3);

	regmap_read(priv->regmap, 0x08D0, &reg8d0);
	regmap_read(priv->regmap, 0x08D1, &reg8d1);
	regmap_read(priv->regmap, 0x08D2, &reg8d2);
	regmap_read(priv->regmap, 0x08D3, &reg8d3);

	dev_info(priv->dev, "---------------- MIPI 4-lane PHY status ----------------\n");

	dev_info(priv->dev,
		 "[PHY-HW] 0x08A0=0x%02x 0x08A2=0x%02x 0x08A3=0x%02x 0x08A4=0x%02x\n",
		 phy_mode, phy_en, lane_map01, lane_map23);

	dev_info(priv->dev,
		 "[PHY-HW] CTRL lane cnt: C0=0x%02x C1=0x%02x C2=0x%02x C3=0x%02x\n",
		 ctrl0_lane_cnt, ctrl1_lane_cnt, ctrl2_lane_cnt, ctrl3_lane_cnt);

	dev_info(priv->dev,
		 "[PHY-HW] DPLL: 0x0415=0x%02x 0x0418=0x%02x 0x041B=0x%02x 0x041E=0x%02x preset=%s\n",
		 dpll0, dpll1, dpll2, dpll3, DES_TEST_RATE_NAME);

	dev_info(priv->dev,
		 "[PHY-DECODE] link lock raw 0x001A=0x%02x: link0=%u link1=%u link2=%u link3=%u\n",
		 lock,
		 !!(lock & BIT(0)),
		 !!(lock & BIT(1)),
		 !!(lock & BIT(2)),
		 !!(lock & BIT(3)));

	dev_info(priv->dev,
		 "[PHY-DECODE] 0x08A0: force_all_clk=%u phy3_clk=%u phy0_clk=%u mode_low=0x%x\n",
		 !!(phy_mode & BIT(7)),
		 !!(phy_mode & BIT(6)),
		 !!(phy_mode & BIT(5)),
		 phy_mode & 0x0f);

	dev_info(priv->dev,
		 "[PHY-DECODE] 0x08A2 enable: PHY3=%u PHY2=%u PHY1=%u PHY0=%u\n",
		 !!(phy_en & BIT(7)),
		 !!(phy_en & BIT(6)),
		 !!(phy_en & BIT(5)),
		 !!(phy_en & BIT(4)));

	dev_info(priv->dev,
		 "[PHY-DECODE] 4-lane expected path: Controller1 uses PHY0 + PHY1, so 0x08A2 should have bit4+bit5 set\n");

	dev_info(priv->dev,
		 "[PHY-DECODE] LANE_MAP01 0x08A3: phy1_l1=D%u phy1_l0=D%u phy0_l1=D%u phy0_l0=D%u\n",
		 (lane_map01 >> 6) & 0x3,
		 (lane_map01 >> 4) & 0x3,
		 (lane_map01 >> 2) & 0x3,
		 (lane_map01 >> 0) & 0x3);

	dev_info(priv->dev,
		 "[PHY-DECODE] LANE_MAP23 0x08A4: phy3_l1=D%u phy3_l0=D%u phy2_l1=D%u phy2_l0=D%u\n",
		 (lane_map23 >> 6) & 0x3,
		 (lane_map23 >> 4) & 0x3,
		 (lane_map23 >> 2) & 0x3,
		 (lane_map23 >> 0) & 0x3);

	dev_info(priv->dev,
		 "[PHY-DECODE] Controller1 lane field=%u => %s\n",
		 (ctrl1_lane_cnt >> 6) & 0x3,
		 (((ctrl1_lane_cnt >> 6) & 0x3) == 0x3) ? "4-lane" :
		 (((ctrl1_lane_cnt >> 6) & 0x3) == 0x2) ? "3-lane" :
		 (((ctrl1_lane_cnt >> 6) & 0x3) == 0x1) ? "2-lane" : "1-lane");

	max96724_decode_pkt_regs(priv->dev, reg8d0, reg8d1, reg8d2, reg8d3,
				 "PHY-STATUS");

	dev_info(priv->dev, "--------------------------------------------------------\n");

	return 0;
}

static int max96724_des_log_status(struct max_des *des)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int reg_0006 = 0;
	unsigned int reg_0018 = 0, reg_001a = 0;
	unsigned int reg_00f0 = 0, reg_00f1 = 0, reg_00f4 = 0;
	unsigned int reg_08a0 = 0, reg_08a2 = 0, reg_08a3 = 0, reg_08a4 = 0;
	unsigned int reg_08ca = 0;
	unsigned int reg_090a = 0, reg_094a = 0, reg_098a = 0, reg_09ca = 0;
	unsigned int reg_0943 = 0, reg_0944 = 0;
	unsigned int dpll0 = 0, dpll1 = 0, dpll2 = 0, dpll3 = 0;
	unsigned int pipe_sel[4];
	unsigned int pipe_link[4];
	unsigned int pipe_stream[4];
	int i;
	static const unsigned int sample_ms[] = { 0, 20, 50, 100, 200 };

	dev_info(priv->dev, "================= MAX96724 4-LANE STATUS =================\n");

	regmap_read(priv->regmap, 0x0006, &reg_0006);
	regmap_read(priv->regmap, 0x0018, &reg_0018);
	regmap_read(priv->regmap, 0x001A, &reg_001a);

	regmap_read(priv->regmap, 0x00F0, &reg_00f0);
	regmap_read(priv->regmap, 0x00F1, &reg_00f1);
	regmap_read(priv->regmap, 0x00F4, &reg_00f4);

	regmap_read(priv->regmap, 0x08A0, &reg_08a0);
	regmap_read(priv->regmap, 0x08A2, &reg_08a2);
	regmap_read(priv->regmap, 0x08A3, &reg_08a3);
	regmap_read(priv->regmap, 0x08A4, &reg_08a4);
	regmap_read(priv->regmap, 0x08CA, &reg_08ca);

	regmap_read(priv->regmap, 0x090A, &reg_090a);
	regmap_read(priv->regmap, 0x094A, &reg_094a);
	regmap_read(priv->regmap, 0x098A, &reg_098a);
	regmap_read(priv->regmap, 0x09CA, &reg_09ca);

	regmap_read(priv->regmap, 0x0943, &reg_0943);
	regmap_read(priv->regmap, 0x0944, &reg_0944);

	regmap_read(priv->regmap, 0x0415, &dpll0);
	regmap_read(priv->regmap, 0x0418, &dpll1);
	regmap_read(priv->regmap, 0x041B, &dpll2);
	regmap_read(priv->regmap, 0x041E, &dpll3);

	pipe_sel[0] = reg_00f0 & 0x0f;
	pipe_sel[1] = (reg_00f0 >> 4) & 0x0f;
	pipe_sel[2] = reg_00f1 & 0x0f;
	pipe_sel[3] = (reg_00f1 >> 4) & 0x0f;

	for (i = 0; i < 4; i++) {
		pipe_stream[i] = pipe_sel[i] & 0x3;
		pipe_link[i] = (pipe_sel[i] >> 2) & 0x3;
	}

	dev_info(priv->dev, "[HW] active=%d\n", des->active);

	dev_info(priv->dev,
		 "[HW] link select/reset/lock: 0x0006=0x%02x 0x0018=0x%02x 0x001A=0x%02x\n",
		 reg_0006, reg_0018, reg_001a);

	dev_info(priv->dev,
		 "[HW-DECODE] selected links: link0=%u link1=%u link2=%u link3=%u\n",
		 !!(reg_0006 & BIT(0)),
		 !!(reg_0006 & BIT(1)),
		 !!(reg_0006 & BIT(2)),
		 !!(reg_0006 & BIT(3)));

	dev_info(priv->dev,
		 "[HW-DECODE] locked links:   link0=%u link1=%u link2=%u link3=%u\n",
		 !!(reg_001a & BIT(0)),
		 !!(reg_001a & BIT(1)),
		 !!(reg_001a & BIT(2)),
		 !!(reg_001a & BIT(3)));

	dev_info(priv->dev,
		 "[HW] pipe select: 0x00F0=0x%02x 0x00F1=0x%02x 0x00F4=0x%02x\n",
		 reg_00f0, reg_00f1, reg_00f4);

	for (i = 0; i < 4; i++) {
		dev_info(priv->dev,
			 "[HW-DECODE] pipe%d: sel=0x%x link%d stream%d enabled=%u\n",
			 i,
			 pipe_sel[i],
			 pipe_link[i],
			 pipe_stream[i],
			 !!(reg_00f4 & BIT(i)));
	}

	dev_info(priv->dev,
		 "[HW] MIPI global: 0x08A0=0x%02x 0x08A2=0x%02x 0x08A3=0x%02x 0x08A4=0x%02x 0x08CA=0x%02x\n",
		 reg_08a0, reg_08a2, reg_08a3, reg_08a4, reg_08ca);

	dev_info(priv->dev,
		 "[HW] lane count: 0x090A=0x%02x 0x094A=0x%02x 0x098A=0x%02x 0x09CA=0x%02x\n",
		 reg_090a, reg_094a, reg_098a, reg_09ca);

	dev_info(priv->dev,
		 "[HW] deskew: 0x0943=0x%02x 0x0944=0x%02x\n",
		 reg_0943, reg_0944);

	dev_info(priv->dev,
		 "[HW] DPLL: 0x0415=0x%02x 0x0418=0x%02x 0x041B=0x%02x 0x041E=0x%02x preset=%s\n",
		 dpll0, dpll1, dpll2, dpll3, DES_TEST_RATE_NAME);

	dev_info(priv->dev,
		 "[HW-DECODE] 4-lane check: 0x08A2 bit4/bit5 should be 1 => PHY0=%u PHY1=%u\n",
		 !!(reg_08a2 & BIT(4)),
		 !!(reg_08a2 & BIT(5)));

	dev_info(priv->dev,
		 "[HW-DECODE] Controller1 lane field=%u => %s\n",
		 (reg_094a >> 6) & 0x3,
		 (((reg_094a >> 6) & 0x3) == 0x3) ? "4-lane" :
		 (((reg_094a >> 6) & 0x3) == 0x2) ? "3-lane" :
		 (((reg_094a >> 6) & 0x3) == 0x1) ? "2-lane" : "1-lane");

	dev_info(priv->dev,
		 "[SW] phy1 representative: enabled=%d lanes=%u clk_lane=%u freq=%lld\n",
		 des->phys[1].enabled,
		 des->phys[1].mipi.num_data_lanes,
		 des->phys[1].mipi.clock_lane,
		 des->phys[1].link_frequency);

	for (i = 0; i < 4; i++) {
		dev_info(priv->dev,
			 "[SW] pipe%d: enabled=%d link_id=%u stream_id=%u phy_id=%u\n",
			 i,
			 des->pipes[i].enabled,
			 des->pipes[i].link_id,
			 des->pipes[i].stream_id,
			 des->pipes[i].phy_id);
	}

	/*
	 * 4-lane output uses Controller1 with PHY0 + PHY1.
	 * So always print PHY status once, and print all enabled pipe maps.
	 */
	max96724_des_log_phy_status(des, &des->phys[1]);

	for (i = 0; i < 4; i++) {
		if (reg_00f4 & BIT(i))
			max96724_des_log_pipe_status(des, &des->pipes[i]);
	}

	for (i = 0; i < ARRAY_SIZE(sample_ms); i++) {
		unsigned int reg8d0 = 0, reg8d1 = 0, reg8d2 = 0, reg8d3 = 0;
		unsigned int next8d0 = 0, next8d1 = 0, next8d2 = 0, next8d3 = 0;

		if (sample_ms[i])
			msleep(sample_ms[i]);

		regmap_read(priv->regmap, 0x08D0, &reg8d0);
		regmap_read(priv->regmap, 0x08D1, &reg8d1);
		regmap_read(priv->regmap, 0x08D2, &reg8d2);
		regmap_read(priv->regmap, 0x08D3, &reg8d3);

		max96724_decode_pkt_regs(priv->dev, reg8d0, reg8d1, reg8d2, reg8d3,
					 "SAMPLE-A");

		usleep_range(3000, 5000);

		regmap_read(priv->regmap, 0x08D0, &next8d0);
		regmap_read(priv->regmap, 0x08D1, &next8d1);
		regmap_read(priv->regmap, 0x08D2, &next8d2);
		regmap_read(priv->regmap, 0x08D3, &next8d3);

		max96724_decode_pkt_regs(priv->dev, next8d0, next8d1, next8d2, next8d3,
					 "SAMPLE-B");

		max96724_log_pkt_delta(priv->dev,
				       reg8d0, reg8d1, reg8d2, reg8d3,
				       next8d0, next8d1, next8d2, next8d3,
				       "SAMPLE-DELTA");

		dev_info(priv->dev,
			 "[SAMPLE @%ums] Controller1/PHY0+PHY1 packet counters %s changing\n",
			 sample_ms[i],
			 (reg8d0 != next8d0 || reg8d1 != next8d1 ||
			  reg8d2 != next8d2 || reg8d3 != next8d3) ?
			 "ARE" : "are NOT");
	}

	dev_info(priv->dev, "=========================================================\n");

	return 0;
}

static const struct max_des_ops max96724_ops = {
	.num_phys = 4,
	.num_pipes = 4,
	.num_links = 4,
	.num_remaps_per_pipe = 16,

	.init = max96724_des_init,
	.select_links = max96724_des_select_links,
	.set_link_rate = max96724_des_set_link_rate,

	.log_status = max96724_des_log_status,
	.log_reg_status = max96724_reg_dump_ops,
	.log_pipe_status = max96724_des_log_pipe_status,
	.log_phy_status = max96724_des_log_phy_status,

};

static int max96724_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max96724_priv *priv;
	struct max_des_ops *ops;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ops = devm_kzalloc(dev, sizeof(*ops), GFP_KERNEL);
	if (!ops)
		return -ENOMEM;

	priv->info = &max96724_info;
	priv->dev = dev;
	priv->client = client;
	i2c_set_clientdata(client, priv);

	ret = max96724_upboard_pwren_gpio(client, true);
	if (ret)
		return ret;

	priv->regmap = devm_regmap_init_i2c(client, &max96724_i2c_regmap);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		goto err_power_off;
	}

	*ops = max96724_ops;
	priv->des.ops = ops;

	ret = max96724_reset(priv);
	if (ret)
		goto err_power_off;

	ret = max_des_probe(client, &priv->des);
	if (ret)
		goto err_power_off;

	/*
	 * v4l2_i2c_subdev_init() may overwrite i2c clientdata with
	 * struct v4l2_subdev *. Restore it back to max96724_priv.
	 */
	i2c_set_clientdata(client, priv);
	
	dev_info(dev, "[MAX96724] probe done: client=%px priv=%px des.priv=%px\n",
	 client, priv, priv->des.priv);

	return 0;

err_power_off:
	max96724_upboard_pwren_gpio(client, false);
	return ret;
}

static void max96724_remove(struct i2c_client *client)
{
	struct max96724_priv *priv = i2c_get_clientdata(client);

	dev_info(&client->dev, "[MAX96724] remove enter\n");

	if (!priv) {
		dev_warn(&client->dev, "[MAX96724] remove: priv is NULL\n");
		return;
	}

	/*
	 * Normal case:
	 * probe() restored clientdata to max96724_priv.
	 */
	if (priv->client != client || priv->dev != &client->dev) {
		dev_warn(&client->dev,
			 "[MAX96724] remove: invalid clientdata=%px, priv->client=%px client=%px priv->dev=%px dev=%px\n",
			 priv,
			 priv->client,
			 client,
			 priv->dev,
			 &client->dev);
		return;
	}

	dev_info(&client->dev,
		 "[MAX96724] remove ptrs: priv=%px des=%px des.priv=%px ops=%px\n",
		 priv, &priv->des, priv->des.priv, priv->des.ops);

	dev_info(&client->dev, "[MAX96724] remove: calling max_des_remove\n");

	max_des_remove(&priv->des);

	max96724_upboard_pwren_gpio(client, false);

	i2c_set_clientdata(client, NULL);

	dev_info(&client->dev, "[MAX96724] remove exit\n");
}

#ifdef CONFIG_ACPI
static const struct acpi_device_id max96724_acpi_ids[] = {
	{ "MAX96724", (kernel_ulong_t)&max96724_info },
	{ "APR96724", (kernel_ulong_t)&max96724_info },
	{ "INTC113C", (kernel_ulong_t)&max96724_info },
	{ }
};
MODULE_DEVICE_TABLE(acpi, max96724_acpi_ids);
#else
static const struct of_device_id max96724_of_table[] = {
	{ .compatible = "maxim,max96724", .data = &max96724_info },
	{ .compatible = "maxim,max96724f", .data = &max96724_info },
	{ .compatible = "maxim,max96724r", .data = &max96724_info },
	{ }
};
MODULE_DEVICE_TABLE(of, max96724_of_table);
#endif

static struct i2c_driver max96724_i2c_driver = {
	.driver = {
		.name = "max96724",
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(max96724_acpi_ids),
#else
		.of_match_table = max96724_of_table,
#endif
	},
	.probe = max96724_probe,
	.remove = max96724_remove,
};

module_i2c_driver(max96724_i2c_driver);

MODULE_SOFTDEP("pre: intel_ipu6_isys");
MODULE_IMPORT_NS("MAX_SERDES");
MODULE_DESCRIPTION("Maxim MAX96724 Quad GMSL2 Deserializer Driver");
MODULE_LICENSE("GPL");