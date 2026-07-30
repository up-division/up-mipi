// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim GMSL2 Deserializer Driver (V4L2 Pad Architecture Aligned with Vendor)
 * V4L2 Framework & Software Logic with Detailed Logging
 */

#include <linux/delay.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/ktime.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>

#include "max_des.h"

#ifndef MEDIA_PAD_FL_INTERNAL
#define MEDIA_PAD_FL_INTERNAL 0
#endif

#define DES_TEST_DPLL_REG 0x19
#define MAX_DES_LINK_FREQUENCY_DEFAULT DES_TEST_DPLL_REG * 50000000ULL

#define V4L2_FREQ MAX_DES_LINK_FREQUENCY_DEFAULT/4

#define TEST_LINK_ID 0

#define DES_PAD_SOURCE 0
#define DES_PAD_SINK   1
#define DES_NUM_PADS   2

#define MAX_DES_CAM_STREAM_REG 0x02BE
#define MAX_DES_CAM_STREAM_OFF 0x00
#define MAX_DES_CAM_STREAM_ON  0x10

#define MAX_DES_LINK_SETTLE_MS 100

struct max_des_priv {
	struct max_des *des;
	struct device *dev;
	struct i2c_client *client;
	struct i2c_mux_core *mux;

	struct mutex lock;
	struct list_head hotplug_node;

	struct media_pad *pads;
	struct regulator **pocs;

	u32 detected_links_mask;
	u64 requested_streams_mask;
	u64 active_streams_mask;

	struct v4l2_subdev sd;
	struct v4l2_async_notifier nf;
	struct v4l2_ctrl_handler ctrl_handler;
};

static inline struct max_des_priv *sd_to_priv(struct v4l2_subdev *sd)
{
	return container_of(sd, struct max_des_priv, sd);
}

static inline struct max_des_priv *nf_to_priv(struct v4l2_async_notifier *nf)
{
	return container_of(nf, struct max_des_priv, nf);
}

static struct max_des_phy *max_des_pad_to_phy(struct max_des *des, u32 pad)
{
	/*
	 * PortA 的實體 4-lane 輸出在硬體上可能由多個 PHY 組成，
	 * 但 V4L2 mbus config 這層只需要一個 representative phy。
	 *
	 * 目前仍用 phys[1] 當 PortA 4-lane representative。
	 * get_mbus_config() 會直接回傳 des->phys[1].mipi，
	 * 所以 num_data_lanes / data_lanes / link_frequency
	 * 都必須先在 phys[1] 上填成 PortA 的真實 4-lane 狀態。
	 */
	return &des->phys[1];
}

static int max_des_remote_write16(struct i2c_adapter *adap, u8 addr_7bit, u16 reg, u8 val)
{
	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	struct i2c_msg msg = {
		.addr = addr_7bit,
		.flags = 0,
		.len = sizeof(buf),
		.buf = buf,
	};

	return (i2c_transfer(adap, &msg, 1) == 1) ? 0 : -EIO;
}

static int max_des_remote_read16(struct i2c_adapter *adap, u8 addr_7bit,
				 u16 reg, u8 *val)
{
	u8 reg_buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[2] = {
		{
			.addr = addr_7bit,
			.flags = 0,
			.len = sizeof(reg_buf),
			.buf = reg_buf,
		},
		{
			.addr = addr_7bit,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = val,
		},
	};
	int ret;

	if (!val)
		return -EINVAL;

	ret = i2c_transfer(adap, msgs, ARRAY_SIZE(msgs));
	return ret == ARRAY_SIZE(msgs) ? 0 : (ret < 0 ? ret : -EIO);
}

static inline u8 max_des_ser_new_addr_8bit(unsigned int link_id)
{
	/* vendor script:
	 * A -> 0x82, B -> 0x84, C -> 0x86, D -> 0x88
	 */
	return 0x82 + (link_id * 2);
}

static bool max_des_i2c_addr_present(struct i2c_adapter *adap, u8 addr_7bit)
{
	struct i2c_msg msgs[1];
	u8 dummy = 0;
	int ret;

	msgs[0].addr  = addr_7bit;
	msgs[0].flags = I2C_M_RD;
	msgs[0].len   = 1;
	msgs[0].buf   = &dummy;

	ret = i2c_transfer(adap, msgs, 1);
	return ret == 1;
}

static void max_des_clear_link_scan_result(struct max_des_link *link)
{
	unsigned int index;

	if (!link)
		return;

	index = link->index;
	memset(link, 0, sizeof(*link));
	link->index = index;
}

static int max_des_set_all_link_rates(struct max_des_priv *priv,
				      enum cam_gmsl2_rx_rate rate)
{
	struct max_des *des = priv->des;
	unsigned int link;
	int ret;

	if (!des->ops->set_link_rate)
		return -EOPNOTSUPP;

	for (link = 0; link < des->ops->num_links; link++) {
		ret = des->ops->set_link_rate(des, link, rate);
		if (ret) {
			dev_err(priv->dev,
				"[DES-SCAN] failed to set link%u rate=%u: %d\n",
				link, rate, ret);
			return ret;
		}
	}

	/*
	 * After four per-link updates the MAX96724 registers are equivalent to:
	 *   rate 3G: 0x0010 = 0x11, 0x0011 = 0x11
	 *   rate 6G: 0x0010 = 0x22, 0x0011 = 0x22
	 */
	msleep(MAX_DES_LINK_SETTLE_MS);
	return 0;
}

static int max_des_read_serializer_identity(struct max_des_priv *priv,
					     unsigned int link_id,
					     u8 *ser_addr,
					     u8 *dev_id,
					     bool *dev_id_valid,
					     u8 *dev_rev,
					     bool *dev_rev_valid)
{
	struct i2c_adapter *adap = priv->client->adapter;
	u8 final_addr = 0x41 + link_id;
	u8 candidates[2] = { final_addr, 0x40 };
	unsigned int i;
	int ret;

	*ser_addr = 0;
	*dev_id = 0;
	*dev_rev = 0;
	*dev_id_valid = false;
	*dev_rev_valid = false;

	for (i = 0; i < ARRAY_SIZE(candidates); i++) {
		u8 id;
		u8 rev;

		ret = max_des_remote_read16(adap, candidates[i], 0x000d, &id);
		if (ret)
			continue;

		*ser_addr = candidates[i];
		*dev_id = id;
		*dev_id_valid = true;

		ret = max_des_remote_read16(adap, candidates[i], 0x000e, &rev);
		if (!ret) {
			*dev_rev = rev;
			*dev_rev_valid = true;
		}

		dev_info(priv->dev,
			 "[DES-SCAN] link%u serializer found at 0x%02x DEV_ID=0x%02x DEV_REV=%s0x%02x\n",
			 link_id, candidates[i], id,
			 *dev_rev_valid ? "" : "unreadable/",
			 *dev_rev_valid ? rev : 0);

		return 0;
	}

	dev_info(priv->dev,
		 "[DES-SCAN] link%u no readable serializer DEV_ID at 0x40/0x%02x\n",
		 link_id, final_addr);

	return -ENODEV;
}

static int max_des_scan_one_link(struct max_des_priv *priv,
				 unsigned int link_id,
				 enum cam_gmsl2_rx_rate rate)
{
	struct max_des *des = priv->des;
	struct max_des_link *link = &des->links[link_id];
	const struct cam_profile *profile;
	u8 ser_addr = 0;
	u8 dev_id = 0;
	u8 dev_rev = 0;
	bool dev_id_valid = false;
	bool dev_rev_valid = false;
	bool eeprom_0x50;
	bool eeprom_0x51;
	int ret;

	ret = des->ops->select_links(des, BIT(link_id));
	if (ret)
		return ret;

	msleep(MAX_DES_LINK_SETTLE_MS);

	/*
	 * Probe both serializer address states:
	 *   default address: 0x40
	 *   re-addressed link0..3: 0x41..0x44
	 */
	max_des_read_serializer_identity(priv, link_id,
					 &ser_addr,
					 &dev_id, &dev_id_valid,
					 &dev_rev, &dev_rev_valid);

	/*
	 * Module EEPROM signature:
	 *   ISX031 module: 0x50
	 *   AR0820C module: 0x51
	 *
	 * Only one physical link is selected, so identical EEPROM addresses on
	 * different links do not collide during this scan.
	 */
	eeprom_0x50 = max_des_i2c_addr_present(priv->client->adapter, 0x50);
	eeprom_0x51 = max_des_i2c_addr_present(priv->client->adapter, 0x51);

	dev_info(priv->dev,
		 "[DES-SCAN] link%u rate=%uG signatures: ser=%s addr=0x%02x id=%s0x%02x rev=%s0x%02x eeprom50=%u eeprom51=%u\n",
		 link_id,
		 rate == CAM_GMSL2_RX_RATE_6GBPS ? 6 : 3,
		 dev_id_valid ? "yes" : "no",
		 ser_addr,
		 dev_id_valid ? "" : "invalid/",
		 dev_id,
		 dev_rev_valid ? "" : "invalid/",
		 dev_rev,
		 eeprom_0x50,
		 eeprom_0x51);

	profile = cam_profile_match_scan(rate,
					 dev_id_valid,
					 dev_id,
					 eeprom_0x50,
					 eeprom_0x51);
	if (!profile)
		return -ENODEV;

	if (link->cam && link->cam != profile) {
		dev_warn(priv->dev,
			 "[DES-SCAN] link%u already matched %s, ignore conflicting %s\n",
			 link_id, link->cam->profile_name, profile->profile_name);
		return -EEXIST;
	}

	link->enabled = true;
	link->cam = profile;
	link->detected_rate = rate;
	link->serializer_addr_7bit = ser_addr;
	link->serializer_present = dev_id_valid;
	link->serializer_dev_id_valid = dev_id_valid;
	link->serializer_dev_id = dev_id;
	link->serializer_dev_rev_valid = dev_rev_valid;
	link->serializer_dev_rev = dev_rev;
	link->eeprom_present = eeprom_0x50 || eeprom_0x51;
	link->eeprom_addr_7bit = eeprom_0x50 ? 0x50 :
				 (eeprom_0x51 ? 0x51 : 0x00);

	dev_info(priv->dev,
		 "[DES-SCAN] link%u matched profile=%s module=%s rate=%uG serializer=%s EEPROM=0x%02x\n",
		 link_id,
		 profile->profile_name,
		 profile->module_name,
		 rate == CAM_GMSL2_RX_RATE_6GBPS ? 6 : 3,
		 profile->serializer.name,
		 link->eeprom_addr_7bit);

	return 0;
}

static int max_des_apply_detected_link_rates(struct max_des_priv *priv)
{
	struct max_des *des = priv->des;
	unsigned int link;
	int ret;
	int first_err = 0;

	for (link = 0; link < des->ops->num_links; link++) {
		enum cam_gmsl2_rx_rate rate;

		if (des->links[link].cam)
			rate = des->links[link].cam->serializer.forward_rate;
		else
			rate = CAM_GMSL2_RX_RATE_3GBPS;

		ret = des->ops->set_link_rate(des, link, rate);
		if (ret && !first_err)
			first_err = ret;
	}

	msleep(MAX_DES_LINK_SETTLE_MS);
	return first_err;
}

/*
 * Scan sequence requested for mixed ISX031 / AR0820C modules:
 *
 *   1. Set all links to 3G (0x0010=0x11, 0x0011=0x11).
 *   2. Select link0..3 individually (0x0006=0xf1/0xf2/0xf4/0xf8).
 *   3. Probe serializer 0x40 and 0x41..0x44 plus EEPROM 0x50/0x51.
 *   4. Set all links to 6G (0x0010=0x22, 0x0011=0x22).
 *   5. Repeat the four-link probe.
 *   6. Apply each detected camera profile's final per-link rate.
 */
static int max_des_scan_func(struct max_des_priv *priv)
{
	static const enum cam_gmsl2_rx_rate scan_rates[] = {
		CAM_GMSL2_RX_RATE_3GBPS,
		CAM_GMSL2_RX_RATE_6GBPS,
	};
	struct max_des *des;
	unsigned int rate_idx;
	unsigned int link;
	u32 detected_mask = 0;
	u32 restore_mask;
	int ret;
	int first_err = 0;

	if (!priv || !priv->des || !priv->des->ops)
		return -EINVAL;

	des = priv->des;

	if (!des->ops->select_links || !des->ops->set_link_rate)
		return -EOPNOTSUPP;

	dev_info(priv->dev,
		 "[DES-SCAN] ===== camera profile scan start =====\n");

	for (link = 0; link < des->ops->num_links; link++)
		max_des_clear_link_scan_result(&des->links[link]);

	for (rate_idx = 0; rate_idx < ARRAY_SIZE(scan_rates); rate_idx++) {
		enum cam_gmsl2_rx_rate rate = scan_rates[rate_idx];

		dev_info(priv->dev,
			 "[DES-SCAN] pass%u: set all links to %uGbps\n",
			 rate_idx,
			 rate == CAM_GMSL2_RX_RATE_6GBPS ? 6 : 3);

		ret = max_des_set_all_link_rates(priv, rate);
		if (ret)
			return ret;

		for (link = 0; link < des->ops->num_links; link++) {
			ret = max_des_scan_one_link(priv, link, rate);
			if (ret == -ENODEV)
				continue;

			if (ret && ret != -EEXIST && !first_err)
				first_err = ret;
		}
	}

	for (link = 0; link < des->ops->num_links; link++) {
		if (des->links[link].enabled && des->links[link].cam)
			detected_mask |= BIT(link);
	}

	ret = max_des_apply_detected_link_rates(priv);
	if (ret && !first_err)
		first_err = ret;

	restore_mask = detected_mask;
	if (!restore_mask)
		restore_mask = (1U << des->ops->num_links) - 1;

	ret = des->ops->select_links(des, restore_mask);
	if (ret && !first_err)
		first_err = ret;

	dev_info(priv->dev,
		 "[DES-SCAN] scan complete: detected_mask=0x%x first_err=%d\n",
		 detected_mask, first_err);

	for (link = 0; link < des->ops->num_links; link++) {
		const struct max_des_link *scan = &des->links[link];

		dev_info(priv->dev,
			 "[DES-SCAN] result link%u: enabled=%u profile=%s rate=%uG ser_addr=0x%02x id=%s0x%02x rev=%s0x%02x eeprom=0x%02x\n",
			 link,
			 scan->enabled,
			 scan->cam ? scan->cam->profile_name : "none",
			 scan->cam &&
			 scan->cam->serializer.forward_rate ==
				CAM_GMSL2_RX_RATE_6GBPS ? 6 : 3,
			 scan->serializer_addr_7bit,
			 scan->serializer_dev_id_valid ? "" : "invalid/",
			 scan->serializer_dev_id,
			 scan->serializer_dev_rev_valid ? "" : "invalid/",
			 scan->serializer_dev_rev,
			 scan->eeprom_addr_7bit);
	}

	dev_info(priv->dev,
		 "[DES-SCAN] ===== camera profile scan end =====\n");

	if (!detected_mask)
		return first_err ? first_err : -ENODEV;

	return first_err;
}


static int max_des_init_one_serializer(struct max_des_priv *priv,
				       struct i2c_adapter *adap,
				       unsigned int link_id)
{
	u8 final_addr = max_des_ser_new_addr_8bit(link_id) >> 1;
	int ret;

	dev_info(priv->dev,
		 "[DES-PROBE] [Link %u] probing serializer: default 0x40 or final 0x%02x\n",
		 link_id, final_addr);

	if (max_des_i2c_addr_present(adap, final_addr)) {
		dev_info(priv->dev,
			"[DES-PROBE] [Link %u] serializer already present at final addr 0x%02x\n",
			link_id, final_addr);

		/*
		* In case the serializer/camera was already streaming from a previous run,
		* force it to stopped state during init.
		*/
		ret = max_des_remote_write16(adap, final_addr,
						MAX_DES_CAM_STREAM_REG,
						MAX_DES_CAM_STREAM_OFF);
		if (ret)
			return ret;

		dev_info(priv->dev,
			"[DES-PROBE] [Link %u] camera stream held off: 0x%02x[0x%04x] = 0x%02x\n",
			link_id, final_addr, MAX_DES_CAM_STREAM_REG, MAX_DES_CAM_STREAM_OFF);

		return 0;
	}

	if (!max_des_i2c_addr_present(adap, 0x40)) {
		dev_info(priv->dev,
			 "[DES-PROBE] [Link %u] no serializer detected (0x40 / 0x%02x)\n",
			 link_id, final_addr);
		return -ENODEV;
	}

	dev_info(priv->dev,
		 "[DES-PROBE] [Link %u] serializer found at default addr 0x40\n",
		 link_id);

	/* vendor script sequence */
	ret = max_des_remote_write16(adap, 0x40, 0x02BE, MAX_DES_CAM_STREAM_OFF);
	if (ret) return ret;

	ret = max_des_remote_write16(adap, 0x40, 0x0318, 0x5E);
	if (ret) return ret;

	ret = max_des_remote_write16(adap, 0x40, 0x02D3, 0x00);
	if (ret) return ret;
	msleep(300);

	ret = max_des_remote_write16(adap, 0x40, 0x02D3, 0x10);
	if (ret) return ret;

	ret = max_des_remote_write16(adap, 0x40, 0x02D6, 0x00);
	if (ret) return ret;
	msleep(300);

	ret = max_des_remote_write16(adap, 0x40, 0x02D6, 0x10);
	if (ret) return ret;

	ret = max_des_remote_write16(adap, 0x40, 0x0000, max_des_ser_new_addr_8bit(link_id));
	if (ret) return ret;

	msleep(300);

	dev_info(priv->dev,
		 "[DES-PROBE] [Link %u] serializer re-addressed: 0x40 -> 0x%02x\n",
		 link_id, final_addr);

	ret = max_des_remote_write16(adap, final_addr,
			     MAX_DES_CAM_STREAM_REG,
			     MAX_DES_CAM_STREAM_OFF);
	if (ret)
		return ret;

	dev_info(priv->dev,
	 "[DES-PROBE] [Link %u] camera stream held off: 0x%02x[0x%04x] = 0x%02x\n",
	 link_id, final_addr, MAX_DES_CAM_STREAM_REG,
	 MAX_DES_CAM_STREAM_OFF);

	return 0;
}

static int max_des_i2c_mux_select(struct i2c_mux_core *muxc, u32 chan)
{
	struct max_des_priv *priv = muxc->priv;
	struct max_des *des = priv->des;
	int ret;

	if (!des || !des->ops || !des->ops->select_links)
		return -EOPNOTSUPP;

	if (chan >= des->ops->num_links)
		return -EINVAL;

	ret = des->ops->select_links(des, BIT(chan));
	if (ret)
		dev_err(priv->dev,
			"[DES-I2C-MUX] failed to switch to link %u: %d\n",
			chan, ret);

	return ret;
}

static int max_des_i2c_mux_init(struct max_des_priv *priv)
{
	struct max_des *des = priv->des;
	u32 flags = I2C_MUX_LOCKED;
	unsigned int i;
	unsigned int restore_mask = 0;
	int ret;

	dev_info(priv->dev, "[DES-PROBE] 3. Registering I2C MUX for GMSL links...\n");

	priv->mux = i2c_mux_alloc(priv->client->adapter, priv->dev,
				  des->ops->num_links, 0, flags,
				  max_des_i2c_mux_select, NULL);
	if (!priv->mux)
		return -ENOMEM;

	priv->mux->priv = priv;

	for (i = 0; i < des->ops->num_links; i++) {
		dev_info(priv->dev, "[DES-PROBE] 3a. Adding MUX adapter for Link %u...\n", i);
		ret = i2c_mux_add_adapter(priv->mux, 0, i);
		if (ret)
			goto err_add_adapters;

		if (!priv->mux->adapter[i])
			continue;

		/*
		 * Add all physical link adapters so a later idle rescan can detect
		 * a newly hot-plugged camera.  Only initialize serializers that were
		 * matched by max_des_scan_func().
		 */
		if (!des->links[i].enabled || !des->links[i].cam)
			continue;

		/* 每條 link 切換後，照 vendor script 等待 */
		if (des->ops->select_links) {
			ret = des->ops->select_links(des, BIT(i));
			if (ret) {
				dev_warn(priv->dev,
					 "[DES-PROBE] [Link %u] select_links failed: %d\n",
					 i, ret);
				continue;
			}
		}

		msleep(300);

		ret = max_des_init_one_serializer(priv, priv->mux->adapter[i], i);
		if (ret == -ENODEV) {
			des->links[i].enabled = false;
			continue;
		}
		if (ret) {
			des->links[i].enabled = false;
			dev_warn(priv->dev,
				"[DES-PROBE] [Link %u] serializer init failed: %d\n",
				i, ret);
			continue;
		}

		des->links[i].enabled = true;
	}

	for (i = 0; i < des->ops->num_links; i++) {
		if (des->links[i].enabled)
			restore_mask |= BIT(i);
	}
	if (!restore_mask) {
		dev_err(priv->dev,
			"[DES-PROBE] no serializer links detected, restore_mask=0\n");
		return -ENODEV;
	}

	/* Restore detected links after per-link I2C mux probing */
	if (des->ops->select_links) {
		ret = des->ops->select_links(des, restore_mask);
		if (ret)
			dev_warn(priv->dev, "[DES-PROBE] restore all links failed: %d\n", ret);
	}

	dev_info(priv->dev,
		"[DES-PROBE] restore detected links mask=0x%x\n",
		restore_mask);

	return 0;

err_add_adapters:
	dev_err(priv->dev, "[DES-PROBE] Failed to add MUX adapter for link %d\n", i);
	i2c_mux_del_adapters(priv->mux);
	return ret;
}



static int max_des_rescan_links_idle(struct max_des_priv *priv)
{
	struct max_des *des;
	unsigned int i;
	u32 detected_mask = 0;
	u32 restore_mask;
	ktime_t t_start;
	s64 elapsed_ms;
	int ret = 0;
	int first_err = 0;

	if (!priv)
		return -EINVAL;

	des = priv->des;
	if (!des || !des->ops || !des->ops->num_links)
		return -EINVAL;

	t_start = ktime_get();

	mutex_lock(&priv->lock);

	if (des->active || priv->active_streams_mask) {
		dev_warn(priv->dev,
			 "[DES-HOTPLUG] rescan rejected: DES active=%d active_mask=0x%llx\n",
			 des->active,
			 (unsigned long long)priv->active_streams_mask);
		ret = -EBUSY;
		goto out_unlock;
	}

	if (!priv->mux) {
		dev_err(priv->dev,
			"[DES-HOTPLUG] rescan failed: mux not initialized\n");
		ret = -ENODEV;
		goto out_unlock;
	}

	dev_info(priv->dev,
		 "[DES-HOTPLUG] manual idle profile rescan start\n");

	ret = max_des_scan_func(priv);
	if (ret) {
		dev_warn(priv->dev,
			 "[DES-HOTPLUG] profile scan failed: %d\n", ret);
		first_err = ret;
	}

	for (i = 0; i < des->ops->num_links; i++) {
		if (!des->links[i].enabled || !des->links[i].cam)
			continue;

		if (!priv->mux->adapter[i]) {
			dev_warn(priv->dev,
				 "[DES-HOTPLUG] link%u has no mux adapter\n", i);
			des->links[i].enabled = false;
			if (!first_err)
				first_err = -ENODEV;
			continue;
		}

		if (des->ops->select_links) {
			ret = des->ops->select_links(des, BIT(i));
			if (ret) {
				dev_warn(priv->dev,
					 "[DES-HOTPLUG] link%u select failed: %d\n",
					 i, ret);
				des->links[i].enabled = false;
				if (!first_err)
					first_err = ret;
				continue;
			}
		}

		msleep(MAX_DES_LINK_SETTLE_MS);

		ret = max_des_init_one_serializer(priv,
						  priv->mux->adapter[i], i);
		if (ret) {
			dev_warn(priv->dev,
				 "[DES-HOTPLUG] link%u serializer init failed: %d\n",
				 i, ret);
			des->links[i].enabled = false;
			if (!first_err)
				first_err = ret;
			continue;
		}

		detected_mask |= BIT(i);

		dev_info(priv->dev,
			 "[DES-HOTPLUG] link%u initialized as %s\n",
			 i, des->links[i].cam->profile_name);
	}

	priv->detected_links_mask = detected_mask;

	restore_mask = detected_mask;
	if (!restore_mask)
		restore_mask = (1U << des->ops->num_links) - 1;

	if (des->ops->select_links) {
		ret = des->ops->select_links(des, restore_mask);
		if (ret) {
			dev_warn(priv->dev,
				 "[DES-HOTPLUG] restore links failed: mask=0x%x ret=%d\n",
				 restore_mask, ret);
			if (!first_err)
				first_err = ret;
		}
	}

	elapsed_ms = ktime_ms_delta(ktime_get(), t_start);

	dev_info(priv->dev,
		 "[DES-HOTPLUG] manual idle rescan done: detected_links_mask=0x%x first_err=%d elapsed=%lld ms\n",
		 priv->detected_links_mask, first_err, elapsed_ms);

	ret = first_err;

out_unlock:
	mutex_unlock(&priv->lock);
	return ret;
}

static LIST_HEAD(max_des_instances);
static DEFINE_MUTEX(max_des_instances_lock);
static int max_des_hotplug_rescan;

static int max_des_hotplug_rescan_set(const char *val,
				      const struct kernel_param *kp)
{
	struct max_des_priv *priv;
	int value;
	int ret;
	int first_err = 0;
	int scanned = 0;
	int skipped = 0;

	ret = kstrtoint(val, 0, &value);
	if (ret)
		return ret;

	if (!value) {
		max_des_hotplug_rescan = 0;
		return 0;
	}

	mutex_lock(&max_des_instances_lock);

	list_for_each_entry(priv, &max_des_instances, hotplug_node) {
		struct max_des *des;

		if (!priv || !priv->des)
			continue;

		des = priv->des;

		if (des->active || priv->active_streams_mask) {
			dev_warn(priv->dev,
				 "[DES-HOTPLUG] manual rescan skipped: active=%d active_mask=0x%llx\n",
				 des->active,
				 (unsigned long long)priv->active_streams_mask);
			skipped++;
			continue;
		}

		dev_info(priv->dev,
			 "[DES-HOTPLUG] manual rescan triggered by module parameter\n");

		ret = max_des_rescan_links_idle(priv);
		if (ret) {
			dev_warn(priv->dev,
				 "[DES-HOTPLUG] manual rescan failed or partial: %d\n",
				 ret);
			if (!first_err)
				first_err = ret;
		}

		scanned++;
	}

	mutex_unlock(&max_des_instances_lock);

	max_des_hotplug_rescan = 0;

	pr_info("[DES-HOTPLUG] manual rescan done: scanned=%d skipped=%d first_err=%d trigger reset to 0\n",
		scanned, skipped, first_err);

	return 0;
}

static const struct kernel_param_ops max_des_hotplug_rescan_ops = {
	.set = max_des_hotplug_rescan_set,
	.get = param_get_int,
};

module_param_cb(hotplug_rescan,
		&max_des_hotplug_rescan_ops,
		&max_des_hotplug_rescan,
		0644);
MODULE_PARM_DESC(hotplug_rescan,
		 "Write 1 to manually rescan MAX DES links when idle; auto resets to 0");

static void max_des_fill_default_fmt(struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = 3840;
	fmt->height = 2160;
	fmt->code = MEDIA_BUS_FMT_UYVY8_1X16;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
}

static int max_des_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct max_des_priv *priv = sd_to_priv(sd);
	struct v4l2_subdev_route routes[4] = {
		{
			.sink_pad = DES_PAD_SINK,
			.sink_stream = 0,
			.source_pad = DES_PAD_SOURCE,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
		{
			.sink_pad = DES_PAD_SINK,
			.sink_stream = 1,
			.source_pad = DES_PAD_SOURCE,
			.source_stream = 1,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
		{
			.sink_pad = DES_PAD_SINK,
			.sink_stream = 2,
			.source_pad = DES_PAD_SOURCE,
			.source_stream = 2,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
		{
			.sink_pad = DES_PAD_SINK,
			.sink_stream = 3,
			.source_pad = DES_PAD_SOURCE,
			.source_stream = 3,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
	};
	struct v4l2_subdev_krouting routing = {
		.routes = routes,
		.num_routes = ARRAY_SIZE(routes),
	};
	struct v4l2_mbus_framefmt *fmt;
	unsigned int i;
	int ret;

	dev_info(priv->dev,
		 "[DES-DBG] init_state() called, fixed stream0~3 route mode\n");

	ret = v4l2_subdev_set_routing(sd, state, &routing);
	if (ret)
		return ret;

	for (i = 0; i < 4; i++) {
		fmt = v4l2_subdev_state_get_format(state, DES_PAD_SINK, i);
		if (fmt)
			max_des_fill_default_fmt(fmt);

		fmt = v4l2_subdev_state_get_format(state, DES_PAD_SOURCE, i);
		if (fmt)
			max_des_fill_default_fmt(fmt);
	}

	return 0;
}

static int max_des_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *f = v4l2_subdev_state_get_format(state, format->pad, format->stream);
	if (f && f->width && f->height && f->code) {
		format->format = *f;
		return 0;
	}
	max_des_fill_default_fmt(&format->format);
	if (f) *f = format->format;
	return 0;
}

static int max_des_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *f;
	if (!format->format.width || !format->format.height || !format->format.code)
		max_des_fill_default_fmt(&format->format);
	f = v4l2_subdev_state_get_format(state, format->pad, format->stream);
	if (f) *f = format->format;
	return 0;
}

static int max_des_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	struct max_des_priv *priv = sd_to_priv(sd);
	unsigned int i;

	dev_info(priv->dev,
		 "[DES-DBG] get_frame_desc() called: pad=%u, fixed 4-entry VC0~VC3\n",
		 pad);

	if (pad != DES_PAD_SOURCE) {
		fd->num_entries = 0;
		return 0;
	}

	memset(fd, 0, sizeof(*fd));

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 4;

	for (i = 0; i < 4; i++) {
		fd->entry[i].stream = i;
		fd->entry[i].flags = V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
		fd->entry[i].length = 1920 * 1536 * 2 * 2;
		fd->entry[i].pixelcode = MEDIA_BUS_FMT_UYVY8_1X16;
		fd->entry[i].bus.csi2.vc = i;
		fd->entry[i].bus.csi2.dt = 0x1E;

		dev_info(priv->dev,
			 "[DES-DBG] frame_desc entry[%u]: stream=%u vc=%u dt=0x%02x len=%u\n",
			 i,
			 fd->entry[i].stream,
			 fd->entry[i].bus.csi2.vc,
			 fd->entry[i].bus.csi2.dt,
			 fd->entry[i].length);
	}

	return 0;
}

static int max_des_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				   struct v4l2_mbus_config *cfg)
{
	struct max_des_priv *priv = sd_to_priv(sd);
	struct max_des_phy *phy = max_des_pad_to_phy(priv->des, pad);

	if (!phy) return -EINVAL;
	cfg->type = V4L2_MBUS_CSI2_DPHY; /* 強制覆寫避免未初始化錯誤 */
	cfg->bus.mipi_csi2 = phy->mipi;
	return 0;
}

static int max_des_set_routing(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       enum v4l2_subdev_format_whence which,
			       struct v4l2_subdev_krouting *routing)
{
	struct max_des_priv *priv = sd_to_priv(sd);

	dev_info(priv->dev,
		 "[DES-DBG] set_routing() called: which=%u num_routes=%u active=%d\n",
		 which, routing ? routing->num_routes : 0, priv->des->active);

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE && priv->des->active) {
		dev_info(priv->dev,
			 "[DES-DBG] set_routing: reject with -EBUSY because des is active\n");
		return -EBUSY;
	}

	return v4l2_subdev_set_routing(sd, state, routing);
}




static int max_des_set_camera_stream_one(struct max_des_priv *priv,
					 unsigned int link_id,
					 bool enable)
{
	struct i2c_adapter *adap;
	u8 ser_addr;
	u8 val;
	int ret;

	if (!priv || !priv->des)
		return -EINVAL;

	if (link_id >= priv->des->ops->num_links)
		return -EINVAL;

	/*
	 * Runtime camera reset control must NOT use priv->mux->adapter[link_id].
	 * Using mux adapter will trigger max_des_i2c_mux_select(), which changes
	 * MAX96724 0x0006 to one-link-only, e.g. link2 -> 0xF4.
	 *
	 * Serializer has already been re-addressed to unique final address,
	 * so use parent I2C adapter directly.
	 */
	adap = priv->client->adapter;

	ser_addr = max_des_ser_new_addr_8bit(link_id) >> 1;
	val = enable ? MAX_DES_CAM_STREAM_ON : MAX_DES_CAM_STREAM_OFF;

	ret = max_des_remote_write16(adap, ser_addr,
				     MAX_DES_CAM_STREAM_REG, val);
	if (ret)
		return ret;

	dev_info(priv->dev,
		 "[CAM-STREAM] link%u serializer 0x%02x reg 0x%04x = 0x%02x (%s)\n",
		 link_id, ser_addr, MAX_DES_CAM_STREAM_REG, val,
		 enable ? "ON" : "OFF");

	return 0;
}

static int max_des_set_camera_stream_mask(struct max_des_priv *priv,
					  u64 streams_mask,
					  bool enable)
{
	struct max_des *des = priv->des;
	unsigned int i;
	u32 mask = (u32)(streams_mask & 0xFULL);
	int ret;
	int first_err = 0;

	if (!mask) {
		dev_info(priv->dev,
			 "[CAM-STREAM] empty stream mask for camera %s, skip\n",
			 enable ? "ON" : "OFF");
		return 0;
	}

	for (i = 0; i < des->ops->num_links; i++) {
		if (!(mask & BIT(i)))
			continue;

		ret = max_des_set_camera_stream_one(priv, i, enable);
		if (ret) {
			dev_warn(priv->dev,
				 "[CAM-STREAM] failed to set link%u camera %s: %d\n",
				 i, enable ? "ON" : "OFF", ret);

			if (!first_err)
				first_err = ret;
		}
	}

	return first_err;
}

static int max_des_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct max_des_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;
	struct max_des *des = priv->des;
	u64 cam_mask;
	int ret = 0;

	dev_info(priv->dev, "============================================\n");
	dev_info(priv->dev, "[DES-STREAM] s_stream(%d) START active=%d requested=0x%llx active_mask=0x%llx\n",
		 enable, des->active,
		 (unsigned long long)priv->requested_streams_mask,
		 (unsigned long long)priv->active_streams_mask);

	if (enable) {
		if (des->active) {
			dev_info(priv->dev,
				 "[DES-STREAM] already active, skip legacy s_stream enable\n");
			dev_info(priv->dev, "============================================\n\n");
			return 0;
		}

		if (pm_runtime_enabled(&client->dev)) {
			ret = pm_runtime_resume_and_get(&client->dev);
			if (ret < 0) {
				dev_err(priv->dev,
					"[DES-STREAM] pm_runtime_resume_and_get failed: %d\n",
					ret);
				dev_info(priv->dev, "============================================\n\n");
				return ret;
			}
		}

		/*
		 * Stream path must NOT touch MAX96724 link/pipe registers.
		 * max96724 init already enabled DES link/pipe/PHY/DPLL/deskew.
		 * Only release camera reset here.
		 */
		cam_mask = priv->requested_streams_mask;

		ret = max_des_set_camera_stream_mask(priv, cam_mask, true);
		if (ret) {
			dev_err(priv->dev,
				"[CAM-STREAM] failed to start camera mask=0x%llx: %d\n",
				(unsigned long long)cam_mask, ret);

			max_des_set_camera_stream_mask(priv, cam_mask, false);

			if (pm_runtime_enabled(&client->dev))
				pm_runtime_put(&client->dev);

			dev_info(priv->dev, "============================================\n\n");
			return ret;
		}

		des->active = true;

		if(des->ops->log_reg_status){
			des->ops->log_reg_status(des, "after_stream");
		}

		dev_info(priv->dev,
			 "[DES-STREAM] camera stream ON mask=0x%llx\n",
			 (unsigned long long)cam_mask);
		dev_info(priv->dev, "============================================\n\n");

		return 0;
	}

	/*
	 * Legacy stop path. Normally disable_streams() handles per-stream stop.
	 * If legacy s_stream(0) is called, stop all currently active cameras.
	 */
	cam_mask = priv->active_streams_mask;

	ret = max_des_set_camera_stream_mask(priv, cam_mask, false);
	if (ret)
		dev_warn(priv->dev,
			 "[CAM-STREAM] failed to stop camera mask=0x%llx: %d\n",
			 (unsigned long long)cam_mask, ret);

	if (!des->active) {
		dev_info(priv->dev,
			 "[DES-STREAM] already stopped, camera OFF requested\n");
		dev_info(priv->dev, "============================================\n\n");
		return 0;
	}

	des->active = false;
	priv->active_streams_mask = 0;
	priv->requested_streams_mask = 0;

	if (pm_runtime_enabled(&client->dev))
		pm_runtime_put(&client->dev);

	dev_info(priv->dev, "[DES-STREAM] camera stream OFF mask=0x%llx\n",
		 (unsigned long long)cam_mask);
	dev_info(priv->dev, "============================================\n\n");

	return 0;
}

static int max_des_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	struct max_des_priv *priv = sd_to_priv(sd);
	struct max_des *des = priv->des;
	u64 old_mask = priv->active_streams_mask;
	u64 new_mask = old_mask | streams_mask;
	u64 start_mask = new_mask & ~old_mask;
	int ret;

	dev_info(priv->dev,
		 "[DES-DBG] enable_streams(): pad=%u streams_mask=0x%llx old_active=0x%llx start_mask=0x%llx new_active=0x%llx detected=0x%x\n",
		 pad,
		 (unsigned long long)streams_mask,
		 (unsigned long long)old_mask,
		 (unsigned long long)start_mask,
		 (unsigned long long)new_mask,
		 priv->detected_links_mask);

	if (!streams_mask)
		return -EINVAL;

	if (new_mask & ~0xFULL) {
		dev_err(priv->dev,
			"[DES-STREAM] supports only stream0~3, new_mask=0x%llx\n",
			(unsigned long long)new_mask);
		return -EINVAL;
	}

	if (new_mask & ~priv->detected_links_mask) {
		dev_err(priv->dev,
			"[DES-STREAM] requested missing link(s): new_mask=0x%llx detected=0x%x\n",
			(unsigned long long)new_mask,
			priv->detected_links_mask);
		return -ENODEV;
	}

	if (!start_mask) {
		dev_info(priv->dev,
			 "[DES-STREAM] requested streams already active: 0x%llx\n",
			 (unsigned long long)streams_mask);
		return 0;
	}

	priv->requested_streams_mask = new_mask;

	/*
	 * First stream enable.
	 * Do not call max96724 stream_prepare here.
	 * DES hardware path is already pre-enabled in max96724 init.
	 */
	if (!des->active) {
		ret = max_des_s_stream(sd, 1);
		if (ret) {
			priv->requested_streams_mask = old_mask;
			return ret;
		}

		priv->active_streams_mask = new_mask;

		dev_info(priv->dev,
			 "[DES-STREAM] first enable done: active_streams_mask=0x%llx\n",
			 (unsigned long long)priv->active_streams_mask);
		return 0;
	}

	/*
	 * DES already active:
	 * only release newly requested camera reset lines.
	 */
	ret = max_des_set_camera_stream_mask(priv, start_mask, true);
	if (ret) {
		priv->requested_streams_mask = old_mask;
		return ret;
	}

	priv->active_streams_mask = new_mask;
	priv->requested_streams_mask = new_mask;

	dev_info(priv->dev,
		 "[DES-STREAM] partial enable done: start_mask=0x%llx active_streams_mask=0x%llx\n",
		 (unsigned long long)start_mask,
		 (unsigned long long)priv->active_streams_mask);

	return 0;
}

static int max_des_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   u32 pad, u64 streams_mask)
{
	struct max_des_priv *priv = sd_to_priv(sd);
	struct max_des *des = priv->des;
	u64 old_mask = priv->active_streams_mask;
	u64 stop_mask = old_mask & streams_mask;
	u64 new_mask = old_mask & ~streams_mask;
	int ret;

	dev_info(priv->dev,
		 "[DES-DBG] disable_streams(): pad=%u streams_mask=0x%llx old_active=0x%llx stop_mask=0x%llx new_active=0x%llx\n",
		 pad,
		 (unsigned long long)streams_mask,
		 (unsigned long long)old_mask,
		 (unsigned long long)stop_mask,
		 (unsigned long long)new_mask);

	if (!stop_mask) {
		dev_info(priv->dev,
			 "[DES-STREAM] no matching active streams to disable\n");
		return 0;
	}

	/*
	 * Only assert camera reset for streams being disabled.
	 * Do not touch MAX96724 link / pipe / PHY registers.
	 */
	ret = max_des_set_camera_stream_mask(priv, stop_mask, false);
	if (ret)
		return ret;

	priv->active_streams_mask = new_mask;
	priv->requested_streams_mask = new_mask;

	if (!new_mask) {
		des->active = false;

		if (pm_runtime_enabled(&priv->client->dev))
			pm_runtime_put(&priv->client->dev);

		dev_info(priv->dev,
			 "[DES-STREAM] all streams disabled, camera OFF, DES hw path kept enabled\n");
		return 0;
	}

	dev_info(priv->dev,
		 "[DES-STREAM] partial disable done: stop_mask=0x%llx active_streams_mask=0x%llx\n",
		 (unsigned long long)stop_mask,
		 (unsigned long long)priv->active_streams_mask);

	return 0;
}

static int max_des_log_status(struct v4l2_subdev *sd)
{
	struct max_des_priv *priv = v4l2_get_subdevdata(sd);
	struct max_des *des = priv->des;

	if (des->ops->log_status)
		des->ops->log_status(des);

	return 0;
}

static const struct v4l2_subdev_core_ops max_des_core_ops = { .log_status = max_des_log_status };
static const struct v4l2_subdev_pad_ops max_des_pad_ops = {
	.enable_streams = max_des_enable_streams,
	.disable_streams = max_des_disable_streams,
	.set_routing = max_des_set_routing,
	.get_frame_desc = max_des_get_frame_desc,
	.get_mbus_config = max_des_get_mbus_config,
	.get_fmt = max_des_get_fmt,
	.set_fmt = max_des_set_fmt,
};
static const struct v4l2_subdev_ops max_des_subdev_ops = {
	.core = &max_des_core_ops,
	.pad = &max_des_pad_ops,
};
static const struct v4l2_subdev_internal_ops max_des_internal_ops = {
	.init_state = &max_des_init_state,
};

static const struct media_entity_operations max_des_media_ops = {
	/* 回歸 V4L2 標準 1_to_1 對接：ACPI Port 0 自動對接 V4L2 Pad 0 */
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
	.has_pad_interdep = v4l2_subdev_has_pad_interdep,
	.link_validate = v4l2_subdev_link_validate,
};


static int max_des_v4l2_register(struct max_des_priv *priv)
{
	struct v4l2_subdev *sd = &priv->sd;
	struct v4l2_ctrl *ctrl;
	int ret;

	dev_info(priv->dev, "[DES-PROBE] 4. Initializing V4L2 Subdev and Media Pads...\n");

	v4l2_i2c_subdev_init(sd, priv->client, &max_des_subdev_ops);
	snprintf(sd->name, sizeof(sd->name), "max96724");
	sd->internal_ops = &max_des_internal_ops;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &max_des_media_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_STREAMS;
	priv->pads = devm_kcalloc(priv->dev, DES_NUM_PADS, sizeof(*priv->pads), GFP_KERNEL);
	if (!priv->pads) return -ENOMEM;

	/* 套用極簡 Pad 架構 */
	priv->pads[DES_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	priv->pads[DES_PAD_SINK].flags = MEDIA_PAD_FL_SINK;

	v4l2_set_subdevdata(sd, priv);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, 3);
	priv->sd.ctrl_handler = &priv->ctrl_handler;

	ctrl = v4l2_ctrl_new_std(&priv->ctrl_handler, NULL, V4L2_CID_PIXEL_RATE, V4L2_FREQ, V4L2_FREQ, 1, V4L2_FREQ);
	if (ctrl) ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	static const s64 link_freq_menu_items[] = { MAX_DES_LINK_FREQUENCY_DEFAULT }; 
	ctrl = v4l2_ctrl_new_int_menu(&priv->ctrl_handler, NULL, V4L2_CID_LINK_FREQ, 0, 0, link_freq_menu_items);
	if (ctrl) ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (priv->ctrl_handler.error) {
		dev_err(priv->dev, "[DES-PROBE] Ctrl handler error: %d\n", priv->ctrl_handler.error);
		return priv->ctrl_handler.error;
	}

	ret = media_entity_pads_init(&sd->entity, DES_NUM_PADS, priv->pads);
	if (ret) return ret;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret) return ret;

	dev_info(priv->dev,
	 "[DES-DBG] v4l2_register: HAS_DEVNODE=1 FL_STREAMS=1 name=%s\n",
	 sd->name);

	return v4l2_async_register_subdev(sd);
}

int max_des_probe(struct i2c_client *client, struct max_des *des)
{
	struct max_des_priv *priv;
	int ret, i;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	priv->dev = &client->dev;
	priv->des = des;
	des->priv = priv;
	mutex_init(&priv->lock);
	INIT_LIST_HEAD(&priv->hotplug_node);

	dev_info(priv->dev, "\n============================================\n");
	dev_info(priv->dev, "[DES-PROBE] 1. Allocating driver memory...\n");

	des->phys = devm_kcalloc(priv->dev, des->ops->num_phys,
				 sizeof(*des->phys), GFP_KERNEL);
	des->pipes = devm_kcalloc(priv->dev, des->ops->num_pipes,
				  sizeof(*des->pipes), GFP_KERNEL);
	des->links = devm_kcalloc(priv->dev, des->ops->num_links,
				  sizeof(*des->links), GFP_KERNEL);

	if (!des->phys || !des->pipes || !des->links)
		return -ENOMEM;

	for (i = 0; i < des->ops->num_phys; i++) {
		des->phys[i].index = i;
		des->phys[i].bus_type = V4L2_MBUS_CSI2_DPHY;
		des->phys[i].mipi.num_data_lanes = 4;
		des->phys[i].mipi.clock_lane = 0;
		des->phys[i].mipi.data_lanes[0] = 1;
		des->phys[i].mipi.data_lanes[1] = 2;
		des->phys[i].mipi.data_lanes[2] = 3;
		des->phys[i].mipi.data_lanes[3] = 4;
		des->phys[i].link_frequency = MAX_DES_LINK_FREQUENCY_DEFAULT;
	}

	for (i = 0; i < des->ops->num_pipes; i++)
		des->pipes[i].index = i;

	for (i = 0; i < des->ops->num_links; i++) {
		des->links[i].index = i;
		des->links[i].enabled = false;
		des->links[i].cam = NULL;
	}

	dev_info(priv->dev,
		 "[DES-PROBE] 2. Invoking Hardware Init (GMSL Link Wakeup), TEST_LINK_ID=%u...\n",
		 TEST_LINK_ID);

	if (des->ops->init) {
		ret = des->ops->init(des);
		if (ret) {
			dev_err(priv->dev,
				"[DES-PROBE] des hardware init failed: %d\n",
				ret);
			return ret;
		}
	}

	/*
	 * Detect each camera profile before serializer re-addressing and before
	 * V4L2 formats are created.
	 */
	ret = max_des_scan_func(priv);
	if (ret) {
		dev_err(priv->dev,
			"[DES-PROBE] camera scan failed: %d\n", ret);
		return ret;
	}

	ret = max_des_i2c_mux_init(priv);
	if (ret)
		return ret;

	priv->detected_links_mask = 0;
	for (i = 0; i < des->ops->num_links; i++) {
		if (des->links[i].enabled)
			priv->detected_links_mask |= BIT(i);
	}

	priv->requested_streams_mask = 0;
	priv->active_streams_mask = 0;

	dev_info(priv->dev,
		 "[DES-PROBE] detected_links_mask=0x%x\n",
		 priv->detected_links_mask);

	ret = max_des_v4l2_register(priv);
	if (ret)
		return ret;

	mutex_lock(&max_des_instances_lock);
	list_add_tail(&priv->hotplug_node, &max_des_instances);
	mutex_unlock(&max_des_instances_lock);

	dev_info(priv->dev,
		 "[DES-HOTPLUG] instance registered for manual rescan\n");

	dev_info(priv->dev, "[DES-PROBE] Probe complete! Status: %d\n", ret);
	dev_info(priv->dev, "============================================\n\n");
	return ret;
}
EXPORT_SYMBOL_NS_GPL(max_des_probe, "MAX_SERDES");

int max_des_remove(struct max_des *des)
{
	struct max_des_priv *priv = des->priv;

	mutex_lock(&max_des_instances_lock);
	if (!list_empty(&priv->hotplug_node))
		list_del_init(&priv->hotplug_node);
	mutex_unlock(&max_des_instances_lock);

	pr_info("[DES-HOTPLUG] instance unregistered from manual rescan\n");
	dev_info(priv->dev, "[DES] Removing driver...\n");
	v4l2_async_unregister_subdev(&priv->sd);
	v4l2_subdev_cleanup(&priv->sd);
	v4l2_async_nf_unregister(&priv->nf);
	v4l2_async_nf_cleanup(&priv->nf);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	i2c_mux_del_adapters(priv->mux);
	return 0;
}
EXPORT_SYMBOL_NS_GPL(max_des_remove, "MAX_SERDES");


MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("I2C_ATR");