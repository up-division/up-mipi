/* SPDX-License-Identifier: GPL-2.0 */
/*
 * caminfo.h - Camera module profiles for MAX96724 GMSL2 deserializer
 *
 * 目前收錄：
 *   1. SG3S-ISX031C-GMSL2F
 *   2. SG8S-AR0820C-5300-G2A
 *
 * 設計目的：
 *   - 將相機模組差異集中在 profile，不在 max96724.c / max_des.c 內寫死。
 *   - 後續每一條 GMSL link 可透過 const struct cam_profile * 指向不同相機。
 *   - 明確區分 I2C 7-bit address 與 datasheet 常見的 8-bit write address。
 *
 * 注意：
 *   - 相機輸出解析度、GMSL2 forward rate、Serializer/ISP/Sensor 型號屬於
 *     camera profile。
 *   - MAX96724 到 IPU6 的 CSI-2 lane count、DPLL、lane mapping 屬於 DES
 *     輸出埠設定，不應放在單一 camera profile 中。
 */

#ifndef CAMINFO_H
#define CAMINFO_H

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#include <media/v4l2-mediabus.h>

/* CSI-2 standard data type */
#define CAM_CSI2_DT_FRAME_START      0x00
#define CAM_CSI2_DT_FRAME_END        0x01
#define CAM_CSI2_DT_YUV422_8BIT      0x1e

/* MAX96724 GMSL2 receive-rate field value。 */
enum cam_gmsl2_rx_rate {
	CAM_GMSL2_RX_RATE_3GBPS = 0x1,
	CAM_GMSL2_RX_RATE_6GBPS = 0x2,
};

enum cam_profile_id {
	CAM_PROFILE_INVALID = 0,
	CAM_PROFILE_ISX031,
	CAM_PROFILE_AR0820C_GW5300,
};

enum cam_sensor_model {
	CAM_SENSOR_UNKNOWN = 0,
	CAM_SENSOR_SONY_ISX031,
	CAM_SENSOR_ONSEMI_AR0820,
};

enum cam_isp_model {
	CAM_ISP_NONE = 0,
	CAM_ISP_ISX031_BUILT_IN,
	CAM_ISP_GW5300,
};

enum cam_serializer_model {
	CAM_SERIALIZER_UNKNOWN = 0,
	CAM_SERIALIZER_MAX96717F,
	CAM_SERIALIZER_MAX9295A,
};

enum cam_cfa_pattern {
	CAM_CFA_NOT_APPLICABLE = 0,
	CAM_CFA_RGGB,
};

enum cam_output_format {
	CAM_OUTPUT_UNKNOWN = 0,
	CAM_OUTPUT_YUV422_8BIT,
};

enum cam_reset_target {
	CAM_RESET_TARGET_UNKNOWN = 0,
	CAM_RESET_TARGET_SENSOR,
	CAM_RESET_TARGET_ISP,
};

enum cam_fsync_target {
	CAM_FSYNC_TARGET_UNKNOWN = 0,
	CAM_FSYNC_TARGET_SENSOR,
};

/*
 * 同時保存 7-bit 與 8-bit address，避免 0x80 / 0x40 混淆。
 *
 * 例如：
 *   datasheet 8-bit write address 0x80
 *   Linux I2C 7-bit address       0x40
 */
struct cam_i2c_endpoint {
	bool present;
	u8 addr_7bit;
	u8 addr_8bit;
};

struct cam_video_info {
	u32 width;
	u32 height;
	u32 fps_num;
	u32 fps_den;

	/* V4L2 media-bus format */
	u32 mbus_code;

	/* CSI-2 packet information before DES VC remap */
	u8 csi2_data_type;
	u8 source_vc;
	u8 bits_per_pixel;
	u8 bytes_per_pixel;
	u8 camera_csi2_data_lanes;

	enum cam_output_format output_format;
	enum cam_cfa_pattern sensor_cfa;
	bool uncompressed;
};

struct cam_sensor_info {
	enum cam_sensor_model model;
	const char *name;
	struct cam_i2c_endpoint i2c;

	/* 使用整數奈米，避免 kernel floating point。 */
	u32 pixel_width_nm;
	u32 pixel_height_nm;

	/* Datasheet 標示，例如 "1/2.42-inch"。 */
	const char *optical_format;

	bool hdr_supported;
	bool lfm_supported;
};

struct cam_isp_info {
	enum cam_isp_model model;
	const char *name;
	struct cam_i2c_endpoint i2c;
	bool built_into_sensor;
};

struct cam_serializer_stream_ctrl {
	bool supported;
	u16 reg;
	u8 stream_on;
	u8 stream_off;
};


/*
 * Camera scan signature.
 *
 * Serializer DEV_ID/DEV_REV use 16-bit register addresses.
 * EEPROM address is also kept in aux_i2c; detect.eeprom_addr_7bit is a
 * convenient primary signature used by the early scan flow.
 *
 * serializer_dev_id_required:
 *   true  = DEV_ID must match before selecting this profile.
 *   false = DEV_ID is useful diagnostic information, but EEPROM/rate may
 *           still identify the module when the exact ID has not been
 *           verified on local hardware.
 */
struct cam_detect_info {
	u16 serializer_dev_id_reg;
	u16 serializer_dev_rev_reg;

	bool serializer_dev_id_required;
	u8 serializer_dev_id;
	u8 serializer_dev_id_mask;

	bool observed_serializer_dev_rev_valid;
	u8 observed_serializer_dev_rev;

	bool eeprom_required;
	u8 eeprom_addr_7bit;
};

/*
 * 未來若兩種 serializer 需要不同初始化表，可使用此結構。
 * delay_ms == 0 表示寫入後不需要額外延遲。
 */
struct cam_regval {
	u16 reg;
	u8 val;
	u16 delay_ms;
};

struct cam_serializer_info {
	enum cam_serializer_model model;
	const char *name;
	struct cam_i2c_endpoint default_i2c;

	enum cam_gmsl2_rx_rate forward_rate;
	u32 forward_rate_mbps;

	/* Serializer MFP mapping */
	u8 reset_mfp;
	enum cam_reset_target reset_target;
	u8 fsync_mfp;
	enum cam_fsync_target fsync_target;

	struct cam_serializer_stream_ctrl stream_ctrl;
	struct cam_detect_info detect;

	/* 可選的 serializer-specific 初始化表，目前先留空。 */
	const struct cam_regval *init_seq;
	u32 init_seq_num;
};

struct cam_aux_i2c_info {
	bool has_eeprom;
	struct cam_i2c_endpoint eeprom_i2c;
};

struct cam_power_info {
	u32 poc_min_mv;
	u32 poc_max_mv;
	u32 nominal_mv;
	u32 max_current_ma;
};

struct cam_physical_info {
	u32 width_um;
	u32 length_um;
	u32 height_um;
	u32 max_weight_g;
};

/*
 * 完整 camera module profile。
 *
 * 建議後續在 struct max_des_link 中加入：
 *
 *     const struct cam_profile *cam;
 *
 * 使 link0~link3 可以各自綁定不同 profile。
 */
struct cam_profile {
	enum cam_profile_id id;

	/* Profile/產品識別 */
	const char *profile_name;
	const char *module_name;
	const char *vendor;

	struct cam_video_info video;
	struct cam_sensor_info sensor;
	struct cam_isp_info isp;
	struct cam_serializer_info serializer;
	struct cam_aux_i2c_info aux_i2c;
	struct cam_power_info power;
	struct cam_physical_info physical;
};

/*
 * SG3S-ISX031C-GMSL2F
 *   Sensor/ISP : Sony ISX031（ISP built-in）
 *   Output     : 1920x1536 @ 30 fps, YUV422 8-bit
 *   Serializer : MAX96717F
 *   GMSL2      : 3 Gbps
 */
static const struct cam_profile cam_profile_isx031 = {
	.id = CAM_PROFILE_ISX031,
	.profile_name = "isx031",
	.module_name = "SG3S-ISX031C-GMSL2F",
	.vendor = "SZ Sensing TECH",

	.video = {
		.width = 1920,
		.height = 1536,
		.fps_num = 30,
		.fps_den = 1,
		.mbus_code = MEDIA_BUS_FMT_UYVY8_1X16,
		.csi2_data_type = CAM_CSI2_DT_YUV422_8BIT,
		.source_vc = 0,
		.bits_per_pixel = 16,
		.bytes_per_pixel = 2,
		.camera_csi2_data_lanes = 4,
		.output_format = CAM_OUTPUT_YUV422_8BIT,
		.sensor_cfa = CAM_CFA_RGGB,
		.uncompressed = true,
	},

	.sensor = {
		.model = CAM_SENSOR_SONY_ISX031,
		.name = "Sony ISX031",
		.i2c = {
			.present = true,
			.addr_7bit = 0x1a,
			.addr_8bit = 0x34,
		},
		.pixel_width_nm = 3000,
		.pixel_height_nm = 3000,
		.optical_format = "1/2.42-inch",
		.hdr_supported = true,
		.lfm_supported = true,
	},

	.isp = {
		.model = CAM_ISP_ISX031_BUILT_IN,
		.name = "ISX031 built-in ISP",
		.i2c = {
			/* ISP 與 ISX031 整合，不另列獨立 I2C endpoint。 */
			.present = false,
			.addr_7bit = 0x00,
			.addr_8bit = 0x00,
		},
		.built_into_sensor = true,
	},

	.serializer = {
		.model = CAM_SERIALIZER_MAX96717F,
		.name = "MAX96717F",
		.default_i2c = {
			.present = true,
			.addr_7bit = 0x40,
			.addr_8bit = 0x80,
		},
		.forward_rate = CAM_GMSL2_RX_RATE_3GBPS,
		.forward_rate_mbps = 3000,
		.reset_mfp = 0,
		.reset_target = CAM_RESET_TARGET_SENSOR,
		.fsync_mfp = 7,
		.fsync_target = CAM_FSYNC_TARGET_SENSOR,
		.stream_ctrl = {
			.supported = true,
			.reg = 0x02be,
			.stream_on = 0x10,
			.stream_off = 0x00,
		},
		.detect = {
			.serializer_dev_id_reg = 0x000d,
			.serializer_dev_rev_reg = 0x000e,
			/*
			 * MAX96717F is expected to report 0xc8, but the local
			 * ISX031 module has not yet been measured.  Keep EEPROM
			 * 0x50 as the required module signature for now.
			 */
			.serializer_dev_id_required = false,
			.serializer_dev_id = 0xc8,
			.serializer_dev_id_mask = 0xff,
			.observed_serializer_dev_rev_valid = false,
			.observed_serializer_dev_rev = 0x00,
			.eeprom_required = true,
			.eeprom_addr_7bit = 0x50,
		},
		.init_seq = NULL,
		.init_seq_num = 0,
	},

	.aux_i2c = {
		.has_eeprom = true,
		.eeprom_i2c = {
			.present = true,
			.addr_7bit = 0x50,
			.addr_8bit = 0xa0,
		},
	},

	.power = {
		.poc_min_mv = 9000,
		.poc_max_mv = 16000,
		.nominal_mv = 12000,
		.max_current_ma = 200,
	},

	.physical = {
		.width_um = 25000,
		.length_um = 25000,
		.height_um = 18600,
		.max_weight_g = 50,
	},
};

/*
 * SG8S-AR0820C-5300-G2A
 *   Sensor     : onsemi AR0820
 *   ISP        : GW5300
 *   Output     : 3840x2160 @ 30 fps, YUV422 8-bit
 *   Serializer : MAX9295A
 *   GMSL2      : 6 Gbps
 */
static const struct cam_profile cam_profile_ar0820c = {
	.id = CAM_PROFILE_AR0820C_GW5300,
	.profile_name = "ar0820c-gw5300",
	.module_name = "SG8S-AR0820C-5300-G2A",
	.vendor = "SZ Sensing TECH",

	.video = {
		.width = 3840,
		.height = 2160,
		.fps_num = 30,
		.fps_den = 1,
		.mbus_code = MEDIA_BUS_FMT_UYVY8_1X16,
		.csi2_data_type = CAM_CSI2_DT_YUV422_8BIT,
		.source_vc = 0,
		.bits_per_pixel = 16,
		.bytes_per_pixel = 2,
		.camera_csi2_data_lanes = 4,
		.output_format = CAM_OUTPUT_YUV422_8BIT,
		.sensor_cfa = CAM_CFA_RGGB,
		.uncompressed = true,
	},

	.sensor = {
		.model = CAM_SENSOR_ONSEMI_AR0820,
		.name = "onsemi AR0820",
		.i2c = {
			.present = true,
			.addr_7bit = 0x10,
			.addr_8bit = 0x20,
		},
		.pixel_width_nm = 2100,
		.pixel_height_nm = 2100,
		.optical_format = "1/2-inch",
		.hdr_supported = true,
		.lfm_supported = false,
	},

	.isp = {
		.model = CAM_ISP_GW5300,
		.name = "GW5300",
		.i2c = {
			.present = true,
			.addr_7bit = 0x6d,
			.addr_8bit = 0xda,
		},
		.built_into_sensor = false,
	},

	.serializer = {
		.model = CAM_SERIALIZER_MAX9295A,
		.name = "MAX9295A",
		.default_i2c = {
			.present = true,
			.addr_7bit = 0x40,
			.addr_8bit = 0x80,
		},
		.forward_rate = CAM_GMSL2_RX_RATE_6GBPS,
		.forward_rate_mbps = 6000,
		.reset_mfp = 0,
		.reset_target = CAM_RESET_TARGET_ISP,
		.fsync_mfp = 7,
		.fsync_target = CAM_FSYNC_TARGET_SENSOR,
		.stream_ctrl = {
			.supported = true,
			.reg = 0x02be,
			.stream_on = 0x10,
			.stream_off = 0x00,
		},
		.detect = {
			.serializer_dev_id_reg = 0x000d,
			.serializer_dev_rev_reg = 0x000e,
			.serializer_dev_id_required = true,
			.serializer_dev_id = 0x91,
			.serializer_dev_id_mask = 0xff,
			/* Measured locally on the AR0820C/MAX9295A module. */
			.observed_serializer_dev_rev_valid = true,
			.observed_serializer_dev_rev = 0x08,
			.eeprom_required = true,
			.eeprom_addr_7bit = 0x51,
		},
		.init_seq = NULL,
		.init_seq_num = 0,
	},

	.aux_i2c = {
		.has_eeprom = true,
		.eeprom_i2c = {
			.present = true,
			.addr_7bit = 0x51,
			.addr_8bit = 0xa2,
		},
	},

	.power = {
		.poc_min_mv = 9000,
		.poc_max_mv = 16000,
		.nominal_mv = 12000,
		.max_current_ma = 400,
	},

	.physical = {
		.width_um = 30000,
		.length_um = 30000,
		.height_um = 22570,
		.max_weight_g = 80,
	},
};

/*
 * 統一 profile table。
 *
 * 注意：此版本將定義放在 header，方便目前直接導入測試；若未來有多個
 * translation unit 同時使用，可再把實體定義移到 caminfo.c，header 僅保留
 * extern declaration。
 */
static const struct cam_profile * const cam_profiles[] = {
	&cam_profile_isx031,
	&cam_profile_ar0820c,
};

static inline const struct cam_profile *
cam_profile_get_by_id(enum cam_profile_id id)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(cam_profiles); i++) {
		if (cam_profiles[i]->id == id)
			return cam_profiles[i];
	}

	return NULL;
}

static inline const struct cam_profile *
cam_profile_get_by_name(const char *name)
{
	u32 i;

	if (!name)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(cam_profiles); i++) {
		if (!strcmp(cam_profiles[i]->profile_name, name))
			return cam_profiles[i];
	}

	return NULL;
}


static inline const struct cam_profile *
cam_profile_match_scan(enum cam_gmsl2_rx_rate rate,
		       bool serializer_id_valid,
		       u8 serializer_dev_id,
		       bool eeprom_0x50_present,
		       bool eeprom_0x51_present)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(cam_profiles); i++) {
		const struct cam_profile *profile = cam_profiles[i];
		const struct cam_detect_info *detect = &profile->serializer.detect;
		bool id_match = false;
		bool eeprom_match = false;

		if (profile->serializer.forward_rate != rate)
			continue;

		if (serializer_id_valid)
			id_match = ((serializer_dev_id & detect->serializer_dev_id_mask) ==
				    (detect->serializer_dev_id &
				     detect->serializer_dev_id_mask));

		if (detect->eeprom_addr_7bit == 0x50)
			eeprom_match = eeprom_0x50_present;
		else if (detect->eeprom_addr_7bit == 0x51)
			eeprom_match = eeprom_0x51_present;

		if (detect->eeprom_required && !eeprom_match)
			continue;

		if (detect->serializer_dev_id_required && !id_match)
			continue;

		/*
		 * At least one actual signature must be present.  This prevents
		 * an unverified optional DEV_ID from matching an empty link.
		 */
		if (!eeprom_match && !id_match)
			continue;

		return profile;
	}

	return NULL;
}

#endif /* CAMINFO_H */
