// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX219 cameras.
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 *
 * Based on Sony imx258 camera driver
 * Copyright (C) 2018 Intel Corporation
 *
 * DT / fwnode changes, and regulator / GPIO control taken from imx214 driver
 * Copyright 2018 Qtechnology A/S
 *
 * Flip handling taken from the Sony IMX319 driver.
 * Copyright (C) 2018 Intel Corporation
 *
 * Adapted for Rockchip BSP kernel 6.1 by removing CCI dependency and
 * adding Rockchip integration layer (RKMODULE ioctls, DT properties).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/rk-camera-module.h>
#include <linux/compat.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

/* Chip ID */
#define IMX219_REG_CHIP_ID		0x0000
#define IMX219_CHIP_ID			0x0219

#define IMX219_REG_MODE_SELECT		0x0100
#define IMX219_MODE_STANDBY		0x00
#define IMX219_MODE_STREAMING		0x01

#define IMX219_REG_CSI_LANE_MODE	0x0114
#define IMX219_CSI_2_LANE_MODE		0x01
#define IMX219_CSI_4_LANE_MODE		0x03

#define IMX219_REG_DPHY_CTRL		0x0128
#define IMX219_DPHY_CTRL_TIMING_AUTO	0

#define IMX219_REG_EXCK_FREQ_HI	0x012a
#define IMX219_REG_EXCK_FREQ_LO	0x012b

/* Analog gain control */
#define IMX219_REG_ANALOG_GAIN		0x0157
#define IMX219_ANA_GAIN_MIN		0
#define IMX219_ANA_GAIN_MAX		232
#define IMX219_ANA_GAIN_STEP		1
#define IMX219_ANA_GAIN_DEFAULT		0x0

/* Digital gain control */
#define IMX219_REG_DIGITAL_GAIN_HI	0x0158
#define IMX219_REG_DIGITAL_GAIN_LO	0x0159
#define IMX219_DGTL_GAIN_MIN		0x0100
#define IMX219_DGTL_GAIN_MAX		0x0fff
#define IMX219_DGTL_GAIN_DEFAULT	0x0100
#define IMX219_DGTL_GAIN_STEP		1

/* Exposure control */
#define IMX219_REG_EXPOSURE_HI		0x015a
#define IMX219_REG_EXPOSURE_LO		0x015b
#define IMX219_EXPOSURE_MIN		4
#define IMX219_EXPOSURE_STEP		1
#define IMX219_EXPOSURE_DEFAULT		0x640
#define IMX219_EXPOSURE_MAX		65535

/* V_TIMING internal */
#define IMX219_REG_VTS_HI		0x0160
#define IMX219_REG_VTS_LO		0x0161
#define IMX219_VTS_MAX			0xffff

#define IMX219_VBLANK_MIN		32

/* HBLANK control - read only */
#define IMX219_PPL_DEFAULT		3560

#define IMX219_REG_LINE_LENGTH_A_HI	0x0162
#define IMX219_REG_LINE_LENGTH_A_LO	0x0163
#define IMX219_REG_X_ADD_STA_A_HI	0x0164
#define IMX219_REG_X_ADD_STA_A_LO	0x0165
#define IMX219_REG_X_ADD_END_A_HI	0x0166
#define IMX219_REG_X_ADD_END_A_LO	0x0167
#define IMX219_REG_Y_ADD_STA_A_HI	0x0168
#define IMX219_REG_Y_ADD_STA_A_LO	0x0169
#define IMX219_REG_Y_ADD_END_A_HI	0x016a
#define IMX219_REG_Y_ADD_END_A_LO	0x016b
#define IMX219_REG_X_OUTPUT_SIZE_HI	0x016c
#define IMX219_REG_X_OUTPUT_SIZE_LO	0x016d
#define IMX219_REG_Y_OUTPUT_SIZE_HI	0x016e
#define IMX219_REG_Y_OUTPUT_SIZE_LO	0x016f
#define IMX219_REG_X_ODD_INC_A		0x0170
#define IMX219_REG_Y_ODD_INC_A		0x0171
#define IMX219_REG_ORIENTATION		0x0172

/* Binning Mode */
#define IMX219_REG_BINNING_MODE_H	0x0174
#define IMX219_REG_BINNING_MODE_V	0x0175
#define IMX219_REG_BINNING_CAL_MODE_H	0x0176
#define IMX219_REG_BINNING_CAL_MODE_V	0x0177
#define IMX219_BINNING_NONE		0x00
#define IMX219_BINNING_X2		0x01
#define IMX219_BINNING_X2_ANALOG	0x03
#define IMX219_BINNING_CAL_MODE_AVERAGE	0x00
#define IMX219_BINNING_CAL_MODE_SUM	0x01

#define IMX219_REG_CSI_DATA_FORMAT_A_HI	0x018c
#define IMX219_REG_CSI_DATA_FORMAT_A_LO	0x018d

/* PLL Settings */
#define IMX219_REG_VTPXCK_DIV		0x0301
#define IMX219_REG_VTSYCK_DIV		0x0303
#define IMX219_REG_PREPLLCK_VT_DIV	0x0304
#define IMX219_REG_PREPLLCK_OP_DIV	0x0305
#define IMX219_REG_PLL_VT_MPY_HI	0x0306
#define IMX219_REG_PLL_VT_MPY_LO	0x0307
#define IMX219_REG_OPPXCK_DIV		0x0309
#define IMX219_REG_OPSYCK_DIV		0x030b
#define IMX219_REG_PLL_OP_MPY_HI	0x030c
#define IMX219_REG_PLL_OP_MPY_LO	0x030d

/* Test Pattern Control */
#define IMX219_REG_TEST_PATTERN_HI	0x0600
#define IMX219_REG_TEST_PATTERN_LO	0x0601
#define IMX219_TEST_PATTERN_DISABLE	0
#define IMX219_TEST_PATTERN_SOLID_COLOR	1
#define IMX219_TEST_PATTERN_COLOR_BARS	2
#define IMX219_TEST_PATTERN_GREY_COLOR	3
#define IMX219_TEST_PATTERN_PN9		4

/* Test pattern colour components */
#define IMX219_REG_TESTP_RED_HI		0x0602
#define IMX219_REG_TESTP_RED_LO		0x0603
#define IMX219_REG_TESTP_GREENR_HI	0x0604
#define IMX219_REG_TESTP_GREENR_LO	0x0605
#define IMX219_REG_TESTP_BLUE_HI	0x0606
#define IMX219_REG_TESTP_BLUE_LO	0x0607
#define IMX219_REG_TESTP_GREENB_HI	0x0608
#define IMX219_REG_TESTP_GREENB_LO	0x0609
#define IMX219_TESTP_COLOUR_MIN		0
#define IMX219_TESTP_COLOUR_MAX		0x03ff
#define IMX219_TESTP_COLOUR_STEP	1

#define IMX219_REG_TP_WINDOW_WIDTH_HI	0x0624
#define IMX219_REG_TP_WINDOW_WIDTH_LO	0x0625
#define IMX219_REG_TP_WINDOW_HEIGHT_HI	0x0626
#define IMX219_REG_TP_WINDOW_HEIGHT_LO	0x0627

/* External clock frequency is 24.0M */
#define IMX219_XCLK_FREQ		24000000

/* Pixel rate is fixed for all the modes */
#define IMX219_PIXEL_RATE		182400000
#define IMX219_PIXEL_RATE_4LANE		281600000

#define IMX219_DEFAULT_LINK_FREQ	456000000
#define IMX219_DEFAULT_LINK_FREQ_4LANE	364000000

#define IMX219_NAME			"imx219"

/* IMX219 native and active pixel array size. */
#define IMX219_NATIVE_WIDTH		3296U
#define IMX219_NATIVE_HEIGHT		2480U
#define IMX219_PIXEL_ARRAY_LEFT		8U
#define IMX219_PIXEL_ARRAY_TOP		8U
#define IMX219_PIXEL_ARRAY_WIDTH	3280U
#define IMX219_PIXEL_ARRAY_HEIGHT	2464U

enum binning_bit_depths {
	BINNING_IDX_8_BIT,
	BINNING_IDX_10_BIT,
	BINNING_IDX_MAX
};

struct imx219_reg {
	u16 address;
	u8 value;
};

/* Mode : resolution and related config&values */
struct imx219_mode {
	/* Frame width */
	unsigned int width;
	/* Frame height */
	unsigned int height;

	/* V-timing */
	unsigned int vts_def;

	/* binning mode based on format code */
	unsigned int binning[BINNING_IDX_MAX];
};

/* Register access helpers using regmap */
static int imx219_read_reg(struct regmap *regmap, u16 reg, u8 *val)
{
	unsigned int regval;
	int ret;

	ret = regmap_read(regmap, reg, &regval);
	if (ret)
		return ret;

	*val = regval & 0xff;
	return 0;
}

static int imx219_write_reg(struct regmap *regmap, u16 reg, u8 val)
{
	return regmap_write(regmap, reg, val);
}

static int imx219_write_reg16(struct regmap *regmap, u16 reg, u16 val)
{
	int ret;

	ret = regmap_write(regmap, reg, (val >> 8) & 0xff);
	if (ret)
		return ret;

	return regmap_write(regmap, reg + 1, val & 0xff);
}

static int imx219_read_reg16(struct regmap *regmap, u16 reg, u16 *val)
{
	u8 hi, lo;
	int ret;

	ret = imx219_read_reg(regmap, reg, &hi);
	if (ret)
		return ret;

	ret = imx219_read_reg(regmap, reg + 1, &lo);
	if (ret)
		return ret;

	*val = ((u16)hi << 8) | lo;
	return 0;
}

static int imx219_write_regs(struct regmap *regmap,
			     const struct imx219_reg *regs, unsigned int count)
{
	unsigned int i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = imx219_write_reg(regmap, regs[i].address, regs[i].value);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Common register init sequence shared across all modes.
 * Ported from RPi driver's imx219_common_regs[].
 */
static const struct imx219_reg imx219_common_regs[] = {
	{ 0x0100, 0x00 },	/* Mode Select: standby */

	/* To Access Addresses 3000-5fff, send the following commands */
	{ 0x30eb, 0x05 },
	{ 0x30eb, 0x0c },
	{ 0x300a, 0xff },
	{ 0x300b, 0xff },
	{ 0x30eb, 0x05 },
	{ 0x30eb, 0x09 },

	/* Undocumented registers */
	{ 0x455e, 0x00 },
	{ 0x471e, 0x4b },
	{ 0x4767, 0x0f },
	{ 0x4750, 0x14 },
	{ 0x4540, 0x00 },
	{ 0x47b4, 0x14 },
	{ 0x4713, 0x30 },
	{ 0x478b, 0x10 },
	{ 0x478f, 0x10 },
	{ 0x4793, 0x10 },
	{ 0x4797, 0x0e },
	{ 0x479b, 0x0e },

	/* Frame Bank Register Group "A" */
	{ 0x0162, 0x0d },	/* Line length A [15:8] = 3560 */
	{ 0x0163, 0xe8 },	/* Line length A [7:0] */
	{ 0x0170, 0x01 },	/* X odd inc A */
	{ 0x0171, 0x01 },	/* Y odd inc A */

	/* Output setup registers */
	{ 0x0128, 0x00 },	/* DPHY_CTRL: auto timing */
	{ 0x012a, 0x18 },	/* EXCK_FREQ [15:8] = 24MHz */
	{ 0x012b, 0x00 },	/* EXCK_FREQ [7:0] */
};

/* 2-lane CSI mode PLL settings */
static const struct imx219_reg imx219_2lane_regs[] = {
	{ 0x0301, 0x05 },	/* VTPXCK_DIV */
	{ 0x0303, 0x01 },	/* VTSYCK_DIV */
	{ 0x0304, 0x03 },	/* PREPLLCK_VT_DIV (AUTO) */
	{ 0x0305, 0x03 },	/* PREPLLCK_OP_DIV (AUTO) */
	{ 0x0306, 0x00 },	/* PLL_VT_MPY [15:8] = 48 */
	{ 0x0307, 0x30 },	/* PLL_VT_MPY [7:0] */
	{ 0x030b, 0x01 },	/* OPSYCK_DIV */
	{ 0x030c, 0x00 },	/* PLL_OP_MPY [15:8] = 96 */
	{ 0x030d, 0x60 },	/* PLL_OP_MPY [7:0] */
	{ 0x0114, 0x01 },	/* CSI 2-lane mode */
};

/* 4-lane CSI mode PLL settings */
static const struct imx219_reg imx219_4lane_regs[] = {
	{ 0x0301, 0x05 },	/* VTPXCK_DIV */
	{ 0x0303, 0x01 },	/* VTSYCK_DIV */
	{ 0x0304, 0x03 },	/* PREPLLCK_VT_DIV (AUTO) */
	{ 0x0305, 0x03 },	/* PREPLLCK_OP_DIV (AUTO) */
	{ 0x0306, 0x00 },	/* PLL_VT_MPY [15:8] = 88 */
	{ 0x0307, 0x58 },	/* PLL_VT_MPY [7:0] */
	{ 0x030b, 0x01 },	/* OPSYCK_DIV */
	{ 0x030c, 0x00 },	/* PLL_OP_MPY [15:8] = 91 */
	{ 0x030d, 0x5b },	/* PLL_OP_MPY [7:0] */
	{ 0x0114, 0x03 },	/* CSI 4-lane mode */
};

static const s64 imx219_link_freq_menu[] = {
	IMX219_DEFAULT_LINK_FREQ,
};

static const s64 imx219_link_freq_4lane_menu[] = {
	IMX219_DEFAULT_LINK_FREQ_4LANE,
};

static const char * const imx219_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Solid Color",
	"Grey Color Bars",
	"PN9"
};

static const int imx219_test_pattern_val[] = {
	IMX219_TEST_PATTERN_DISABLE,
	IMX219_TEST_PATTERN_COLOR_BARS,
	IMX219_TEST_PATTERN_SOLID_COLOR,
	IMX219_TEST_PATTERN_GREY_COLOR,
	IMX219_TEST_PATTERN_PN9,
};

/* regulator supplies */
static const char * const imx219_supply_name[] = {
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.8V) supply */
	"VDDL",  /* IF (1.2V) supply */
};

#define IMX219_NUM_SUPPLIES ARRAY_SIZE(imx219_supply_name)

/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 imx219_mbus_formats[] = {
	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SBGGR8_1X8,
};

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby), must be not less than:
 *   t4 + max(t5, t6 + <time to initialize the sensor register over I2C>)
 * For any acceptable external clock t6 < t5, so (t4 + t5) = 6200 uS is safe.
 */
#define IMX219_XCLR_MIN_DELAY_US	6200
#define IMX219_XCLR_DELAY_RANGE_US	1000

/* Mode configs - ported from RPi driver */
static const struct imx219_mode supported_modes[] = {
	{
		/* Focused debug mode: 2x2 binned 1640x1232 */
		.width = 1640,
		.height = 1232,
		.vts_def = 1438,
		.binning = {
			[BINNING_IDX_8_BIT] = IMX219_BINNING_X2_ANALOG,
			[BINNING_IDX_10_BIT] = IMX219_BINNING_X2,
		},
	},
};

struct imx219 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct regmap *regmap;
	struct clk *xclk;
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX219_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode and format */
	const struct imx219_mode *cur_mode;
	struct v4l2_mbus_framefmt cur_fmt;
	struct v4l2_rect cur_crop;

	/* Two or Four lanes */
	u8 lanes;

	/* Streaming state */
	bool streaming;

	/* Rockchip module info */
	u32 module_index;
	const char *module_facing;
	const char *module_name;
	const char *len_name;

	struct mutex lock; /* serialize access */
};

static inline struct imx219 *to_imx219(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx219, sd);
}

/* Get bayer order based on flip setting. */
static u32 imx219_get_format_code(struct imx219 *imx219, u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(imx219_mbus_formats); i++)
		if (imx219_mbus_formats[i] == code)
			break;

	if (i >= ARRAY_SIZE(imx219_mbus_formats))
		i = 0;

	i = (i & ~3) | (imx219->vflip && imx219->vflip->val ? 2 : 0) |
	    (imx219->hflip && imx219->hflip->val ? 1 : 0);

	return imx219_mbus_formats[i];
}

static u32 imx219_get_format_bpp(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
		return 8;

	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	default:
		return 10;
	}
}

static unsigned int imx219_get_binning_mode(struct imx219 *imx219,
					    u8 *bin_h, u8 *bin_v)
{
	const struct imx219_mode *mode = imx219->cur_mode;
	unsigned int bin_mode = IMX219_BINNING_NONE;
	u32 bpp = imx219_get_format_bpp(imx219->cur_fmt.code);

	switch (bpp) {
	case 8:
		bin_mode = mode->binning[BINNING_IDX_8_BIT];
		break;
	case 10:
		bin_mode = mode->binning[BINNING_IDX_10_BIT];
		break;
	}

	*bin_h = imx219->cur_crop.width / imx219->cur_fmt.width;
	*bin_v = imx219->cur_crop.height / imx219->cur_fmt.height;

	if (*bin_h == 2 && *bin_v == 2)
		return bin_mode;
	else if (*bin_h == 2 || *bin_v == 2)
		return IMX219_BINNING_X2;
	else
		return IMX219_BINNING_NONE;
}

static inline u32 imx219_get_rate_factor(struct imx219 *imx219)
{
	u8 bin_h, bin_v;
	unsigned int binning = imx219_get_binning_mode(imx219, &bin_h, &bin_v);

	if (binning == IMX219_BINNING_X2_ANALOG)
		return 2;

	return 1;
}

/* -----------------------------------------------------------------------------
 * Controls
 */

static int imx219_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx219 *imx219 =
		container_of(ctrl->handler, struct imx219, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
	u32 rate_factor;
	int ret = 0;

	rate_factor = imx219_get_rate_factor(imx219);

	if (ctrl->id == V4L2_CID_VBLANK) {
		int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
		exposure_max = imx219->cur_fmt.height + ctrl->val - 4;
		exposure_def = (exposure_max < IMX219_EXPOSURE_DEFAULT) ?
			exposure_max : IMX219_EXPOSURE_DEFAULT;
		__v4l2_ctrl_modify_range(imx219->exposure,
					 imx219->exposure->minimum,
					 exposure_max, imx219->exposure->step,
					 exposure_def);
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx219_write_reg(imx219->regmap, IMX219_REG_ANALOG_GAIN,
				       ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx219_write_reg16(imx219->regmap, IMX219_REG_EXPOSURE_HI,
					 ctrl->val / rate_factor);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx219_write_reg16(imx219->regmap, IMX219_REG_DIGITAL_GAIN_HI,
					 ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx219_write_reg16(imx219->regmap, IMX219_REG_TEST_PATTERN_HI,
					 imx219_test_pattern_val[ctrl->val]);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = imx219_write_reg(imx219->regmap, IMX219_REG_ORIENTATION,
				       imx219->hflip->val | imx219->vflip->val << 1);
		break;
	case V4L2_CID_VBLANK: {
		u16 vts = (imx219->cur_fmt.height + ctrl->val) / rate_factor;

		ret = imx219_write_reg16(imx219->regmap, IMX219_REG_VTS_HI, vts);
		break;
	}
	case V4L2_CID_TEST_PATTERN_RED:
		ret = imx219_write_reg16(imx219->regmap, IMX219_REG_TESTP_RED_HI,
					 ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = imx219_write_reg16(imx219->regmap, IMX219_REG_TESTP_GREENR_HI,
					 ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = imx219_write_reg16(imx219->regmap, IMX219_REG_TESTP_BLUE_HI,
					 ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = imx219_write_reg16(imx219->regmap, IMX219_REG_TESTP_GREENB_HI,
					 ctrl->val);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx219_ctrl_ops = {
	.s_ctrl = imx219_set_ctrl,
};

static unsigned long imx219_get_pixel_rate(struct imx219 *imx219)
{
	return (imx219->lanes == 2) ? IMX219_PIXEL_RATE : IMX219_PIXEL_RATE_4LANE;
}

/* Initialize control handlers */
static int imx219_init_controls(struct imx219 *imx219)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
	const struct imx219_mode *mode = &supported_modes[0];
	struct v4l2_ctrl_handler *ctrl_hdlr;
	int exposure_max, exposure_def, hblank;
	int i, ret;

	ctrl_hdlr = &imx219->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 12);
	if (ret)
		return ret;

	/* By default, PIXEL_RATE is read only */
	imx219->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       imx219_get_pixel_rate(imx219),
					       imx219_get_pixel_rate(imx219), 1,
					       imx219_get_pixel_rate(imx219));

	imx219->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx219_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       0, 0,
				       (imx219->lanes == 2) ?
				       imx219_link_freq_menu :
				       imx219_link_freq_4lane_menu);
	if (imx219->link_freq)
		imx219->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Initial vblank/hblank/exposure parameters based on current mode */
	imx219->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops,
					   V4L2_CID_VBLANK, IMX219_VBLANK_MIN,
					   IMX219_VTS_MAX - mode->height, 1,
					   mode->vts_def - mode->height);
	hblank = IMX219_PPL_DEFAULT - mode->width;
	imx219->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank,
					   1, hblank);
	if (imx219->hblank)
		imx219->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	exposure_max = mode->vts_def - 4;
	exposure_def = (exposure_max < IMX219_EXPOSURE_DEFAULT) ?
		exposure_max : IMX219_EXPOSURE_DEFAULT;
	imx219->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX219_EXPOSURE_MIN, exposure_max,
					     IMX219_EXPOSURE_STEP,
					     exposure_def);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX219_ANA_GAIN_MIN, IMX219_ANA_GAIN_MAX,
			  IMX219_ANA_GAIN_STEP, IMX219_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX219_DGTL_GAIN_MIN, IMX219_DGTL_GAIN_MAX,
			  IMX219_DGTL_GAIN_STEP, IMX219_DGTL_GAIN_DEFAULT);

	imx219->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx219->hflip)
		imx219->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx219->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx219->vflip)
		imx219->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx219_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx219_test_pattern_menu) - 1,
				     0, 0, imx219_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(ctrl_hdlr, &imx219_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  IMX219_TESTP_COLOUR_MIN,
				  IMX219_TESTP_COLOUR_MAX,
				  IMX219_TESTP_COLOUR_STEP,
				  IMX219_TESTP_COLOUR_MAX);
	}

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "Control init failed (%d)\n", ret);
		goto error;
	}

	imx219->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static void imx219_free_controls(struct imx219 *imx219)
{
	v4l2_ctrl_handler_free(imx219->sd.ctrl_handler);
}

/* -----------------------------------------------------------------------------
 * Subdev operations
 */

static int imx219_set_framefmt(struct imx219 *imx219)
{
	unsigned int binning;
	u8 bin_h, bin_v;
	u32 bpp;
	int ret = 0;
	u16 val;

	bpp = imx219_get_format_bpp(imx219->cur_fmt.code);

	val = imx219->cur_crop.left - IMX219_PIXEL_ARRAY_LEFT;
	ret = imx219_write_reg16(imx219->regmap, IMX219_REG_X_ADD_STA_A_HI, val);
	if (ret)
		return ret;

	val = imx219->cur_crop.left - IMX219_PIXEL_ARRAY_LEFT +
	      imx219->cur_crop.width - 1;
	ret = imx219_write_reg16(imx219->regmap, IMX219_REG_X_ADD_END_A_HI, val);
	if (ret)
		return ret;

	val = imx219->cur_crop.top - IMX219_PIXEL_ARRAY_TOP;
	ret = imx219_write_reg16(imx219->regmap, IMX219_REG_Y_ADD_STA_A_HI, val);
	if (ret)
		return ret;

	val = imx219->cur_crop.top - IMX219_PIXEL_ARRAY_TOP +
	      imx219->cur_crop.height - 1;
	ret = imx219_write_reg16(imx219->regmap, IMX219_REG_Y_ADD_END_A_HI, val);
	if (ret)
		return ret;

	binning = imx219_get_binning_mode(imx219, &bin_h, &bin_v);
	ret = imx219_write_reg(imx219->regmap, IMX219_REG_BINNING_MODE_H,
			       (bin_h == 2) ? binning : IMX219_BINNING_NONE);
	if (ret)
		return ret;
	ret = imx219_write_reg(imx219->regmap, IMX219_REG_BINNING_MODE_V,
			       (bin_v == 2) ? binning : IMX219_BINNING_NONE);
	if (ret)
		return ret;

	/*
	 * The original Rockchip mode tables programmed BINNING_CAL_MODE for
	 * all video-sized outputs. Keep that behavior in the dynamic path to
	 * avoid leaving these mode-dependent registers at reset defaults.
	 */
	val = (imx219->cur_fmt.width == IMX219_PIXEL_ARRAY_WIDTH &&
	       imx219->cur_fmt.height == IMX219_PIXEL_ARRAY_HEIGHT) ?
		IMX219_BINNING_CAL_MODE_AVERAGE :
		IMX219_BINNING_CAL_MODE_SUM;
	ret = imx219_write_reg(imx219->regmap, IMX219_REG_BINNING_CAL_MODE_H,
			       val);
	if (ret)
		return ret;
	ret = imx219_write_reg(imx219->regmap, IMX219_REG_BINNING_CAL_MODE_V,
			       val);
	if (ret)
		return ret;

	ret = imx219_write_reg16(imx219->regmap, IMX219_REG_X_OUTPUT_SIZE_HI,
				 imx219->cur_fmt.width);
	if (ret)
		return ret;
	ret = imx219_write_reg16(imx219->regmap, IMX219_REG_Y_OUTPUT_SIZE_HI,
				 imx219->cur_fmt.height);
	if (ret)
		return ret;

	ret = imx219_write_reg16(imx219->regmap, IMX219_REG_TP_WINDOW_WIDTH_HI,
				 imx219->cur_fmt.width);
	if (ret)
		return ret;
	ret = imx219_write_reg16(imx219->regmap, IMX219_REG_TP_WINDOW_HEIGHT_HI,
				 imx219->cur_fmt.height);
	if (ret)
		return ret;

	ret = imx219_write_reg(imx219->regmap, IMX219_REG_CSI_DATA_FORMAT_A_HI,
			       bpp);
	if (ret)
		return ret;
	ret = imx219_write_reg(imx219->regmap, IMX219_REG_CSI_DATA_FORMAT_A_LO,
			       bpp);
	if (ret)
		return ret;

	ret = imx219_write_reg(imx219->regmap, IMX219_REG_OPPXCK_DIV, bpp);

	return ret;
}

static int imx219_configure_lanes(struct imx219 *imx219)
{
	const struct imx219_reg *regs;
	unsigned int count;

	if (imx219->lanes == 2) {
		regs = imx219_2lane_regs;
		count = ARRAY_SIZE(imx219_2lane_regs);
	} else {
		regs = imx219_4lane_regs;
		count = ARRAY_SIZE(imx219_4lane_regs);
	}

	return imx219_write_regs(imx219->regmap, regs, count);
}

static int imx219_start_streaming(struct imx219 *imx219)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
	int ret;

	ret = pm_runtime_resume_and_get(&client->dev);
	if (ret < 0)
		return ret;

	/* Send all registers that are common to all modes */
	ret = imx219_write_regs(imx219->regmap, imx219_common_regs,
				ARRAY_SIZE(imx219_common_regs));
	if (ret) {
		dev_err(&client->dev, "%s failed to send common regs\n", __func__);
		goto err_rpm_put;
	}

	/* Configure two or four Lane mode */
	ret = imx219_configure_lanes(imx219);
	if (ret) {
		dev_err(&client->dev, "%s failed to configure lanes\n", __func__);
		goto err_rpm_put;
	}

	/* Apply format and crop settings */
	ret = imx219_set_framefmt(imx219);
	if (ret) {
		dev_err(&client->dev, "%s failed to set frame format: %d\n",
			__func__, ret);
		goto err_rpm_put;
	}

	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(imx219->sd.ctrl_handler);
	if (ret)
		goto err_rpm_put;

	/* set stream on register */
	ret = imx219_write_reg(imx219->regmap, IMX219_REG_MODE_SELECT,
			       IMX219_MODE_STREAMING);
	if (ret)
		goto err_rpm_put;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx219->vflip, true);
	__v4l2_ctrl_grab(imx219->hflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static void imx219_stop_streaming(struct imx219 *imx219)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
	int ret;

	ret = imx219_write_reg(imx219->regmap, IMX219_REG_MODE_SELECT,
			       IMX219_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);

	__v4l2_ctrl_grab(imx219->vflip, false);
	__v4l2_ctrl_grab(imx219->hflip, false);

	pm_runtime_put(&client->dev);
}

static int imx219_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx219 *imx219 = to_imx219(sd);
	int ret = 0;

	mutex_lock(&imx219->lock);

	if (enable)
		ret = imx219_start_streaming(imx219);
	else
		imx219_stop_streaming(imx219);

	if (!ret)
		imx219->streaming = enable;

	mutex_unlock(&imx219->lock);
	return ret;
}

static void imx219_update_pad_format(struct imx219 *imx219,
				     const struct imx219_mode *mode,
				     struct v4l2_mbus_framefmt *fmt, u32 code)
{
	fmt->code = imx219_get_format_code(imx219, code);
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int imx219_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx219 *imx219 = to_imx219(sd);

	if (code->index >= (ARRAY_SIZE(imx219_mbus_formats) / 4))
		return -EINVAL;

	code->code = imx219_get_format_code(imx219,
					    imx219_mbus_formats[code->index * 4]);

	return 0;
}

static int imx219_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx219 *imx219 = to_imx219(sd);
	u32 code;

	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	code = imx219_get_format_code(imx219, fse->code);
	if (fse->code != code)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int imx219_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	struct imx219 *imx219 = to_imx219(sd);
	const struct imx219_mode *m;
	u32 code, bpp, bmode, rf = 1;

	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	code = imx219_get_format_code(imx219, fie->code);
	if (fie->code != code)
		return -EINVAL;

	m = &supported_modes[fie->index];
	fie->width  = m->width;
	fie->height = m->height;

	bpp = imx219_get_format_bpp(fie->code);
	bmode = (bpp == 8) ? m->binning[BINNING_IDX_8_BIT]
			   : m->binning[BINNING_IDX_10_BIT];
	if (bmode == IMX219_BINNING_X2_ANALOG)
		rf = 2;

	fie->interval.numerator   = 1;
	fie->interval.denominator =
		IMX219_PIXEL_RATE * rf / (IMX219_PPL_DEFAULT * m->vts_def);

	return 0;
}

static int imx219_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx219 *imx219 = to_imx219(sd);
	const struct imx219_mode *mode;
	unsigned int bin_h, bin_v, binning;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	imx219_update_pad_format(imx219, mode, &fmt->format, fmt->format.code);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_get_pad_format(sd, state, 0) = fmt->format;
		return 0;
	}

	imx219->cur_mode = mode;
	imx219->cur_fmt = fmt->format;

	/*
	 * Use binning to maximize the crop rectangle size, and centre it in the
	 * sensor.
	 */
	bin_h = min(IMX219_PIXEL_ARRAY_WIDTH / fmt->format.width, 2U);
	bin_v = min(IMX219_PIXEL_ARRAY_HEIGHT / fmt->format.height, 2U);
	binning = min(bin_h, bin_v);

	imx219->cur_crop.width = fmt->format.width * binning;
	imx219->cur_crop.height = fmt->format.height * binning;
	imx219->cur_crop.left = (IMX219_NATIVE_WIDTH - imx219->cur_crop.width) / 2;
	imx219->cur_crop.top = (IMX219_NATIVE_HEIGHT - imx219->cur_crop.height) / 2;

	{
		int exposure_max;
		int exposure_def;
		int hblank;
		int pixel_rate;

		/* Update limits and set FPS to default */
		__v4l2_ctrl_modify_range(imx219->vblank, IMX219_VBLANK_MIN,
					 IMX219_VTS_MAX - mode->height, 1,
					 mode->vts_def - mode->height);
		__v4l2_ctrl_s_ctrl(imx219->vblank,
				   mode->vts_def - mode->height);
		/* Update max exposure while meeting expected vblanking */
		exposure_max = mode->vts_def - 4;
		exposure_def = (exposure_max < IMX219_EXPOSURE_DEFAULT) ?
			exposure_max : IMX219_EXPOSURE_DEFAULT;
		__v4l2_ctrl_modify_range(imx219->exposure,
					 imx219->exposure->minimum,
					 exposure_max, imx219->exposure->step,
					 exposure_def);
		/*
		 * Currently PPL is fixed to IMX219_PPL_DEFAULT, so hblank
		 * depends on mode->width only, and is not changeable in any
		 * way other than changing the mode.
		 */
		hblank = IMX219_PPL_DEFAULT - mode->width;
		__v4l2_ctrl_modify_range(imx219->hblank, hblank, hblank, 1,
					 hblank);

		/*
		 * Keep the reported pixel rate aligned with the fixed CSI link
		 * timing. Rockchip BSP receivers use V4L2_CID_PIXEL_RATE to
		 * size the CSI/ISP path, while the IMX219 lane PLL remains
		 * fixed for a given lane configuration.
		 */
		pixel_rate = imx219_get_pixel_rate(imx219);
		__v4l2_ctrl_modify_range(imx219->pixel_rate, pixel_rate,
					 pixel_rate, 1, pixel_rate);
	}

	return 0;
}

static int imx219_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx219 *imx219 = to_imx219(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		fmt->format = *v4l2_subdev_get_pad_format(sd, state, 0);
		return 0;
	}

	fmt->format = imx219->cur_fmt;

	return 0;
}

static int imx219_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	struct imx219 *imx219 = to_imx219(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = imx219->cur_crop;
		return 0;

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = IMX219_NATIVE_WIDTH;
		sel->r.height = IMX219_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = IMX219_PIXEL_ARRAY_TOP;
		sel->r.left = IMX219_PIXEL_ARRAY_LEFT;
		sel->r.width = IMX219_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX219_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

/* -----------------------------------------------------------------------------
 * Rockchip integration
 */

static void imx219_get_module_inf(struct imx219 *imx219,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, IMX219_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, imx219->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, imx219->len_name, sizeof(inf->base.lens));
}

static long imx219_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct imx219 *imx219 = to_imx219(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		imx219_get_module_inf(imx219, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long imx219_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	long ret;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx219_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = imx219_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int imx219_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct imx219 *imx219 = to_imx219(sd);

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->bus.mipi_csi2.num_data_lanes = imx219->lanes;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev ops
 */

static const struct v4l2_subdev_core_ops imx219_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.ioctl = imx219_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx219_compat_ioctl32,
#endif
};

static int imx219_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct imx219 *imx219 = to_imx219(sd);
	const struct imx219_mode *mode = imx219->cur_mode;
	u32 rate_factor = imx219_get_rate_factor(imx219);
	u32 vts = mode->height + imx219->vblank->val;

	fi->interval.numerator   = 1;
	fi->interval.denominator =
		IMX219_PIXEL_RATE * rate_factor / (IMX219_PPL_DEFAULT * vts);

	return 0;
}

static const struct v4l2_subdev_video_ops imx219_video_ops = {
	.s_stream         = imx219_set_stream,
	.g_frame_interval = imx219_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx219_pad_ops = {
	.enum_mbus_code = imx219_enum_mbus_code,
	.get_fmt = imx219_get_pad_format,
	.set_fmt = imx219_set_pad_format,
	.get_selection = imx219_get_selection,
	.enum_frame_size = imx219_enum_frame_size,
	.enum_frame_interval = imx219_enum_frame_interval,
	.get_mbus_config = imx219_g_mbus_config,
};

static const struct v4l2_subdev_ops imx219_subdev_ops = {
	.core = &imx219_core_ops,
	.video = &imx219_video_ops,
	.pad = &imx219_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Power management
 */

static int imx219_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx219 *imx219 = to_imx219(sd);
	int ret;

	ret = regulator_bulk_enable(IMX219_NUM_SUPPLIES, imx219->supplies);
	if (ret) {
		dev_err(dev, "%s: failed to enable regulators\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(imx219->xclk);
	if (ret) {
		dev_err(dev, "%s: failed to enable clock\n", __func__);
		goto reg_off;
	}

	/*
	 * Deassert reset: drive XCLR high to bring sensor out of reset.
	 * With gpiod, the DT GPIO_ACTIVE_LOW flag inverts the physical level:
	 * - If DT says GPIO_ACTIVE_LOW: gpiod_set(0) → physical HIGH → run
	 * - If DT says GPIO_ACTIVE_HIGH: gpiod_set(1) → physical HIGH → run
	 * Rockchip DT overlays typically use GPIO_ACTIVE_LOW for reset pins,
	 * so we set logical 0 here to deassert (physical HIGH).
	 */
	gpiod_set_value_cansleep(imx219->reset_gpio, 0);
	usleep_range(IMX219_XCLR_MIN_DELAY_US,
		     IMX219_XCLR_MIN_DELAY_US + IMX219_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(IMX219_NUM_SUPPLIES, imx219->supplies);

	return ret;
}

static int imx219_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx219 *imx219 = to_imx219(sd);

	/* Assert reset: drive XCLR low to put sensor in reset */
	gpiod_set_value_cansleep(imx219->reset_gpio, 1);
	regulator_bulk_disable(IMX219_NUM_SUPPLIES, imx219->supplies);
	clk_disable_unprepare(imx219->xclk);

	return 0;
}

/* -----------------------------------------------------------------------------
 * Probe & remove
 */

static int imx219_get_regulators(struct imx219 *imx219)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
	unsigned int i;

	for (i = 0; i < IMX219_NUM_SUPPLIES; i++)
		imx219->supplies[i].supply = imx219_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       IMX219_NUM_SUPPLIES,
				       imx219->supplies);
}

static int imx219_identify_module(struct imx219 *imx219)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
	u16 val;
	int ret;

	ret = imx219_read_reg16(imx219->regmap, IMX219_REG_CHIP_ID, &val);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x\n",
			IMX219_CHIP_ID);
		return ret;
	}

	if (val != IMX219_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n",
			IMX219_CHIP_ID, val);
		return -EIO;
	}

	dev_info(&client->dev, "IMX219 chip ID 0x%04x\n", val);

	return 0;
}

static int imx219_check_hwcfg(struct device *dev, struct imx219 *imx219)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg);
	if (ret) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2 &&
	    ep_cfg.bus.mipi_csi2.num_data_lanes != 4) {
		dev_err(dev, "only 2 or 4 data lanes are currently supported\n");
		ret = -EINVAL;
		goto error_out;
	}
	imx219->lanes = ep_cfg.bus.mipi_csi2.num_data_lanes;

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static const struct regmap_config imx219_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xffff,
};

static int imx219_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct imx219 *imx219;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	imx219 = devm_kzalloc(dev, sizeof(*imx219), GFP_KERNEL);
	if (!imx219)
		return -ENOMEM;

	/* Parse Rockchip DT properties */
	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &imx219->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &imx219->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &imx219->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &imx219->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	v4l2_i2c_subdev_init(&imx219->sd, client, &imx219_subdev_ops);

	/* Check the hardware configuration in device tree */
	ret = imx219_check_hwcfg(dev, imx219);
	if (ret)
		return ret;

	/* Initialize regmap for I2C register access */
	imx219->regmap = devm_regmap_init_i2c(client, &imx219_regmap_config);
	if (IS_ERR(imx219->regmap))
		return dev_err_probe(dev, PTR_ERR(imx219->regmap),
				     "failed to initialize regmap\n");

	/* Get system clock (xclk) */
	imx219->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx219->xclk))
		return dev_err_probe(dev, PTR_ERR(imx219->xclk),
				     "failed to get xclk\n");

	imx219->xclk_freq = clk_get_rate(imx219->xclk);
	if (imx219->xclk_freq != IMX219_XCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "xclk frequency not supported: %d Hz\n",
				     imx219->xclk_freq);

	ret = imx219_get_regulators(imx219);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	/* Request optional reset pin, start with reset asserted */
	imx219->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for imx219_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = imx219_power_on(dev);
	if (ret)
		return ret;

	ret = imx219_identify_module(imx219);
	if (ret)
		goto error_power_off;

	/*
	 * Sensor doesn't enter LP-11 state upon power up until and unless
	 * streaming is started, so upon power up switch the modes to:
	 * streaming -> standby
	 */
	ret = imx219_write_reg(imx219->regmap, IMX219_REG_MODE_SELECT,
			       IMX219_MODE_STREAMING);
	if (ret < 0)
		goto error_power_off;

	usleep_range(100, 110);

	ret = imx219_write_reg(imx219->regmap, IMX219_REG_MODE_SELECT,
			       IMX219_MODE_STANDBY);
	if (ret < 0)
		goto error_power_off;

	usleep_range(100, 110);

	mutex_init(&imx219->lock);

	/* Set default mode (before controls, so cur_mode is valid) */
	imx219->cur_mode = &supported_modes[0];

	ret = imx219_init_controls(imx219);
	if (ret)
		goto error_mutex_destroy;

	/* Set default format and crop (after controls, so flip is valid) */
	imx219_update_pad_format(imx219, imx219->cur_mode,
				 &imx219->cur_fmt,
				 MEDIA_BUS_FMT_SRGGB8_1X8);

	imx219->cur_crop.left = IMX219_PIXEL_ARRAY_LEFT;
	imx219->cur_crop.top = IMX219_PIXEL_ARRAY_TOP;
	imx219->cur_crop.width = IMX219_PIXEL_ARRAY_WIDTH;
	imx219->cur_crop.height = IMX219_PIXEL_ARRAY_HEIGHT;

	/* Initialize subdev */
	imx219->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	imx219->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	imx219->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx219->sd.entity, 1, &imx219->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads\n");
		goto error_handler_free;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	sd = &imx219->sd;
	memset(facing, 0, sizeof(facing));
	if (strcmp(imx219->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 imx219->module_index, facing,
		 IMX219_NAME, dev_name(sd->dev));

	ret = v4l2_async_register_subdev_sensor(&imx219->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device\n");
		goto error_media_entity;
	}

	pm_runtime_idle(dev);

	return 0;

error_media_entity:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	media_entity_cleanup(&imx219->sd.entity);

error_handler_free:
	imx219_free_controls(imx219);

error_mutex_destroy:
	mutex_destroy(&imx219->lock);

error_power_off:
	imx219_power_off(dev);

	return ret;
}

static void imx219_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx219 *imx219 = to_imx219(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx219_free_controls(imx219);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx219_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&imx219->lock);
}

static const struct of_device_id imx219_dt_ids[] = {
	{ .compatible = "sony,imx219" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx219_dt_ids);

static const struct dev_pm_ops imx219_pm_ops = {
	SET_RUNTIME_PM_OPS(imx219_power_off, imx219_power_on, NULL)
};

static struct i2c_driver imx219_i2c_driver = {
	.driver = {
		.name = "imx219",
		.of_match_table = imx219_dt_ids,
		.pm = &imx219_pm_ops,
	},
	.probe_new = imx219_probe,
	.remove = imx219_remove,
};

module_i2c_driver(imx219_i2c_driver);

MODULE_DESCRIPTION("Sony IMX219 Camera driver");
MODULE_AUTHOR("Raspberry Pi (Trading) Ltd");
MODULE_LICENSE("GPL v2");
