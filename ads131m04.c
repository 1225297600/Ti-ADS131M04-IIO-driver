// SPDX-License-Identifier: GPL-2.0
/*
 * Texas Instruments ADS131M04 4-Channel ADC
 *
 * Based on working old driver, only fixed channel enable register.
 * Copyright (c) 2024 [ZZL]
 *
 * Added per-channel gain control via sysfs (in_voltageX_gain)
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/sched/clock.h>

#include <asm/unaligned.h>
#include <linux/dma-mapping.h>

/* Commands */
#define ADS131M04_CMD_NULL		0x0000
#define ADS131M04_CMD_RESET		0x0011
#define ADS131M04_CMD_STANDBY		0x0022
#define ADS131M04_CMD_WAKEUP		0x0033
#define ADS131M04_CMD_LOCK		0x0555
#define ADS131M04_CMD_UNLOCK		0x0655
#define ADS131M04_CMD_RREG(addr, n)	(0xA000 | (((addr) & 0x3F) << 7) | (((n) - 1) & 0x7F))
#define ADS131M04_CMD_WREG(addr, n)	(0x6000 | (((addr) & 0x3F) << 7) | (((n) - 1) & 0x7F))

/* Registers */
#define ADS131M04_REG_ID		0x00
#define ADS131M04_REG_STA		0x01
#define ADS131M04_REG_MODE		0x02
#define ADS131M04_REG_CLK		0x03
#define ADS131M04_REG_GAIN		0x04
#define ADS131M04_REG_CFG		0x06
#define ADS131M04_REG_THRSH_MSB		0x07
#define ADS131M04_REG_THRSH_LSB		0x08
#define ADS131M04_REG_CH0_CFG		0x09
#define ADS131M04_REG_CH1_CFG		0x0A
#define ADS131M04_REG_CH2_CFG		0x0B
#define ADS131M04_REG_CH3_CFG		0x0C
#define ADS131M04_REG_CRC		0x3E

/* Register field masks */
#define ADS131M04_STA_DRDY_MASK		BIT(15)
#define ADS131M04_STA_CRC_ERR_MASK	BIT(14)
#define ADS131M04_STA_CMD_ERR_MASK	BIT(13)
#define ADS131M04_STA_REG_ERR_MASK	BIT(12)
#define ADS131M04_STA_RESET_MASK	BIT(11)

#define ADS131M04_CLK_OSR_MASK		GENMASK(4, 2)
#define ADS131M04_CLK_PWR_MASK		GENMASK(1, 0)

/* 修正：通道使能位在 CLK 寄存器，而不是 MODE 寄存器 */
#define ADS131M04_CLK_CH_EN_MASK(ch)	BIT(8 + (ch))

/* 增益寄存器字段掩码（每个通道3位） */
#define ADS131M04_GAIN_CH0_MASK		GENMASK(2, 0)   /* GAIN bits 2-0 */
#define ADS131M04_GAIN_CH1_MASK		GENMASK(6, 4)   /* GAIN bits 6-4 */
#define ADS131M04_GAIN_CH2_MASK		GENMASK(10, 8)   /* GAIN bits 10-8 */
#define ADS131M04_GAIN_CH3_MASK		GENMASK(14, 12)   /* GAIN bits 14-12 */

/* 每个通道的偏移量（位偏移） */
#define ADS131M04_GAIN_CH0_SHIFT	0
#define ADS131M04_GAIN_CH1_SHIFT	4
#define ADS131M04_GAIN_CH2_SHIFT	8
#define ADS131M04_GAIN_CH3_SHIFT	12

/* ADC misc */
#define ADS131M04_NUM_CHANNELS		4
#define ADS131M04_DATA_BITS		24
#define ADS131M04_DATA_BYTES		3
#define ADS131M04_STATUS_BYTES		3
/* 关键：帧长度定义为15，但读取时使用 +1 字节（与 old 驱动一致） */
#define ADS131M04_FRAME_SIZE		(ADS131M04_STATUS_BYTES + ADS131M04_NUM_CHANNELS * ADS131M04_DATA_BYTES)

#define ADS131M04_VREF_mV		1200

#define ADS131M04_WAIT_RESET_US		1000
#define ADS131M04_WAIT_SETUP_US		100

#define DEBUG_REG		0
#define DEBUG_DATA		0
#define DEBUG_INT		0

/* OSR values */
struct ads131m04_osr_desc {
	unsigned int rate;
	u8 reg;
};

static const struct ads131m04_osr_desc ads131m04_osr_tbl[] = {
	{ .rate = 32000,   .reg = 0x0 },
	{ .rate = 16000,   .reg = 0x1 },
	{ .rate = 8000,   .reg = 0x2 },
	{ .rate = 4000,    .reg = 0x3 },
	{ .rate = 2000,    .reg = 0x4 },
	{ .rate = 1000,    .reg = 0x5 },
	{ .rate = 500,    .reg = 0x6 },
	{ .rate = 250,     .reg = 0x7 },
};

struct ads131m04_pga_gain_desc {
	unsigned int gain;
	u8 reg;
};

static const struct ads131m04_pga_gain_desc ads131m04_pga_gain_tbl[] = {
	{ .gain = 1,   .reg = 0x0 },
	{ .gain = 2,   .reg = 0x1 },
	{ .gain = 4,   .reg = 0x2 },
	{ .gain = 8,   .reg = 0x3 },
	{ .gain = 16,  .reg = 0x4 },
	{ .gain = 32,  .reg = 0x5 },
	{ .gain = 64,  .reg = 0x6 },
	{ .gain = 128, .reg = 0x7 },
};

struct ads131m04_channel_config {
	unsigned int pga_gain;
	bool enabled;
};

struct ads131m04_state {
	struct spi_device *spi;
	struct iio_trigger *trig;
	struct regulator *vref_reg;
	struct ads131m04_channel_config *channel_config;
	struct gpio_desc *reset_gpio;
	unsigned int data_rate;
	struct completion completion;
	struct mutex lock;

	// [CH0 4B][CH1 4B][CH2 4B][CH3 4B] [timestamp 8B] = 24 字节
	struct {
		int32_t channels[ADS131M04_NUM_CHANNELS];
		uint64_t ts __aligned(8);
	} dev_buf;

	u8 tx_buf[6] __aligned(ARCH_DMA_MINALIGN);
	/* rx_buf 比帧长度多 2 字节，与 old 驱动一致 */
	u8 rx_buf[ADS131M04_FRAME_SIZE + 2];
};

/**
 * ads131m04_exec_cmd - 向 ADC 发送命令
 * @st: 设备状态结构体
 * @cmd: 要发送的命令码（16位）
 *
 * 通过 SPI 向 ADS131M04 发送一条命令（不读写寄存器数据）。
 * 返回 0 成功，负值表示错误码。
 */
static int ads131m04_exec_cmd(struct ads131m04_state *st, u16 cmd)
{
	int ret;
	u8 tx_buf[2] = { cmd >> 8, cmd & 0xFF };
	struct spi_transfer transfer = {
		.tx_buf = tx_buf,
		.len = 2,
	};
	struct spi_message msg;

	spi_message_init(&msg);
	spi_message_add_tail(&transfer, &msg);

	mutex_lock(&st->lock);
	ret = spi_sync(st->spi, &msg);
	mutex_unlock(&st->lock);

	if (ret)
		dev_err(&st->spi->dev, "Exec cmd(%04x) failed\n", cmd);
	return ret;
}

/**
 * ads131m04_read_reg - 读取单个寄存器
 * @st: 设备状态结构体
 * @addr: 寄存器地址
 * @value: 输出参数，读取到的寄存器值
 *
 * 从 ADS131M04 读取一个16位寄存器的值。
 * 返回 0 成功，负值表示错误码。
 */
static int ads131m04_read_reg(struct ads131m04_state *st, u8 addr, u16 *value)
{
	int ret;
	u16 cmd = ADS131M04_CMD_RREG(addr, 1);
	u8 tx_buf[3] = { cmd >> 8, cmd & 0xFF, 0 };
	u8 rx_buf[3] = {0};
	struct spi_transfer transfer = {
		.tx_buf = tx_buf,
		.rx_buf = rx_buf,
		.len = 3,
	};

	mutex_lock(&st->lock);
	/* 第一次发送命令 */
	ret = spi_sync_transfer(st->spi, &transfer, 1);
	if (ret) {
		mutex_unlock(&st->lock);
		return ret;
	}
	/* 第二次读取数据（同时发送哑元） */
	ret = spi_sync_transfer(st->spi, &transfer, 1);
	mutex_unlock(&st->lock);

	if (ret) {
		dev_err(&st->spi->dev, "Read register failed\n");
		return ret;
	}

	*value = (rx_buf[0] << 8) | rx_buf[1];
#if DEBUG_REG
	dev_info(&st->spi->dev, "Read reg 0x%02x = 0x%04x\n", addr, *value);
#endif
	return 0;
}

/**
 * ads131m04_write_reg - 写单个寄存器
 * @st: 设备状态结构体
 * @addr: 寄存器地址
 * @value: 要写入的16位值
 *
 * 向 ADS131M04 的指定寄存器写入一个16位值。
 * 返回 0 成功，负值表示错误码。
 */
static int ads131m04_write_reg(struct ads131m04_state *st, u8 addr, u16 value)
{
	u16 cmd = ADS131M04_CMD_WREG(addr, 1);
	u8 tx_buf[6] = {
		cmd >> 8, cmd & 0xFF, 0,
		value >> 8, value & 0xFF, 0
	};
	u8 rx_buf[6] = {0};
	struct spi_transfer transfer = {
		.tx_buf = tx_buf,
		.rx_buf = rx_buf,
		.len = 6,
	};
	int ret;

	mutex_lock(&st->lock);
	ret = spi_sync_transfer(st->spi, &transfer, 1);
	mutex_unlock(&st->lock);

	if (ret)
		dev_err(&st->spi->dev, "Write register failed\n");
#if DEBUG_REG
	dev_info(&st->spi->dev, "Write reg 0x%02x = 0x%04x\n", addr, value);
#endif
	return ret;
}

/**
 * ads131m04_read_data - 读取 ADC 转换数据帧
 * @st: 设备状态结构体
 *
 * 从 ADS131M04 读取一帧数据，包括状态字节和所有通道的 ADC 数据。
 * 读取的长度为 ADS131M04_FRAME_SIZE + 1，与旧驱动保持一致。
 * 读取结果存储在 st->rx_buf 中。
 * 返回 0 成功，负值表示错误码。
 */
static int ads131m04_read_data(struct ads131m04_state *st)
{
	u8 tx_buf[ADS131M04_FRAME_SIZE] = {0};
	struct spi_transfer transfer = {
		.tx_buf = tx_buf,
		.rx_buf = st->rx_buf,
		.len = ADS131M04_FRAME_SIZE + 1,
	};
	int ret;

	mutex_lock(&st->lock);
	ret = spi_sync_transfer(st->spi, &transfer, 1);
	mutex_unlock(&st->lock);

	if (ret)
		dev_err(&st->spi->dev, "Read data failed\n");

#if DEBUG_DATA
	{
		char hex[128] = {0};
		int i;
		for (i = 0; i < ADS131M04_FRAME_SIZE + 1; i++) {
			char tmp[4];
			snprintf(tmp, sizeof(tmp), "%02X ", st->rx_buf[i]);
			strcat(hex, tmp);
		}
		dev_info(&st->spi->dev, "RX data (%d bytes): %s\n",
			 ADS131M04_FRAME_SIZE + 1, hex);
	}
#endif
	return ret;
}

/**
 * ads131m04_set_data_rate - 设置 ADC 数据输出速率（采样频率）
 * @st: 设备状态结构体
 * @rate: 期望的数据速率（Hz），必须为 osr_tbl 中支持的速率之一
 *
 * 通过配置 CLK 寄存器中的 OSR 位来改变采样率。
 * 返回 0 成功，负值表示错误码（如速率不支持）。
 */
static int ads131m04_set_data_rate(struct ads131m04_state *st, unsigned int rate)
{
	int i, ret;
	u16 reg_val;

	for (i = 0; i < ARRAY_SIZE(ads131m04_osr_tbl); i++) {
		if (ads131m04_osr_tbl[i].rate == rate)
			break;
	}
	if (i == ARRAY_SIZE(ads131m04_osr_tbl))
		return -EINVAL;

	ret = ads131m04_read_reg(st, ADS131M04_REG_CLK, &reg_val);
	if (ret)
		return ret;

	reg_val &= ~ADS131M04_CLK_OSR_MASK;
	reg_val |= FIELD_PREP(ADS131M04_CLK_OSR_MASK, ads131m04_osr_tbl[i].reg);

	ret = ads131m04_write_reg(st, ADS131M04_REG_CLK, reg_val);
	if (ret)
		return ret;

	st->data_rate = rate;
	return 0;
}

/**
 * ads131m04_pga_gain_to_field_value - 将 PGA 增益值转换为寄存器字段值
 * @st: 设备状态结构体
 * @pga_gain: 期望的增益值（如 1,2,4,...,128）
 *
 * 根据增益表查找对应的寄存器编码。
 * 返回寄存器编码（非负值）或负值错误码（如果增益不支持）。
 */
static int ads131m04_pga_gain_to_field_value(struct ads131m04_state *st,
					     unsigned int pga_gain)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ads131m04_pga_gain_tbl); i++) {
		if (ads131m04_pga_gain_tbl[i].gain == pga_gain)
			break;
	}
	if (i == ARRAY_SIZE(ads131m04_pga_gain_tbl))
		return -EINVAL;
	return ads131m04_pga_gain_tbl[i].reg;
}

/**
 * ads131m04_set_channel_enable - 使能或禁用指定通道
 * @st: 设备状态结构体
 * @channel: 通道号（0~3）
 * @enable: true 为使能，false 为禁用
 *
 * 修正版：通道使能位位于 CLK 寄存器的 bit8~bit11。
 * 返回 0 成功，负值表示错误码。
 */
static int ads131m04_set_channel_enable(struct ads131m04_state *st,
					unsigned int channel, bool enable)
{
	int ret;
	u16 reg_val;

	ret = ads131m04_read_reg(st, ADS131M04_REG_CLK, &reg_val);
	if (ret)
		return ret;

	if (enable)
		reg_val |= ADS131M04_CLK_CH_EN_MASK(channel);
	else
		reg_val &= ~ADS131M04_CLK_CH_EN_MASK(channel);

	return ads131m04_write_reg(st, ADS131M04_REG_CLK, reg_val);
}

/**
 * ads131m04_read_direct - 直接读取单个通道的原始 ADC 值
 * @indio_dev: IIO 设备结构体
 * @chan: 通道规格
 * @value: 输出参数，存放转换后的原始值（24位有符号数）
 *
 * 该函数在直接模式下调用，读取最新的数据帧并提取指定通道的值。
 * 返回 0 成功，负值表示错误码。
 */
static int ads131m04_read_direct(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, int *value)
{
	struct ads131m04_state *st = iio_priv(indio_dev);
	u8 *src;
	int ret = 0;

	ret = ads131m04_read_data(st);
	if (ret)
		return ret;

	src = st->rx_buf + ADS131M04_STATUS_BYTES + chan->channel * ADS131M04_DATA_BYTES;
	*value = sign_extend32(get_unaligned_be32(src) >> 8, 23);
	return 0;
}

/**
 * ads131m04_read_raw - IIO 读 raw 属性回调
 * @indio_dev: IIO 设备结构体
 * @chan: 通道规格
 * @val: 输出值
 * @val2: 输出值第二部分（用于小数值）
 * @mask: 请求的信息类型（RAW, SCALE, SAMP_FREQ 等）
 *
 * 根据 mask 返回对应的通道信息：原始数据、比例因子或采样频率。
 * 返回 IIO_VAL_INT 或 IIO_VAL_FRACTIONAL_LOG2 表示成功，负值表示错误。
 */
static int ads131m04_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct ads131m04_state *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;
		ret = ads131m04_read_direct(indio_dev, chan, val);
		iio_device_release_direct_mode(indio_dev);
		return ret ? ret : IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		if (st->vref_reg) {
			ret = regulator_get_voltage(st->vref_reg);
			if (ret < 0)
				return ret;
			*val = ret / 1000;
		} else {
			*val = ADS131M04_VREF_mV;
		}
		/* 加锁读取当前增益，防止与写操作竞争 */
		mutex_lock(&st->lock);
		*val /= st->channel_config[chan->address].pga_gain;
		mutex_unlock(&st->lock);
		*val2 = ADS131M04_DATA_BITS;
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->data_rate;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}


/**
 * ads131m04_write_raw - IIO 写 raw 属性回调
 * @indio_dev: IIO 设备结构体
 * @chan: 通道规格
 * @val: 写入值
 * @val2: 第二部分（未使用）
 * @mask: 要修改的信息类型（目前支持 SAMP_FREQ）
 *
 * 目前仅支持修改采样频率。
 * 返回 0 成功，负值表示错误。
 */
static int ads131m04_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	struct ads131m04_state *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;
		ret = ads131m04_set_data_rate(st, val);
		iio_device_release_direct_mode(indio_dev);
		return ret;
	default:
		return -EINVAL;
	}
}

static const char * const ads131m04_sampling_freq_available[] = {
	"32000", "16000", "8000", "4000", "2000", "1000", "500", "250"
};

static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("32000 16000 8000 4000 2000 1000 500 250");

static struct attribute *ads131m04_attributes[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group ads131m04_attribute_group = {
	.attrs = ads131m04_attributes,
};


/**
 * ads131m04_set_pga_gain - 设置指定通道的 PGA 增益（修正版）
 * @st: 设备状态结构体
 * @channel: 通道号（0~3）
 * @pga_gain: 期望的增益值（1,2,4,8,16,32,64,128）
 *
 * 根据数据手册正确配置 GAIN1/GAIN2 寄存器的相应位。
 */
static int ads131m04_set_pga_gain(struct ads131m04_state *st,
				  unsigned int channel, unsigned int pga_gain)
{
	int field_value, ret;
	u16 reg_val;
	u16 mask, shift;

	if (channel >= ADS131M04_NUM_CHANNELS)
		return -EINVAL;

	field_value = ads131m04_pga_gain_to_field_value(st, pga_gain);
	if (field_value < 0)
		return field_value;

	switch (channel) {
	case 0:
		mask = ADS131M04_GAIN_CH0_MASK;
		shift = ADS131M04_GAIN_CH0_SHIFT;
		break;
	case 1:
		mask = ADS131M04_GAIN_CH1_MASK;
		shift = ADS131M04_GAIN_CH1_SHIFT;
		break;
	case 2:
		mask = ADS131M04_GAIN_CH2_MASK;
		shift = ADS131M04_GAIN_CH2_SHIFT;
		break;
	case 3:
		mask = ADS131M04_GAIN_CH3_MASK;
		shift = ADS131M04_GAIN_CH3_SHIFT;
		break;
	default:
		return -EINVAL;
	}

	ret = ads131m04_read_reg(st, ADS131M04_REG_GAIN, &reg_val);
	if (ret)
		return ret;

	reg_val &= ~mask;
	reg_val |= (field_value << shift) & mask;

	ret = ads131m04_write_reg(st, ADS131M04_REG_GAIN, reg_val);
	if (!ret)
		st->channel_config[channel].pga_gain = pga_gain;
	return ret;
}


/* ---------- 增益控制扩展属性（已修复死锁）---------- */
static ssize_t ads131m04_gain_read(struct iio_dev *indio_dev,
				   uintptr_t private,
				   struct iio_chan_spec const *chan,
				   char *buf)
{
	struct ads131m04_state *st = iio_priv(indio_dev);
	unsigned int gain;

	mutex_lock(&st->lock);
	gain = st->channel_config[chan->channel].pga_gain;
	mutex_unlock(&st->lock);

	return sprintf(buf, "%u\n", gain);
}

static ssize_t ads131m04_gain_write(struct iio_dev *indio_dev,
				    uintptr_t private,
				    struct iio_chan_spec const *chan,
				    const char *buf, size_t len)
{
	struct ads131m04_state *st = iio_priv(indio_dev);
	unsigned int gain;
	int ret;

	ret = kstrtouint(buf, 10, &gain);
	if (ret)
		return ret;

	/* 检查增益是否有效 */
	ret = ads131m04_pga_gain_to_field_value(st, gain);
	if (ret < 0)
		return -EINVAL;

	/* 确保不与缓冲区数据采集冲突 */
	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	/* 注意：此处不再加 st->lock，因为 ads131m04_set_pga_gain 内部会通过
	 * ads131m04_read_reg/ads131m04_write_reg 加锁，避免死锁。
	 */
	ret = ads131m04_set_pga_gain(st, chan->channel, gain);

	iio_device_release_direct_mode(indio_dev);
	return ret ? ret : len;
}

static const struct iio_chan_spec_ext_info ads131m04_gain_ext_info[] = {
	{
		.name = "gain",
		.shared = IIO_SEPARATE,
		.read = ads131m04_gain_read,
		.write = ads131m04_gain_write,
	},
	{ }
};
/* ------------------------------------ */

/**
 * ads131m04_debugfs_reg_access - debugfs 寄存器读写回调
 * @indio_dev: IIO 设备结构体
 * @reg: 寄存器地址
 * @writeval: 写入值（写操作时）
 * @readval: 读取值输出指针（读操作时）
 *
 * 允许通过 debugfs 直接读写 ADS131M04 的内部寄存器。
 * 返回 0 成功，负值表示错误。
 */
static int ads131m04_debugfs_reg_access(struct iio_dev *indio_dev,
					unsigned int reg, unsigned int writeval,
					unsigned int *readval)
{
	struct ads131m04_state *st = iio_priv(indio_dev);

	if (reg > ADS131M04_REG_CRC)
		return -EINVAL;

	if (readval) {
		u16 val;
		int ret = ads131m04_read_reg(st, reg, &val);
		*readval = val;
		return ret;
	}
	return ads131m04_write_reg(st, reg, writeval);
}

static const struct iio_info ads131m04_iio_info = {
	.read_raw = ads131m04_read_raw,
	.write_raw = ads131m04_write_raw,
	.attrs = &ads131m04_attribute_group,
	.debugfs_reg_access = ads131m04_debugfs_reg_access,
};

/**
 * ads131m04_set_trigger_state - 设置触发器的状态（唤醒/待机）
 * @trig: IIO 触发器
 * @state: true 为唤醒，false 为待机
 *
 * 通过发送 WAKEUP 或 STANDBY 命令来控制 ADC 的功耗状态。
 * 返回 0 成功，负值表示错误。
 */
static int ads131m04_set_trigger_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct ads131m04_state *st = iio_priv(indio_dev);
	u16 cmd = state ? ADS131M04_CMD_WAKEUP : ADS131M04_CMD_STANDBY;
	return ads131m04_exec_cmd(st, cmd);
}

static const struct iio_trigger_ops ads131m04_trigger_ops = {
	.set_trigger_state = ads131m04_set_trigger_state,
	.validate_device = iio_trigger_validate_own_device,
};

/**
 * ads131m04_trigger_handler - 触发器中断处理函数（用于缓冲区模式）
 * @irq: 中断号（未使用）
 * @p: 指向 iio_poll_func 的指针
 *
 * 当 ADC 数据就绪且使用硬件触发器时调用。读取数据帧，按扫描掩码提取通道值，
 * 并推送到 IIO 缓冲区。最后通知触发器完成。
 * 返回 IRQ_HANDLED。
 */
static irqreturn_t ads131m04_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ads131m04_state *st = iio_priv(indio_dev);
	uint8_t i = 0;
	int bit;
	u8 *src;

	if (ads131m04_read_data(st))
		goto done;

	st->dev_buf.ts = ktime_get_real();
	
	//  准备数据
	for_each_set_bit(bit, indio_dev->active_scan_mask, indio_dev->num_channels) {
        src = st->rx_buf + ADS131M04_STATUS_BYTES +
              bit * ADS131M04_DATA_BYTES;
        // 用大端方式读取 24 位原始数据并符号扩展为 32 位
        st->dev_buf.channels[i] = sign_extend32(
            get_unaligned_be32(src) >> 8, 23);
        i++;
	}

	// 数据写入/dev/iio:device*
	iio_push_to_buffers(indio_dev, &st->dev_buf);

done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

/**
 * ads131m04_interrupt - 设备中断处理函数
 * @irq: 中断号
 * @private: 指向 iio_dev 的指针
 *
 * 根据当前工作模式决定行为：
 * - 如果缓冲区已使能且使用自己的触发器，则触发 iio_trigger_poll；
 * - 否则完成等待的 completion（用于单次读取）。
 * 返回 IRQ_HANDLED。
 */
#if DEBUG_INT
uint32_t debug_cnt = 0;
#endif
static irqreturn_t ads131m04_interrupt(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct ads131m04_state *st = iio_priv(indio_dev);

	if (iio_buffer_enabled(indio_dev) && iio_trigger_using_own(indio_dev))
	{
		iio_trigger_poll(st->trig);
#if DEBUG_INT
		if(debug_cnt%2000 == 0)
		{
			dev_info(&st->spi->dev, "ads131m04 iio_trigger_poll +2000\n");
		}
#endif
	}else{
		complete(&st->completion);
#if DEBUG_INT
		if(debug_cnt%2000 == 0)
		{
			dev_info(&st->spi->dev, "ads131m04 complete +2000\n");
		}
#endif
	}

	return IRQ_HANDLED;
}

/**
 * ads131m04_parse_dt - 解析设备树配置
 * @indio_dev: IIO 设备结构体
 *
 * 从设备树中读取 data-rate 和 ti,channel-config 属性，并配置 ADC。
 * 如果属性不存在，则使用默认值（采样率 1000Hz，所有通道使能，增益为 1）。
 * 返回 0 成功，负值表示错误。
 */
static int ads131m04_parse_dt(struct iio_dev *indio_dev)
{
	struct ads131m04_state *st = iio_priv(indio_dev);
	struct device_node *np = st->spi->dev.of_node;
	u32 data_rate;
	int ret, i, count;
	u32 ch_config[ADS131M04_NUM_CHANNELS * 3];

	if (!of_property_read_u32(np, "data-rate", &data_rate)) {
		ret = ads131m04_set_data_rate(st, data_rate);
		if (ret) {
			dev_err(&st->spi->dev, "Failed to set data rate\n");
			return ret;
		}
	} else {
		ret = ads131m04_set_data_rate(st, 1000);
		if (ret)
			return ret;
	}

	count = of_property_count_u32_elems(np, "ti,channel-config");
	if (count > 0 && count % 3 == 0) {
		ret = of_property_read_u32_array(np, "ti,channel-config", ch_config, count);
		if (ret) {
			dev_err(&st->spi->dev, "Failed to read ti,channel-config\n");
			return ret;
		}
		for (i = 0; i < count / 3; i++) {
			unsigned int chan = ch_config[i*3];
			unsigned int gain = ch_config[i*3+1];
			unsigned int enable = ch_config[i*3+2];
			if (chan >= ADS131M04_NUM_CHANNELS)
				continue;
			st->channel_config[chan].pga_gain = gain;
			st->channel_config[chan].enabled = !!enable;
		}
	} else {
		for (i = 0; i < ADS131M04_NUM_CHANNELS; i++) {
			st->channel_config[i].pga_gain = 1;
			st->channel_config[i].enabled = true;
		}
	}
	return 0;
}

/**
 * ads131m04_alloc_channels - 分配并初始化 IIO 通道数组
 * @indio_dev: IIO 设备结构体
 *
 * 为 4 个电压通道分配 iio_chan_spec 和内部配置结构体，
 * 并设置扫描类型、信息掩码等。
 * 返回 0 成功，负值表示内存分配失败。
 */
static int ads131m04_alloc_channels(struct iio_dev *indio_dev)
{
	struct ads131m04_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	struct iio_chan_spec *channels;
	struct ads131m04_channel_config *cfg;
	int i;

	// 注意：多分配一个通道给时间戳
	channels = devm_kcalloc(dev, ADS131M04_NUM_CHANNELS+1, sizeof(*channels), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	cfg = devm_kcalloc(dev, ADS131M04_NUM_CHANNELS, sizeof(*cfg), GFP_KERNEL);
	if (!cfg)
		return -ENOMEM;

	for (i = 0; i < ADS131M04_NUM_CHANNELS; i++) {
		cfg[i].pga_gain = 1;
		cfg[i].enabled = true;

		channels[i].type = IIO_VOLTAGE;
		channels[i].indexed = 1;
		channels[i].channel = i;
		channels[i].address = i;
		channels[i].info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE);
		channels[i].info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ);
		channels[i].scan_index = i;
		channels[i].scan_type.sign = 's';
		channels[i].scan_type.realbits = 24;
		channels[i].scan_type.storagebits = 32;
		channels[i].scan_type.shift = 0;
		channels[i].scan_type.endianness = IIO_CPU;
		/* 附加增益控制扩展属性 */
		channels[i].ext_info = ads131m04_gain_ext_info;
	}

	// 添加时间戳通道，scan_index 使用 4（紧随 4 个电压通道）
	channels[ADS131M04_NUM_CHANNELS] = (struct iio_chan_spec) {
		.type = IIO_TIMESTAMP,
		.channel = -1,
		.scan_index = ADS131M04_NUM_CHANNELS,
		.scan_type = {
			.sign = 's',
			.realbits = 64,
			.storagebits = 64,
		},
	};

	indio_dev->channels = channels;
	indio_dev->num_channels = ADS131M04_NUM_CHANNELS + 1;
	st->channel_config = cfg;
	return 0;
}

/**
 * ads131m04_regulator_disable - 禁用参考电压稳压器
 * @data: 指向 ads131m04_state 的指针
 *
 * 用于 devm_add_action_or_reset，在设备卸载时自动禁用 vref 稳压器。
 */
static void ads131m04_regulator_disable(void *data)
{
	struct ads131m04_state *st = data;
	regulator_disable(st->vref_reg);
}

static int ads131m04_check_id_with_retry(struct ads131m04_state *st)
{
    int i, ret;
    u16 reg_val;

    for (i = 0; i < 10; i++) {
        ret = ads131m04_read_reg(st, ADS131M04_REG_ID, &reg_val);
        if (ret == 0 && (reg_val & 0xFF00) == 0x2400) {
            dev_info(&st->spi->dev, "Found ADS131M04 (ID=0x%04x)\n", reg_val);
            return 0;
        }
        msleep(5);
    }
    dev_err(&st->spi->dev, "Invalid ID after retries\n");
    return -ENODEV;
}

/**
 * ads131m04_initial_config - 执行 ADC 的初始化配置
 * @indio_dev: IIO 设备结构体
 *
 * 完成以下步骤：
 * 1. 硬件或软件复位 ADC；
 * 2. 读取 ID 寄存器校验设备型号；
 * 3. 解析设备树配置；
 * 4. 设置各通道的增益和使能状态；
 * 5. 配置 MODE 和 CLK 寄存器的基本值。
 * 返回 0 成功，负值表示错误。
 */
static int ads131m04_initial_config(struct iio_dev *indio_dev)
{
	struct ads131m04_state *st = iio_priv(indio_dev);
	int ret, i;
	u16 reg_val;

	/* 硬件复位 */
	if (st->reset_gpio) {
		dev_info(&st->spi->dev, "Performing hardware reset\n");
		/* 完整复位脉冲：高 -> 低 -> 高 */
		gpiod_set_value_cansleep(st->reset_gpio, 1);   // 确保初始为高
		usleep_range(100, 200);
		gpiod_set_value_cansleep(st->reset_gpio, 0);   // 拉低产生下降沿
		usleep_range(100, 200);
		gpiod_set_value_cansleep(st->reset_gpio, 1);   // 释放复位
		usleep_range(ADS131M04_WAIT_RESET_US, ADS131M04_WAIT_RESET_US + 100);
	} else {
		/* 软件复位保持不变 */
		ret = ads131m04_exec_cmd(st, ADS131M04_CMD_RESET);
		if (ret)
			return ret;
		usleep_range(ADS131M04_WAIT_RESET_US, ADS131M04_WAIT_RESET_US + 100);
	}

	/* 校验设备 ID */
	ret = ads131m04_check_id_with_retry(st);
	if (ret)
		return ret;

	/* 解析设备树配置 */
	ret = ads131m04_parse_dt(indio_dev);
	if (ret)
		return ret;

	/* 应用通道增益和使能 */
	for (i = 0; i < ADS131M04_NUM_CHANNELS; i++) {
		ret = ads131m04_set_pga_gain(st, i, st->channel_config[i].pga_gain);
		if (ret)
			return ret;

		ret = ads131m04_set_channel_enable(st, i, st->channel_config[i].enabled);
		if (ret)
			return ret;
	}

	/* 设置默认 MODE 寄存器 */
	ret = ads131m04_write_reg(st, ADS131M04_REG_MODE, 0x0510);
	if (ret)
		return ret;

	/* 设置默认 CLK 寄存器（保留通道使能位） */
	ret = ads131m04_read_reg(st, ADS131M04_REG_CLK, &reg_val);
	if (ret)
		return ret;
	reg_val |= (0xF << 8); /* 确保所有通道使能 */
	ret = ads131m04_write_reg(st, ADS131M04_REG_CLK, reg_val);
	if (ret)
		return ret;

	return 0;
}

/**
 * ads131m04_probe - SPI 设备探测函数
 * @spi: SPI 设备结构体
 *
 * 初始化 SPI 接口，分配 IIO 设备，设置触发器和缓冲区，
 * 获取参考电压稳压器（可选），执行设备初始化，最后注册 IIO 设备。
 * 返回 0 成功，负值表示错误码。
 */
static int ads131m04_probe(struct spi_device *spi)
{
	struct ads131m04_state *st;
	struct iio_dev *indio_dev;
	int ret;

	spi->mode = SPI_MODE_1;
	spi->bits_per_word = 8;
	spi->max_speed_hz = 1000000;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	mutex_init(&st->lock);
	init_completion(&st->completion);

	ret = ads131m04_alloc_channels(indio_dev);
	if (ret)
		return ret;

	indio_dev->name = "ads131m04";
	indio_dev->info = &ads131m04_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	st->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return PTR_ERR(st->reset_gpio);

	if (spi->irq) {
		ret = devm_request_irq(&spi->dev, spi->irq, ads131m04_interrupt,
				       IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				       spi->dev.driver->name, indio_dev);
		if (ret)
			return dev_err_probe(&spi->dev, ret, "IRQ request failed\n");
	}

	st->trig = devm_iio_trigger_alloc(&spi->dev, "%s-spi%d.%d",
					  indio_dev->name,
					  spi->controller->bus_num,
					  spi->chip_select);

	if (!st->trig)
		return -ENOMEM;
	st->trig->ops = &ads131m04_trigger_ops;
	st->trig->dev.parent = &spi->dev;
	iio_trigger_set_drvdata(st->trig, indio_dev);
	ret = devm_iio_trigger_register(&spi->dev, st->trig);
	if (ret)
		return ret;
	indio_dev->trig = iio_trigger_get(st->trig);

	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev, NULL,
					      ads131m04_trigger_handler, NULL);
	if (ret)
		return ret;

	st->vref_reg = devm_regulator_get_optional(&spi->dev, "vref");
	if (!IS_ERR(st->vref_reg)) {
		ret = regulator_enable(st->vref_reg);
		if (ret)
			return ret;
		ret = devm_add_action_or_reset(&spi->dev, ads131m04_regulator_disable, st);
		if (ret)
			return ret;
	} else if (PTR_ERR(st->vref_reg) != -ENODEV) {
		return PTR_ERR(st->vref_reg);
	} else {
		st->vref_reg = NULL;
	}

	ret = ads131m04_initial_config(indio_dev);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ads131m04_of_match[] = {
	{ .compatible = "ti,ads131m04" },
	{ }
};
MODULE_DEVICE_TABLE(of, ads131m04_of_match);

static const struct spi_device_id ads131m04_ids[] = {
	{ "ads131m04", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, ads131m04_ids);

static struct spi_driver ads131m04_driver = {
	.driver = {
		.name = "ads131m04",
		.of_match_table = ads131m04_of_match,
	},
	.probe = ads131m04_probe,
	.id_table = ads131m04_ids,
};
module_spi_driver(ads131m04_driver);

MODULE_AUTHOR("[ZZL]");
MODULE_DESCRIPTION("Driver for ADS131M04 ADC (fixed from old version) with per-channel gain control");
MODULE_LICENSE("GPL v2");