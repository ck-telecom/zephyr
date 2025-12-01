
#define DT_DRV_COMPAT chipone_co5300

#include <zephyr/device.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/display.h>

LOG_MODULE_REGISTER(display_co5300, CONFIG_DISPLAY_LOG_LEVEL);

/* Display offsets */
#define ROW_OFFSET 0x00
#define COL_OFFSET 0x00

/* CO5300 Registers */
#define REG_SW_RESET           0x01
#define REG_LCD_ID             0x04
#define REG_DSI_ERR            0x05
#define REG_POWER_MODE         0x0A
#define REG_SLEEP_IN           0x10
#define REG_SLEEP_OUT          0x11
#define REG_PARTIAL_DISPLAY    0x12
#define REG_NOR_ON             0x13
#define REG_DISPLAY_INVERSION  0x21
#define REG_ALLP_OFF           0x22
#define REG_ALLP_ON            0x23
#define REG_DISPLAY_OFF        0x28
#define REG_DISPLAY_ON         0x29
#define REG_RAM_WR             0x2C
#define REG_RAM_RD             0x2E
#define REG_CASET              0x2A
#define REG_RASET              0x2B
#define REG_PART_CASET         0x30
#define REG_PART_RASET         0x31
#define REG_VSCRDEF            0x33
#define REG_VSCSAD             0x37
#define REG_TEARING_EFFECT     0x35
#define REG_NORMAL_DISPLAY     0x36
#define REG_IDLE_MODE_OFF      0x38
#define REG_IDLE_MODE_ON       0x39
#define REG_COLOR_MODE         0x3A
#define REG_CONTINUE_WRITE_RAM 0x3C
#define REG_WRDISBV            0x51
#define REG_RDDISBV            0x52
#define REG_WRCTRLD            0x53
#define REG_RDCTRLD            0x54
#define REG_WRHBMDISBV         0x63
#define REG_PORCH_CTRL         0xB2
#define REG_FRAME_CTRL         0xB3
#define REG_GATE_CTRL          0xB7
#define REG_VCOM_SET           0xBB
#define REG_LCM_CTRL           0xC0
#define REG_SET_TIME_SRC       0xC2
#define REG_SET_SPI_MODE       0xC4
#define REG_VCOMH_OFFSET_SET   0xC5
#define REG_FR_CTRL            0xC6
#define REG_POWER_CTRL         0xD0
#define REG_PV_GAMMA_CTRL      0xE0
#define REG_NV_GAMMA_CTRL      0xE1
#define REG_SPI2EN             0xE7
#define REG_PAGE_SWITCH_WR     0xFE
#define REG_PAGE_SWITCH_RD     0xFF

#define CO5300_LCD_ID          0x331100
#define REG_BRIGHTNESS_MAX      0xFF

struct display_co5300_config {
	const struct device *mipi_dbi;
	const struct mipi_dbi_config dbi_config;
	struct gpio_dt_spec dc_gpio;
	struct gpio_dt_spec reset_gpio;
	const struct gpio_dt_spec avdd_en_gpio;
	uint16_t width;
	uint16_t height;
};

struct display_co5300_data {
	uint8_t pixel_format;
	bool blanking_on;
};

static int co5300_command_write(const struct device *dev, uint8_t cmd, const uint8_t *data, size_t len)
{
	const struct display_co5300_config *config = dev->config;

	return mipi_dbi_command_write(config->mipi_dbi, &config->dbi_config, cmd, data, len);
}

static int co5300_command_read(const struct device *dev, uint8_t cmd, uint8_t *data, size_t len)
{
	const struct display_co5300_config *config = dev->config;

	return mipi_dbi_command_read(config->mipi_dbi, &config->dbi_config, &cmd, sizeof(cmd), data, len);
}

static int display_co5300_set_window(const struct device *dev, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	uint8_t caset[4];
	uint8_t raset[4];
	int ret;

	x0 += COL_OFFSET;
	x1 += COL_OFFSET;
	y0 += ROW_OFFSET;
	y1 += ROW_OFFSET;

	caset[0] = (x0 >> 8) & 0x3;
	caset[1] = x0 & 0xFF;
	caset[2] = (x1 >> 8) & 0x3;
	caset[3] = x1 & 0xFF;

	raset[0] = (y0 >> 8) & 0x3;
	raset[1] = y0 & 0xFF;
	raset[2] = (y1 >> 8) & 0x3;
	raset[3] = y1 & 0xFF;

	ret = co5300_command_write(dev, REG_CASET, caset, sizeof(caset));
	if (ret < 0) {
		return ret;
	}

	return co5300_command_write(dev, REG_RASET, raset, sizeof(raset));
}

static int display_co5300_write(const struct device *dev,
		const uint16_t x, const uint16_t y,
		const struct display_buffer_descriptor *desc,
		const void *buf)
{
	const struct display_co5300_config *config = dev->config;
	struct display_co5300_data *data = dev->data;
	int ret = 0;
	const uint16_t *frame_buf = (uint16_t *)buf;

	LOG_DBG("Writing %dx%d (w,h) @ %dx%d (x,y) - %d", desc->width, desc->height, x, y, desc->pitch);

	ret = display_co5300_set_window(dev, x, y, x + desc->width - 1, y + desc->height - 1);
	if (ret < 0) {
		LOG_ERR("Failed to set window: %d", ret);
		return ret;
	}
#if 1
	ret = co5300_command_write(dev, REG_RAM_WR, buf, (desc->width) * desc->height * 2);
	if (ret < 0) {
		LOG_ERR("Failed to write RAM command: %d", ret);
		return ret;
	}
#endif


	//ret = mipi_dbi_write_display(config->mipi_dbi, &config->dbi_config, buf, desc, data->pixel_format);

	return ret;
}

static int display_co5300_read(const struct device *dev,
				const uint16_t x, const uint16_t y,
				const struct display_buffer_descriptor *desc,
				void *buf)
{
	LOG_ERR("Display read not supported");
	return -ENOTSUP;
}

static void display_co5300_get_capabilities(const struct device *dev,
		struct display_capabilities *capabilities)
{
	const struct display_co5300_config *config = dev->config;
	struct display_co5300_data *data = dev->data;

	memset(capabilities, 0, sizeof(struct display_capabilities));

	capabilities->x_resolution = config->width;
	capabilities->y_resolution = config->height;
	capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_565;
		capabilities->current_pixel_format = PIXEL_FORMAT_RGB_565;
	capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static int display_co5300_set_pixel_format(const struct device *dev,
		const enum display_pixel_format pixel_format)
{
	uint8_t param[1] = { 0 };

	if (pixel_format == PIXEL_FORMAT_RGB_888) {
		struct display_co5300_data *data = dev->data;
		data->pixel_format = pixel_format;
		param[0] = 0xF7;
	} else if (pixel_format == PIXEL_FORMAT_RGB_565) {
		struct display_co5300_data *data = dev->data;
		data->pixel_format = pixel_format;
		param[0] = 0x55;
	}

	return co5300_command_write(dev, REG_COLOR_MODE, param, 1);
}

static int display_co5300_set_brightness(const struct device *dev,
					const uint8_t brightness)
{
	return co5300_command_write(dev, REG_WRDISBV, &brightness, 1);
}

static int display_co5300_set_contrast(const struct device *dev,
					const uint8_t contrast)
{
	LOG_DBG("Setting contrast not supported");
	return -ENOTSUP;
}

static int display_co5300_set_orientation(const struct device *dev,
		const enum display_orientation orientation)
{
	LOG_DBG("Setting orientation not supported");
	return -ENOTSUP;
}

static int display_co5300_blanking_on(const struct device *dev)
{
	struct display_co5300_data *data = dev->data;
	int ret;

	ret = co5300_command_write(dev, REG_DISPLAY_OFF, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	data->blanking_on = true;
	return 0;
}

static int display_co5300_blanking_off(const struct device *dev)
{
	struct display_co5300_data *data = dev->data;
	int ret;

	ret = co5300_command_write(dev, REG_DISPLAY_ON, NULL, 0);
	if (ret < 0) {
			return ret;
		}
		k_sleep(K_MSEC(150));
	}

	data->blanking_on = false;
	return 0;
}

static const struct display_driver_api display_co5300_api = {
	.write = display_co5300_write,
	.read = display_co5300_read,
	.get_capabilities = display_co5300_get_capabilities,
	.set_pixel_format = display_co5300_set_pixel_format,
	.set_brightness = display_co5300_set_brightness,
	.set_contrast = display_co5300_set_contrast,
	.set_orientation = display_co5300_set_orientation,
	.blanking_on = display_co5300_blanking_on,
	.blanking_off = display_co5300_blanking_off,
};

static int display_co5300_init(const struct device *dev)
{
	const struct display_co5300_config *config = dev->config;
	struct display_co5300_data *data = dev->data;
	uint8_t id[3] = { 0 };
	uint8_t param = 0;
	int ret;

	LOG_DBG("Initializing display driver");

	if (!device_is_ready(config->mipi_dbi)) {
		LOG_ERR("MIPI DBI device not ready");
		return -ENODEV;
	}
#if 0
	ret = mipi_dbi_reset(config->mipi_dbi, 100);
	if (ret < 0) {
		return ret;
	}
#endif

	if (config->avdd_en_gpio.port) {
		if (!gpio_is_ready_dt(&config->avdd_en_gpio)) {
			LOG_ERR("AVDD_EN GPIO device not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->avdd_en_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure Reset GPIO pin");
			return ret;
		}
	}

	if (config->reset_gpio.port) {
		if (!gpio_is_ready_dt(&config->reset_gpio)) {
			LOG_ERR("Reset GPIO device not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Could not configure reset GPIO (%d)", ret);
			return ret;
		}
		k_sleep(K_MSEC(10));

		ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
			LOG_ERR("Could not configure reset GPIO (%d)", ret);
		return ret;
	}

		k_sleep(K_MSEC(CONFIG_MIPI_DBI_SF32LB_RESET_DELAY_MS));

		ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Could not configure reset GPIO (%d)", ret);
			return ret;
		}
		k_sleep(K_MSEC(50));
	}

#if 0
	/* Hardware reset */
	gpio_pin_set_dt(&config->reset_gpio, 1);
	k_msleep(10);
	gpio_pin_set_dt(&config->reset_gpio, 0);
	k_msleep(10);
	gpio_pin_set_dt(&config->reset_gpio, 1);
	k_msleep(50);

	uint8_t nop[1] = { 0x5A };
	co5300_command_write(dev, 0x00, nop, sizeof(nop));
	co5300_command_read(dev, 0x00, nop, sizeof(nop));
	LOG_INF("nop: 0x%02x", nop[0]);

	co5300_command_write(dev, REG_SW_RESET, NULL, 0);
	k_busy_wait(1000);

	uint8_t pwr_mode[1] = { 0 };
	ret = co5300_command_read(dev, REG_POWER_MODE, pwr_mode, sizeof(pwr_mode));
	if (ret < 0) {
		LOG_ERR("Failed to read power mode");
		return ret;
	}
	LOG_INF("Power mode: 0x%02x", pwr_mode[0]);
#endif
	/* Read display ID */
	ret = co5300_command_read(dev, REG_LCD_ID, id, sizeof(id));
	if (ret < 0) {
		LOG_ERR("Failed to read display ID");
		return ret;
	}

	uint32_t display_id = (id[2] << 16) | (id[1] << 8) | id[0];
	LOG_INF("display id:0x%06x", display_id);
	if (display_id != CO5300_LCD_ID) {
		LOG_ERR("Invalid display ID: 0x%06x", display_id);
		return -ENODEV;
	}

	/* Password unlock */
	param = 0x20;
	co5300_command_write(dev, REG_PAGE_SWITCH_WR, &param, 1);
	param = 0x5A;
	co5300_command_write(dev, 0xF4, &param, 1);
	param = 0x59;
	co5300_command_write(dev, 0xF5, &param, 1);

#if 1
	/* Password lock */
	param = 0x20;
	co5300_command_write(dev, 0xFE, &param, 1);
	param = 0xA5;
	co5300_command_write(dev, 0xF4, &param, 1);
	param = 0xA5;
	co5300_command_write(dev, 0xF5, &param, 1);

	/* Display settings */
	param = 0x00;
	co5300_command_write(dev, 0xFE, &param, 1);
	param = 0x80;
	co5300_command_write(dev, REG_SET_SPI_MODE, &param, 1);
	param = 0x55; /* RGB565 format */
	co5300_command_write(dev, REG_COLOR_MODE, &param, 1);
	param = 0x00;
	co5300_command_write(dev, REG_TEARING_EFFECT, &param, 1);
	param = 0x20;
	co5300_command_write(dev, REG_WRCTRLD, &param, 1);
	param = 0xFF;
	co5300_command_write(dev, REG_WRHBMDISBV, &param, 1);
	param = 0xFF;
	co5300_command_write(dev, REG_WRDISBV, &param, 1);
	/* Set display area */
	ret = display_co5300_set_window(dev, 0, 0, config->width - 1, config->height - 1);
	if (ret < 0) {
		return ret;
	}

	/* Exit sleep mode */
	co5300_command_write(dev, REG_SLEEP_OUT, NULL, 0);
	k_msleep(120);

	/* Turn on display */
	co5300_command_write(dev, REG_DISPLAY_ON, NULL, 0);

	//data->pixel_format = DISPLAY_PIXEL_FORMAT_RGB565;
	//data->blanking_on = false;
	param = 0x00;
	co5300_command_read(dev, REG_POWER_MODE, &param, 1);
	LOG_INF("Power mode after init: 0x%02x", param);

#endif
	LOG_INF("Display initialized");
	return 0;
}

#define CO5300_WORD_SIZE(inst)								\
	((DT_INST_STRING_UPPER_TOKEN(inst, mipi_mode) == MIPI_DBI_MODE_SPI_4WIRE) ?	 \
	SPI_WORD_SET(8) : SPI_WORD_SET(9))
#define DISPLAY_CO5300_DEFINE(inst) \
	static struct display_co5300_config display_co5300_config_##inst = { \
		.mipi_dbi = DEVICE_DT_GET(DT_INST_PARENT(inst)), \
		.dbi_config = MIPI_DBI_CONFIG_DT_INST(inst, CO5300_WORD_SIZE(inst) | SPI_OP_MODE_MASTER, 0), \
		.avdd_en_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, avdd_enable_gpios, {0}), \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {}),                             \
		.width = DT_INST_PROP(inst, width), \
		.height = DT_INST_PROP(inst, height), \
	};											\
	static struct display_co5300_data display_co5300_data_##inst; \
	DEVICE_DT_INST_DEFINE(inst, \
			      &display_co5300_init, \
			      NULL, \
			      &display_co5300_data_##inst, \
			      &display_co5300_config_##inst, \
			    POST_KERNEL,							\
			      CONFIG_DISPLAY_INIT_PRIORITY, \
			      &display_co5300_api);

DT_INST_FOREACH_STATUS_OKAY(DISPLAY_CO5300_DEFINE)
