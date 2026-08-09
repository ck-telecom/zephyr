/*
 * Copyright (c) 2026 Qingsong Gou <gouqs@hotmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_jdi_parallel

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#include <bf0_hal.h>

LOG_MODULE_REGISTER(sf32lb_jdi_parallel, CONFIG_DISPLAY_LOG_LEVEL);

/* Transfer completion timeout (ms) */
#define JDI_XFER_TIMEOUT_MS 500

struct sf32lb_jdi_parallel_config {
	uintptr_t base;
	const struct pinctrl_dev_config *pcfg;
	struct sf32lb_clock_dt_spec clock;
	struct gpio_dt_spec vlcd_gpio;
	struct gpio_dt_spec vddp_gpio;
	struct pwm_dt_spec vcom_pwm;
	struct pwm_dt_spec vcom_pwm_inv;
	uint16_t power_seq_delay_ms;
	uint16_t width;
	uint16_t height;
	void (*irq_configure)(void);
	/** Clock frequency in Hz. */
	uint32_t freq;

	/** Column bank head padding in pixels (left side). */
	uint16_t bank_col_head;
	/** Number of valid/visible columns in pixels. */
	uint16_t valid_columns;
	/** Column bank tail padding in pixels (right side). */
	uint16_t bank_col_tail;

	/** Row bank head padding in pixels (top side). */
	uint16_t bank_row_head;
	/** Number of valid/visible rows in pixels. */
	uint16_t valid_rows;
	/** Row bank tail padding in pixels (bottom side). */
	uint16_t bank_row_tail;

	/** ENB (Enable) active start column number. */
	uint16_t enb_start_col;
	/** ENB (Enable) active end column number. */
	uint16_t enb_end_col;

	/** Enable signal polarity invert. 1 = low active, 0 = high active (default). */
	uint8_t enb_pol_invert: 1;
	/** HCK (Horizontal Clock) polarity invert. 1 = low active, 0 = high active (default). */
	uint8_t hck_pol_invert: 1;
	/** HST (Horizontal Start) polarity invert. 1 = low active, 0 = high active (default). */
	uint8_t hst_pol_invert: 1;
	/** VCK (Vertical Clock) polarity invert. 1 = low active, 0 = high active (default). */
	uint8_t vck_pol_invert: 1;
	/** VST (Vertical Start) polarity invert. 1 = low active, 0 = high active (default). */
	uint8_t vst_pol_invert: 1;
	/** Reserved bits for future use. */
	uint8_t reserved: 3;
};

struct sf32lb_jdi_parallel_data {
	LCDC_HandleTypeDef hlcdc;
	struct k_sem xfer_done;
};

static struct sf32lb_jdi_parallel_data *g_jdi_data;

void HAL_LCDC_SendLayerDataCpltCbk(LCDC_HandleTypeDef *lcdc)
{
	if (g_jdi_data != NULL) {
		k_sem_give(&g_jdi_data->xfer_done);
	}
}

static void sf32lb_jdi_isr(const struct device *dev)
{
	struct sf32lb_jdi_parallel_data *data = dev->data;

	g_jdi_data = data;

	HAL_LCDC_IRQHandler(&data->hlcdc);
}

static int sf32lb_jdi_config(const struct device *dev)
{
	const struct sf32lb_jdi_parallel_config *config = dev->config;
	struct sf32lb_jdi_parallel_data *data = dev->data;
	HAL_StatusTypeDef ret;

	data->hlcdc.Instance = (LCD_IF_TypeDef *)config->base;
	data->hlcdc.Init.lcd_itf = LCDC_INTF_JDI_PARALLEL;
	data->hlcdc.Init.freq = config->freq;
	data->hlcdc.Init.color_mode = LCDC_PIXEL_FORMAT_RGB332;
	data->hlcdc.Init.cfg.jdi.bank_col_head = config->bank_col_head;
	data->hlcdc.Init.cfg.jdi.valid_columns = config->valid_columns;
	data->hlcdc.Init.cfg.jdi.bank_col_tail = config->bank_col_tail;
	data->hlcdc.Init.cfg.jdi.bank_row_head = config->bank_row_head;
	data->hlcdc.Init.cfg.jdi.valid_rows = config->valid_rows;
	data->hlcdc.Init.cfg.jdi.bank_row_tail = config->bank_row_tail;
	data->hlcdc.Init.cfg.jdi.enb_start_col = config->enb_start_col;
	data->hlcdc.Init.cfg.jdi.enb_end_col = config->enb_end_col;
	data->hlcdc.Init.cfg.jdi.enb_pol_invert = config->enb_pol_invert;
	data->hlcdc.Init.cfg.jdi.hck_pol_invert = config->hck_pol_invert;
	data->hlcdc.Init.cfg.jdi.hst_pol_invert = config->hst_pol_invert;
	data->hlcdc.Init.cfg.jdi.vck_pol_invert = config->vck_pol_invert;
	data->hlcdc.Init.cfg.jdi.vst_pol_invert = config->vst_pol_invert;

	ret = HAL_LCDC_Init(&data->hlcdc);
	if (ret != HAL_OK) {
		LOG_ERR("HAL_LCDC_Init failed: %d", ret);
		return -EIO;
	}

	HAL_LCDC_LayerReset(&data->hlcdc, HAL_LCDC_LAYER_DEFAULT);
	HAL_LCDC_LayerSetCmpr(&data->hlcdc, HAL_LCDC_LAYER_DEFAULT, 0);
	HAL_LCDC_LayerSetFormat(&data->hlcdc, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_RGB332);
	HAL_LCDC_LayerVMirror(&data->hlcdc, HAL_LCDC_LAYER_DEFAULT, false);

	return 0;
}

static int sf32lb_jdi_parallel_blanking_on(const struct device *dev)
{
	ARG_UNUSED(dev);

	LOG_DBG("Turning display off");

	return 0;
}

static int sf32lb_jdi_parallel_blanking_off(const struct device *dev)
{
	const struct sf32lb_jdi_parallel_config *config = dev->config;
	int ret;

	LOG_DBG("Turning display on");

	if (config->vlcd_gpio.port) {
		if (device_is_ready(config->vlcd_gpio.port)) {
			gpio_pin_set_dt(&config->vlcd_gpio, 1);
			k_msleep(config->power_seq_delay_ms);
		}
	}

	if (config->vddp_gpio.port) {
		if (device_is_ready(config->vddp_gpio.port)) {
			gpio_pin_set_dt(&config->vddp_gpio, 1);
			k_msleep(config->power_seq_delay_ms);
		}
	}

	/* Start VCOM PWM (50% duty cycle) for JDI memory-in-pixel display */
	if (config->vcom_pwm.dev) {
		if (!pwm_is_ready_dt(&config->vcom_pwm)) {
			LOG_ERR("VCOM PWM device not ready");
			return -ENODEV;
		}
		ret = pwm_set_dt(&config->vcom_pwm, config->vcom_pwm.period,
				 config->vcom_pwm.period / 2U);
		if (ret < 0) {
			LOG_ERR("Failed to set VCOM PWM: %d", ret);
			return ret;
		}
		LOG_DBG("VCOM PWM A started: period=%u ns", config->vcom_pwm.period);
	}

	if (config->vcom_pwm_inv.dev) {
		if (!pwm_is_ready_dt(&config->vcom_pwm_inv)) {
			LOG_ERR("VCOM PWM invert device not ready");
			return -ENODEV;
		}
		ret = pwm_set_dt(&config->vcom_pwm_inv, config->vcom_pwm_inv.period,
				 config->vcom_pwm_inv.period / 2U);
		if (ret < 0) {
			LOG_ERR("Failed to set VCOM PWM invert: %d", ret);
			return ret;
		}
		LOG_DBG("VCOM PWM B (inverted) started: period=%u ns", config->vcom_pwm_inv.period);
	}

	return 0;
}

static int sf32lb_jdi_parallel_write(const struct device *dev, const uint16_t x, const uint16_t y,
				     const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct sf32lb_jdi_parallel_config *config = dev->config;
	struct sf32lb_jdi_parallel_data *data = dev->data;
	uint16_t roi_x1 = x + desc->width - 1;
	uint16_t roi_y1 = y + desc->height - 1;
	HAL_StatusTypeDef ret;

	if (x >= config->width || y >= config->height) {
		return -EINVAL;
	}

	HAL_LCDC_SetROIArea(&data->hlcdc, x, y, roi_x1, roi_y1);
	HAL_LCDC_LayerSetData(&data->hlcdc, HAL_LCDC_LAYER_DEFAULT,
			      (uint8_t *)buf, x, y, roi_x1, roi_y1);

	ret = HAL_LCDC_SendLayerData_IT(&data->hlcdc);
	if (ret != HAL_OK) {
		LOG_ERR("HAL_LCDC_SendLayerData_IT failed: %d", ret);
		return -EIO;
	}

	LOG_DBG("JDI write: (%u,%u) %ux%u", x, y, desc->width, desc->height);

	return 0;
}

static void sf32lb_jdi_parallel_get_capabilities(const struct device *dev,
						 struct display_capabilities *caps)
{
	const struct sf32lb_jdi_parallel_config *config = dev->config;

	memset(caps, 0, sizeof(*caps));

	caps->x_resolution = config->width;
	caps->y_resolution = config->height;
	caps->current_orientation = DISPLAY_ORIENTATION_NORMAL;
	caps->screen_info = SCREEN_INFO_MONO_VTILED;
}

static DEVICE_API(display, sf32lb_jdi_parallel_api) = {
	.blanking_on = sf32lb_jdi_parallel_blanking_on,
	.blanking_off = sf32lb_jdi_parallel_blanking_off,
	.write = sf32lb_jdi_parallel_write,
	.get_capabilities = sf32lb_jdi_parallel_get_capabilities,
};

static int sf32lb_jdi_parallel_init(const struct device *dev)
{
	const struct sf32lb_jdi_parallel_config *config = dev->config;
	struct sf32lb_jdi_parallel_data *data = dev->data;
	int ret;

	LOG_DBG("Initializing JDI device %s", dev->name);

	/* Enable LCDC clock */
	if (!sf32lb_clock_is_ready_dt(&config->clock)) {
		LOG_ERR("LCDC clock not ready");
		return -ENODEV;
	}

	ret = sf32lb_clock_control_on_dt(&config->clock);
	if (ret < 0) {
		LOG_ERR("Failed to enable LCDC clock: %d", ret);
		return ret;
	}

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to configure pins: %d", ret);
		return ret;
	}

	if (config->vlcd_gpio.port) {
		if (!gpio_is_ready_dt(&config->vlcd_gpio)) {
			LOG_ERR("VLCD GPIO device not ready");
			return -ENODEV;
		}
		gpio_pin_configure_dt(&config->vlcd_gpio, GPIO_OUTPUT_INACTIVE);
	}
	if (config->vddp_gpio.port) {
		if (!gpio_is_ready_dt(&config->vddp_gpio)) {
			LOG_ERR("VDDP GPIO device not ready");
			return -ENODEV;
		}
		gpio_pin_configure_dt(&config->vddp_gpio, GPIO_OUTPUT_INACTIVE);
	}

	k_sem_init(&data->xfer_done, 0, 1);

	sf32lb_jdi_config(dev);

	/* Configure and enable IRQ */
	config->irq_configure();

	LOG_DBG("JDI device initialized successfully");

	return 0;
}

#define SF32LB_JDI_PARALLEL_DEFINE(n)                                                              \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static void sf32lb_jdi_irq_configure_##n(void)                                             \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQN(DT_INST_PARENT(n)), DT_IRQ(DT_INST_PARENT(n), priority),       \
			    sf32lb_jdi_isr, DEVICE_DT_INST_GET(n), 0);                             \
		irq_enable(DT_IRQN(DT_INST_PARENT(n)));                                            \
	}                                                                                          \
                                                                                                   \
	static const struct sf32lb_jdi_parallel_config sf32lb_jdi_parallel_config_##n = {          \
		.base = DT_REG_ADDR(DT_INST_PARENT(n)),                                            \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clock = SF32LB_CLOCK_DT_INST_PARENT_SPEC_GET(n),                                  \
		.irq_configure = sf32lb_jdi_irq_configure_##n,                                     \
		.width = DT_INST_PROP(n, width),                                                   \
		.height = DT_INST_PROP(n, height),                                                 \
		.freq = DT_INST_PROP(n, clock_frequency),                                          \
		.bank_col_head = DT_INST_PROP_OR(n, bank_col_head, 0),                             \
		.valid_columns = DT_INST_PROP_OR(n, valid_columns, 0),                             \
		.bank_col_tail = DT_INST_PROP_OR(n, bank_col_tail, 0),                             \
		.bank_row_head = DT_INST_PROP_OR(n, bank_row_head, 0),                             \
		.valid_rows = DT_INST_PROP_OR(n, valid_rows, 0),                                   \
		.bank_row_tail = DT_INST_PROP_OR(n, bank_row_tail, 0),                             \
		.enb_start_col = DT_INST_PROP_OR(n, enb_start_col, 0),                             \
		.enb_end_col = DT_INST_PROP_OR(n, enb_end_col, 0),                                 \
		.enb_pol_invert = DT_INST_PROP_OR(n, enb_pol_invert, 0),                           \
		.hck_pol_invert = DT_INST_PROP_OR(n, hck_pol_invert, 0),                           \
		.hst_pol_invert = DT_INST_PROP_OR(n, hst_pol_invert, 0),                           \
		.vck_pol_invert = DT_INST_PROP_OR(n, vck_pol_invert, 0),                           \
		.vst_pol_invert = DT_INST_PROP_OR(n, vst_pol_invert, 0),                           \
		.vlcd_gpio = GPIO_DT_SPEC_INST_GET_OR(n, vlcd_gpios, {0}),                         \
		.vddp_gpio = GPIO_DT_SPEC_INST_GET_OR(n, vddp_gpios, {0}),                         \
		.vcom_pwm = PWM_DT_SPEC_INST_GET_BY_IDX(n, 0),                                     \
		.vcom_pwm_inv = PWM_DT_SPEC_INST_GET_BY_IDX_OR(n, 1, {0}),                         \
		.power_seq_delay_ms = DT_INST_PROP_OR(n, power_seq_delay_ms, 11),                  \
	};                                                                                         \
                                                                                                   \
	static struct sf32lb_jdi_parallel_data sf32lb_jdi_parallel_data_##n;                       \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, sf32lb_jdi_parallel_init, NULL,                                   \
			      &sf32lb_jdi_parallel_data_##n,                                       \
			      &sf32lb_jdi_parallel_config_##n, POST_KERNEL,                        \
			      CONFIG_DISPLAY_INIT_PRIORITY, &sf32lb_jdi_parallel_api);

DT_INST_FOREACH_STATUS_OKAY(SF32LB_JDI_PARALLEL_DEFINE)
