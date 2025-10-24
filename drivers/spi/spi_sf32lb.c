/*
 * Copyright (c) 2025, Qingsong Gou <gouqs@hotmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_spi

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys_clock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>

#include "bf0_hal.h"

LOG_MODULE_REGISTER(spi_sf32lb, CONFIG_SPI_LOG_LEVEL);

//#define HAL_MAX_DELAY (1000)

#include "spi_context.h"

struct spi_sf32lb_config {
	SPI_TypeDef *reg;
	const struct device *clock_dev;
	const clock_control_subsys_t clock_subsys;
	uint32_t irq_num;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_SPI_SF32LB_INTERRUPT
	void (*irq_config_func)(const struct device *dev);
#endif
};

struct spi_sf32lb_data {
	struct spi_context ctx;
	uint32_t word_size;
	uint32_t pclk_freq;
	SPI_HandleTypeDef hspi;
};

static int spi_sf32lb_get_err(SPI_HandleTypeDef *hspi)
{
	uint32_t itflag = hspi->Instance->STATUS;
	uint32_t itsource = hspi->Instance->INTE;

	if (((itflag & (SPI_FLAG_OVR | SPI_FLAG_UDR)) != RESET) &&
		((itsource & SPI_IT_ERR) != RESET)) {
		return -EIO;
	}
}

static int spi_sf32lb_configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_sf32lb_config *cfg = dev->config;
	struct spi_sf32lb_data *data = dev->data;
	SPI_HandleTypeDef *hspi = &data->hspi;
#if 0
	if (clock_control_get_rate(cfg->clock_dev, cfg->clock_subsys, &data->pclk_freq) != 0) {
		LOG_ERR("Failed to get clock frequency");
		return -EIO;
	}
#endif
	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	hspi->Init.Mode =
		(config->operation & SPI_OP_MODE_SLAVE) ? SPI_MODE_SLAVE : SPI_MODE_MASTER;
	hspi->Init.Direction = SPI_DIRECTION_2LINES;
	hspi->Init.DataSize = (SPI_WORD_SIZE_GET(config->operation) == 8) ? SPI_DATASIZE_8BIT
									  : SPI_DATASIZE_16BIT;
	hspi->Init.CLKPolarity =
		(config->operation & SPI_MODE_CPOL) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
	hspi->Init.CLKPhase =
		(config->operation & SPI_MODE_CPHA) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;

	/* Calculate baud rate prescaler */
	// uint32_t br = 0;
	// uint32_t div = data->pclk_freq / config->frequency;
	/// while (div > (1 << (br + 1)) && br < 7) {
	//	br++;
	//}
	hspi->Init.BaudRatePrescaler = 0xC;
	if (SPI_FRAME_FORMAT_TI == (config->operation & SPI_FRAME_FORMAT_TI)) {
		hspi->Init.FrameFormat = SPI_FRAME_FORMAT_SSP;
	} else {
		hspi->Init.FrameFormat = SPI_FRAME_FORMAT_SPI;
	}
	hspi->Init.SFRMPol = SPI_SFRMPOL_HIGH;

	if (HAL_SPI_Init(hspi) != HAL_OK) {
		LOG_ERR("HAL_SPI_Init failed");
		return -EIO;
	}

	data->word_size = (SPI_WORD_SIZE_GET(config->operation) == 8) ? 1 : 2;

	data->ctx.config = config;
printk("TOP_CTRL 0x%x\n", hspi->Instance->TOP_CTRL);
printk("FIFO_CTRL 0x%x\n", hspi->Instance->FIFO_CTRL);
printk("INTE 0x%x\n", hspi->Instance->INTE);
printk("TO 0x%x\n", hspi->Instance->TO);
printk("DATA 0x%x\n", hspi->Instance->DATA);
printk("STATUS 0x%x\n", hspi->Instance->STATUS);
printk("PSP_CTRL 0x%x\n", hspi->Instance->PSP_CTRL);
printk("CLK_CTRL 0x%x\n", hspi->Instance->CLK_CTRL);
printk("TRIWIRE_CTRL 0x%x\n", hspi->Instance->TRIWIRE_CTRL);
	return 0;
}

static int spi_sf32lb_transceive_dma(const struct device *dev, const struct spi_config *config,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	struct spi_sf32lb_data *data = dev->data;
	int ret = 0;
#ifdef CONFIG_DCACHE
#endif
	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	//k_sem_reset(&data->status_sem);

	ret = spi_sf32lb_configure(dev, config);
	if (ret != 0) {
		return ret;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, data->word_size);

	spi_context_cs_control(&data->ctx, true);
}

static int spi_sf32lb_frame_exchange(const struct device *dev)
{
	struct spi_sf32lb_data *data = dev->data;
	const struct spi_sf32lb_config *cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	SPI_HandleTypeDef *hspi = &data->hspi;
	uint16_t tx_frame = 0U, rx_frame = 0U;

	if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
	{
		/*1line tx enable*/
		SPI_1LINE_TX(hspi);
	}

	/* Check if the SPI is already enabled */
	if ((hspi->Instance->TOP_CTRL & SPI_TOP_CTRL_SSE) != SPI_TOP_CTRL_SSE) {
		/* Enable SPI peripheral */
		__HAL_SPI_ENABLE(hspi);
	}

	if (SPI_WORD_SIZE_GET(ctx->config->operation) == 8) {
		if (spi_context_tx_buf_on(ctx) && __HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)) {
			tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
			*(__IO uint8_t *)(&hspi->Instance->DATA) = *((uint8_t *)&tx_frame);
			spi_context_update_tx(ctx, 1, 1);
		}
	} else {
		if (spi_context_tx_buf_on(ctx) && __HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)) {
			tx_frame = UNALIGNED_GET((uint16_t *)(data->ctx.tx_buf));
			data->hspi.Instance->DATA = tx_frame;
			spi_context_update_tx(ctx, 2, 1);
		}
	}

	if (SPI_WORD_SIZE_GET(ctx->config->operation) == 8) {
		if (spi_context_rx_buf_on(ctx) && __HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) {
			rx_frame = *(__IO uint8_t *)&hspi->Instance->DATA;
			UNALIGNED_PUT(rx_frame, (uint8_t *)data->ctx.rx_buf);
			spi_context_update_rx(ctx, 1, 1);
		}
	} else {
		if (spi_context_rx_buf_on(ctx) && __HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) {
			rx_frame = hspi->Instance->DATA;
			UNALIGNED_PUT(rx_frame, (uint16_t *)data->ctx.rx_buf);
			spi_context_update_rx(ctx, 2, 1);
		}
	}

	return 0;
}

static bool spi_sf32lb_transfer_ongoing(struct spi_sf32lb_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

static int spi_sf32lb_transceive(const struct device *dev, const struct spi_config *config,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	struct spi_sf32lb_data *data = dev->data;
	SPI_HandleTypeDef *hspi = &data->hspi;
	int ret = 0;

	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	ret = spi_sf32lb_configure(dev, config);
	if (ret != 0) {
		return ret;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, data->word_size);

	spi_context_cs_control(&data->ctx, true);

	size_t len = spi_context_total_tx_len(&data->ctx);
	uint8_t *tx_buf = (uint8_t *)data->ctx.tx_buf;
	uint8_t *rx_buf = (uint8_t *)data->ctx.rx_buf;
#ifdef CONFIG_SPI_SF32LB_INTERRUPT
	/* Enable TXE, RXNE and ERR interrupt */
	__HAL_SPI_ENABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

	/* Check if the SPI is already enabled */
	if ((hspi->Instance->TOP_CTRL & SPI_TOP_CTRL_SSE) != SPI_TOP_CTRL_SSE)
	{
		/* Enable SPI peripheral */
		__HAL_SPI_ENABLE(hspi);
	}
#else
	do {
		ret = spi_sf32lb_frame_exchange(dev);
		if (ret < 0) {
			break;
		}
	} while (spi_sf32lb_transfer_ongoing(data));
	/* Check the end of the transaction */
	/* FIXME: check hspi->Instance->STATUS; */
#endif
#if 0
	if (tx_buf && rx_buf) {
		if (HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, len, 1000) != HAL_OK) {
			goto out;
		}
	} else if (tx_buf) {
		if (HAL_SPI_Transmit(hspi, tx_buf, len, 1000) != HAL_OK) {
			goto out;
		}
	} else if (rx_buf) {
		if (HAL_SPI_Receive(hspi, rx_buf, len, 1000) != HAL_OK) {
			goto out;
		}
	}
#endif
out:
	spi_context_release(&data->ctx, ret);

	return ret;
}

#ifdef CONFIG_SPI_ASYNC
static int spi_sf32lb_transceive_async(const struct device *dev,
				     const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs,
				     spi_callback_t cb,
				     void *userdata)
{
	return spi_gd32_transceive_impl(dev, config, tx_bufs, rx_bufs, cb, userdata);
}
#endif

static int spi_sf32lb_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_sf32lb_data *data = dev->data;
	SPI_HandleTypeDef *hspi = &data->hspi;
	spi_context_unlock_unconditionally(&data->ctx);
	// HAL_SPI_DeInit(hspi);
	return 0;
}

static const struct spi_driver_api spi_sf32lb_api = {
	.transceive = spi_sf32lb_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_sf32lb_transceive_async,
#endif
	.release = spi_sf32lb_release,
};

#ifdef CONFIG_SPI_SF32LB_INTERRUPT
void spi_sf32lb_complete(const struct device *dev, int status)
{
	__HAL_SPI_CLEAR_OVRFLAG(hspi);
	__HAL_SPI_CLEAR_UDRFLAG(hspi);
	__HAL_SPI_DISABLE_IT(hspi, SPI_IT_RXNE | SPI_IT_TXE | SPI_IT_ERR);

	spi_context_complete(&data->ctx, dev, status);
}

static void spi_sf32lb_isr(const struct device *dev)
{
	struct spi_sf32lb_data *data = dev->data;
	SPI_HandleTypeDef *hspi = &data->hspi;
	int err = 0;

	err = spi_sf32lb_get_err(hspi);
	if (err) {
		spi_sf32lb_complete(dev, err);
		return;
	}

	if (spi_sf32lb_transfer_ongoing(data)) {
		err = spi_sf32lb_frame_exchange(dev);
	}

	if (err) {
		spi_sf32lb_complete(dev, err);
	}

	if (err || !spi_sf32lb_transfer_ongoing(data)) {
		spi_sf32lb_complete(dev, err);
	}
}
#endif

static int spi_sf32lb_init(const struct device *dev)
{
	const struct spi_sf32lb_config *cfg = dev->config;
	struct spi_sf32lb_data *data = dev->data;
	SPI_HandleTypeDef *hspi = &data->hspi;
	int err = 0;

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("Clock control device not ready");
		return -ENODEV;
	}

	err = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (err != 0) {
		LOG_ERR("Failed to enable clock");
		return err;
	}

	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		LOG_ERR("Failed to set pinctrl");
		return err;
	}

	hspi->Instance = cfg->reg;

	err = spi_context_cs_configure_all(&data->ctx);
	if (err < 0) {
		return err;
	}

	spi_context_unlock_unconditionally(&data->ctx);
#ifdef CONFIG_SPI_SF32LB_INTERRUPT
	cfg->irq_config_func(dev);
#endif
	LOG_INF("%s Initialization OK", dev->name);

	return err;
}

#ifdef CONFIG_SPI_SF32LB_INTERRUPT
#define SF32LB_SPI_IRQ_HANDLER_DECL(id)	\
	static void spi_sf32lb_irq_config_func_##id(const struct device *dev);

#define SF32LB_SPI_IRQ_HANDLER_FUNC(id) \
	.irq_config_func = spi_sf32lb_irq_config_func_##id,

#define SF32LB_SPI_IRQ_HANDLER(id) \
static void spi_sf32lb_irq_config_func_##id(const struct device *dev) \
{ \
	IRQ_CONNECT(DT_INST_IRQN(id), \
		    DT_INST_IRQ(id, priority), \
		    spi_sf32lb_isr, DEVICE_DT_INST_GET(id), 0); \
	irq_enable(DT_INST_IRQN(id)); \
}
#else
#define SF32LB_SPI_IRQ_HANDLER_DECL(id)
#define SF32LB_SPI_IRQ_HANDLER_FUNC(id)
#define SF32LB_SPI_IRQ_HANDLER(id)
#endif /* CONFIG_SPI_SF32LB_INTERRUPT */

#define SPI_SF32LB_INIT(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static struct spi_sf32lb_data spi_sf32lb_data_##n = {                                      \
		SPI_CONTEXT_INIT_LOCK(spi_sf32lb_data_##n, ctx),                                   \
		SPI_CONTEXT_INIT_SYNC(spi_sf32lb_data_##n, ctx),                                   \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
                                                                                                   \
	static const struct spi_sf32lb_config spi_sf32lb_config_##n = {                            \
		.reg = (SPI_TypeDef *)DT_INST_REG_ADDR(n),                                         \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id),                \
		.irq_num = DT_INST_IRQN(n),                                                        \
		SF32LB_SPI_IRQ_HANDLER_FUNC(n)                                                     \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, spi_sf32lb_init, NULL, &spi_sf32lb_data_##n,                      \
			      &spi_sf32lb_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,       \
			      &spi_sf32lb_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_SF32LB_INIT)
