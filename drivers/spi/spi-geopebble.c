/*
 * spi-geopebble.c -- Geopebble ADC SPI controller driver
 *
 * Author: Henrik Nordstrom <henrik@henriknordstrom.net>
 * Copyright (C) 2013 ORSoC AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/wishbone.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <asm/cpuinfo.h>

#define DRIVER_NAME			"oc_spi_geopebble"

#define OCSPI_NUM_CHIPSELECTS		8 

#define OCSPI_REG_DATA0			0x00
#define OCSPI_REG_DATA1			0x04
#define OCSPI_REG_DATA2			0x08
#define OCSPI_REG_DATA3			0x0c
#define OCSPI_REG_CTRL			0x10
#define OCSPI_REG_DIVIDER		0x14
#define OCSPI_REG_SS			0x18
#define OCSPI_REG_DMA_EXECUTE		0x20
#define OCSPI_REG_DMA_CURRENT_ADDR	0x24
#define OCSPI_REG_DMA_CURRENT_CNT	0x28
#define OCSPI_REG_DMA_CLRIRQ		0x2c
#define OCSPI_REG_DMA_ADDR		0x30
#define OCSPI_REG_DMA_CNT		0x34

/* CTRL bits */
/* word len field is 1-128(0) bits (docs also 64? confusing) */
#define OCSPI_CTRL_LEN(n)		((n)&0x7f)
#define OCSPI_CTRL_GET_LEN(r)		({int __r = (r)&0x7f; __r ? __r : 128})
#define OCSPI_CTRL_GO_BSY		(1 << 8)
#define OCSPI_CTRL_Rx_NEG		(1 << 9)
#define OCSPI_CTRL_Tx_NEG		(1 << 10)
#define OCSPI_CTRL_LSB			(1 << 11)
#define OCSPI_CTRL_IE			(1 << 12)
#define OCSPI_CTRL_ASS			(1 << 13)
#define OCSPI_CTRL_CPOL			(1 << 14)

#define INVALID_DMA_ADDRESS	0xffffffff

struct ocspi {
	struct spi_master	*master;

	/* pending work */
	struct list_head	queue;
	spinlock_t		lock;

	/* worker thread */
	struct workqueue_struct	*workqueue;
	struct work_struct	work;
	wait_queue_head_t wait;

	/* Hardware */
	void __iomem		*base;
	int			current_cs;

	/* limits */
	unsigned int		max_speed;
	unsigned int		min_speed;
	unsigned int		polled_mode;
};

static u32
ocspi_read(struct ocspi* hw, unsigned int reg) {
	u32 v = ioread32be(hw->base + reg);
	return v;
}

static void
ocspi_write(struct ocspi* hw, unsigned int reg, u32 value) {
	iowrite32be(value, hw->base + reg);
}

static irqreturn_t ocspi_interrupt(int irq, void *dev_id)
{
	struct ocspi *hw = dev_id;

	/* deassert interrupt by reading any register */
	(void)ocspi_read(hw, OCSPI_REG_CTRL);

	wake_up(&hw->wait);
	return IRQ_HANDLED;
}

static void ocspi_set_cs(struct ocspi *hw, int cs, int cs_delay)
{
	if (cs && hw->current_cs == cs)
		return;

	/* deassert any old CS */
	if (cs < 0 || hw->current_cs >= 0) {
		ocspi_write(hw, OCSPI_REG_SS, 0);
		ndelay(cs_delay);
	}

	/* assert CS */
	if (cs >= 0) {
		ocspi_write(hw, OCSPI_REG_SS, 1<<cs);
		ndelay(cs_delay);
	}
	hw->current_cs = cs;
}

static void ocspi_clear_cs(struct ocspi *hw, int cs_delay)
{
	ocspi_set_cs(hw, -1, cs_delay);
}

static void ocspi_fill_tx(struct ocspi *hw, const void **txbufp, int wordlen)
{
	const u8 *txbuf = *txbufp;
	u32 txdata[4] = { 0, 0, 0, 0 };

	if (txbuf) {
		if (wordlen == 1)
			txdata[0] = *(u8 *)txbuf;
		else if (wordlen == 2) {
			u16 t;
			memcpy(&t, txbuf, wordlen);
			txdata[0] = t;
		} else {
			memcpy(txdata, txbuf, wordlen);
		}
		txbuf += wordlen;
		*txbufp = txbuf;
	}
	switch(wordlen) {
	case 1:
	case 2:
	case 4:
		ocspi_write(hw, OCSPI_REG_DATA0, txdata[0]);
		break;
	case 8:
		ocspi_write(hw, OCSPI_REG_DATA1, txdata[0]);
		ocspi_write(hw, OCSPI_REG_DATA0, txdata[1]);
		break;
	case 16:
		ocspi_write(hw, OCSPI_REG_DATA3, txdata[0]);
		ocspi_write(hw, OCSPI_REG_DATA2, txdata[1]);
		ocspi_write(hw, OCSPI_REG_DATA1, txdata[2]);
		ocspi_write(hw, OCSPI_REG_DATA0, txdata[3]);
		break;
	}
}

static void ocspi_read_rx(struct ocspi *hw, void **rxbufp, int wordlen)
{
	u8 *rxbuf = *rxbufp;
	u32 rxdata[4];

	switch (wordlen) {
	case 1: case 2: case 4:
		rxdata[0] = ocspi_read(hw, OCSPI_REG_DATA0);
		break;
	case 8:
		rxdata[0] = ocspi_read(hw, OCSPI_REG_DATA1);
		rxdata[1] = ocspi_read(hw, OCSPI_REG_DATA0);
		break;
	case 16:
		rxdata[0] = ocspi_read(hw, OCSPI_REG_DATA3);
		rxdata[1] = ocspi_read(hw, OCSPI_REG_DATA2);
		rxdata[2] = ocspi_read(hw, OCSPI_REG_DATA1);
		rxdata[3] = ocspi_read(hw, OCSPI_REG_DATA0);
		break;
	}
	if (rxbuf) {
		if (wordlen == 1)
			*(u8 *)rxbuf = rxdata[0];
		else if (wordlen == 2) {
			u16 t = rxdata[0];
			memcpy(rxbuf, &t, wordlen);
		} else {
			memcpy(rxbuf, rxdata, wordlen);
		}
		rxbuf += wordlen;
		*rxbufp = rxbuf;
	}
}

static int ocspi_busy(struct ocspi *hw)
{
	return ocspi_read(hw, OCSPI_REG_CTRL) & OCSPI_CTRL_GO_BSY;
}

static int ocspi_work_one_xfr(struct ocspi *hw, struct spi_message *m, struct spi_transfer *t)
{
	struct spi_device *spi = m->spi;
	const void *txbuf = t->tx_buf;
	void *rxbuf = t->rx_buf;
	u32 speed_hz = t->speed_hz ? : spi->max_speed_hz;
	u8 bits_per_word = t->bits_per_word ? : spi->bits_per_word;
	int ctrl = 0;
	int len = t->len;
	int wordlen;
	unsigned int cs_delay = 100 + (NSEC_PER_SEC / 2) / spi->max_speed_hz;
	int err = 0;

	if (bits_per_word <= 8)
		wordlen = 1;
	else if (bits_per_word <= 16)
		wordlen = 2;
	else if (bits_per_word <= 32)
		wordlen = 4;
	else if (bits_per_word <= 64)
		wordlen = 8;
	else
		wordlen = 16;

	ocspi_write(hw, OCSPI_REG_DIVIDER,
			DIV_ROUND_UP(cpuinfo.clock_frequency, speed_hz * 2) - 1);
	ctrl = OCSPI_CTRL_LEN(bits_per_word);
	ctrl |= OCSPI_CTRL_Rx_NEG;
	if (spi->mode & SPI_CPOL)
		ctrl |= OCSPI_CTRL_CPOL;
	if (spi->mode & SPI_CPHA)
		ctrl ^= OCSPI_CTRL_Rx_NEG | OCSPI_CTRL_Tx_NEG;
	if (spi->mode & SPI_LSB_FIRST)
		ctrl |= OCSPI_CTRL_LSB;
	ctrl |= OCSPI_CTRL_IE;	/* interrupt on completion */
	ocspi_write(hw, OCSPI_REG_CTRL, ctrl);
	len = t->len;
	while (len > 0) {
		ocspi_fill_tx(hw, &txbuf, wordlen);
		ocspi_set_cs(hw, spi->chip_select, cs_delay);
		ctrl |= OCSPI_CTRL_GO_BSY;
		ocspi_write(hw, OCSPI_REG_CTRL, ctrl);
		if (hw->polled_mode) {
			/* TOOD: Make this interruptible */
			err = 0;
			while(ocspi_busy(hw));
		} else {
			err = wait_event_interruptible(hw->wait, !ocspi_busy(hw));
		}
		if (err)
			return err;
		ocspi_read_rx(hw, &rxbuf, wordlen);
		len -= wordlen;
		m->actual_length += wordlen;
	}
	return 0;
}

/*
 * For DMA, tx_buf/tx_dma have the same relationship as rx_buf/rx_dma:
 *  - The buffer is either valid for CPU access, else NULL
 *  - If the buffer is valid, so is its DMA address
 *
 * This driver manages the dma address unless message->is_dma_mapped.
 */
static int
ocspi_dma_map_xfer(struct ocspi *hw, struct spi_transfer *xfer)
{
	struct device	*dev = hw->master->dev.parent;

	xfer->tx_dma = xfer->rx_dma = INVALID_DMA_ADDRESS;
	if (xfer->tx_buf) {
		/* tx_buf is a const void* where we need a void * for the dma
		 * mapping */
		void *nonconst_tx = (void *)xfer->tx_buf;

		xfer->tx_dma = dma_map_single(dev,
				nonconst_tx, xfer->len,
				DMA_TO_DEVICE);
		if (dma_mapping_error(dev, xfer->tx_dma))
			return -ENOMEM;
	}
	if (xfer->rx_buf) {
		xfer->rx_dma = dma_map_single(dev,
				xfer->rx_buf, xfer->len,
				DMA_FROM_DEVICE);
		if (dma_mapping_error(dev, xfer->rx_dma)) {
			if (xfer->tx_buf)
				dma_unmap_single(dev,
						xfer->tx_dma, xfer->len,
						DMA_TO_DEVICE);
			return -ENOMEM;
		}
	}
	return 0;
}

static void ocspi_dma_unmap_xfer(struct ocspi *hw,
				     struct spi_transfer *xfer)
{
	struct device	*dev = hw->master->dev.parent;

	if (xfer->tx_dma != INVALID_DMA_ADDRESS)
		dma_unmap_single(dev, xfer->tx_dma,
				 xfer->len, DMA_TO_DEVICE);
	if (xfer->rx_dma != INVALID_DMA_ADDRESS)
		dma_unmap_single(dev, xfer->rx_dma,
				 xfer->len, DMA_FROM_DEVICE);
}


static int ocspi_work_one_dma(struct ocspi *hw, struct spi_message *m, struct spi_transfer *t)
{
	struct spi_device *spi = m->spi;
	const void *txbuf = t->tx_buf;
	void *rxbuf = t->rx_buf;
	u32 speed_hz = t->speed_hz ? : spi->max_speed_hz;
	u8 bits_per_word = t->bits_per_word ? : spi->bits_per_word;
	int ctrl = 0;
	int len = t->len;
	int wordlen;
	unsigned int cs_delay = 100 + (NSEC_PER_SEC / 2) / spi->max_speed_hz;
	int err = 0;

	if (bits_per_word != 192)
		return -EINVAL;

	if (txbuf)
		return -EINVAL; /* Only reads supported */

	bits_per_word = 128; /* SPI part is only 128 bits. Followed by 64 bits counters */

	ocspi_write(hw, OCSPI_REG_DIVIDER,
			DIV_ROUND_UP(cpuinfo.clock_frequency, speed_hz * 2) - 1);
	ctrl = OCSPI_CTRL_LEN(bits_per_word);
	ctrl |= OCSPI_CTRL_Rx_NEG;
	if (spi->mode & SPI_CPOL)
		ctrl |= OCSPI_CTRL_CPOL;
	if (spi->mode & SPI_CPHA)
		ctrl ^= OCSPI_CTRL_Rx_NEG | OCSPI_CTRL_Tx_NEG;
	if (spi->mode & SPI_LSB_FIRST)
		ctrl |= OCSPI_CTRL_LSB;
	ocspi_write(hw, OCSPI_REG_CTRL, ctrl);

	ocspi_set_cs(hw, spi->chip_select, cs_delay);

	/* Start DMA */
	ocspi_write(hw, OCSPI_REG_DMA_ADDR, t->rx_dma);
	ocspi_write(hw, OCSPI_REG_DMA_CNT, t->len / 24);
	ocspi_write(hw, OCSPI_REG_DMA_EXECUTE, 1);

	if (hw->polled_mode) {
		/* TOOD: Make this interruptible */
		err = 0;
		while(ocspi_read(hw, OCSPI_REG_DMA_CURRENT_CNT) != 0);
	} else {
		err = wait_event_interruptible(hw->wait, !(ocspi_read(hw, OCSPI_REG_DMA_CURRENT_CNT) != 0));
	}

	if (!m->is_dma_mapped)
		ocspi_dma_unmap_xfer(hw, t);

	len = t->len;
	while (len > 0) {
		ocspi_fill_tx(hw, &txbuf, wordlen);
		if (err)
			return err;
		ocspi_read_rx(hw, &rxbuf, wordlen);
		len -= wordlen;
		m->actual_length += wordlen;
	}
	return 0;
}

static void ocspi_work_one(struct ocspi *hw, struct spi_message *m)
{
	struct spi_device *spi = m->spi;
	struct spi_transfer *t;
	unsigned int cs_delay = 100 + (NSEC_PER_SEC / 2) / spi->max_speed_hz;
	int err = 0;

	if (ocspi_busy(hw)) {
		/* Hardware locked up somehow. Can't access it. */
		dev_err(&spi->dev, "hardware busy\n");
		err = -EBUSY;
		goto out;
	}

	list_for_each_entry (t, &m->transfers, transfer_list) {
		u8 bits_per_word = t->bits_per_word ? : spi->bits_per_word;
		if (bits_per_word == 192)
			err = ocspi_work_one_dma(hw, m, t);
		else
			err = ocspi_work_one_xfr(hw, m, t);
		if (err)
			goto out;
		if (t->cs_change)
			ocspi_clear_cs(hw, cs_delay);
	}
out:
	m->status = err;
	m->complete(m->context);
	ocspi_clear_cs(hw, cs_delay);
}

static void ocspi_work(struct work_struct *work)
{
	struct ocspi *hw = container_of(work, struct ocspi, work);
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);
	while (!list_empty(&hw->queue)) {
		struct spi_message *m = list_entry(hw->queue.next, struct spi_message, queue);
		list_del_init(&m->queue);

		spin_unlock_irqrestore(&hw->lock, flags);
		ocspi_work_one(hw, m);
		spin_lock_irqsave(&hw->lock, flags);
	}

	spin_unlock_irqrestore(&hw->lock, flags);
}

/*
 * This is a bit misnamed. Actual setup is done when starting the transfer.
 * Can only verify basic device protocol settings here
 */
static int ocspi_setup(struct spi_device *spi)
{
	struct ocspi *hw = spi_master_get_devdata(spi->master);

	if ((spi->max_speed_hz == 0)
			|| (spi->max_speed_hz > hw->max_speed))
		spi->max_speed_hz = hw->max_speed;

	if (spi->max_speed_hz < hw->min_speed) {
		dev_err(&spi->dev, "setup: requested speed too low %d Hz\n",
			spi->max_speed_hz);
		return -EINVAL;
	}

	return 0;
}

/* Queue a message chain for transmission */
static int ocspi_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct ocspi *hw = spi_master_get_devdata(spi->master);
	struct spi_transfer *t = NULL;
	unsigned long flags;

	if (list_empty(&m->transfers) || !m->complete)
		return -EINVAL;

	list_for_each_entry(t, &m->transfers, transfer_list) {
		if ((t->bits_per_word < 0 || t->bits_per_word > 128) && t->bits_per_word != 192)
			return -EINVAL;

		if (t->len == 0)
			return -EINVAL;

		if (t->bits_per_word == 192) {
			/*
			 * DMA map early, for performance (empties dcache ASAP) and
			 * better fault reporting.  This is a DMA-only driver.
			 *
			 * NOTE that if dma_unmap_single() ever starts to do work on
			 * platforms supported by this driver, we would need to clean
			 * up mappings for previously-mapped transfers.
			 */
			if (!m->is_dma_mapped) {
				if (ocspi_dma_map_xfer(hw, t) < 0)
					return -ENOMEM;
			}
		}
        }

	/* This is not needed here. Can go in the worker when starting message */
	m->actual_length = 0;
	m->status = 0;

	spin_lock_irqsave(&hw->lock, flags);
	list_add_tail(&m->queue, &hw->queue);
	queue_work(hw->workqueue, &hw->work);
	spin_unlock_irqrestore(&hw->lock, flags);

	return 0;
}

static int __devinit ocspi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct ocspi *hw;
	struct resource *memres;
	int irq;
	int error = 0;

	/* Get hardware resources */
	memres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (memres == NULL) {
		dev_err(&pdev->dev, "invalid resource\n");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_info(&pdev->dev, "no SPI controller IRQ defined. Using polled mode\n");
	}

	master = spi_alloc_master(&pdev->dev, sizeof *hw);
	if (master == NULL) {
		dev_err(&pdev->dev, "spi_alloc_master failed\n");
		return -ENOMEM;
	}
	hw = spi_master_get_devdata(master);
	dev_set_drvdata(&pdev->dev, hw);

	hw->master = master;

	if (!devm_request_mem_region(&pdev->dev, memres->start,
				     resource_size(memres),
		                     dev_name(&pdev->dev))) {
		error = -EBUSY;
		goto out;
	}
	hw->base = devm_ioremap_nocache(&pdev->dev, memres->start,
					 resource_size(memres));
	INIT_LIST_HEAD(&hw->queue);
	spin_lock_init(&hw->lock);
	INIT_WORK(&hw->work, ocspi_work);
	init_waitqueue_head(&hw->wait);
	hw->workqueue = create_singlethread_workqueue(
					dev_name(master->dev.parent));
	if (hw->workqueue == NULL) {
		dev_err(&pdev->dev, "create_singlethread_workqueue failed\n");
		error = -ENOMEM;
		goto out;
	}
	
	if (irq >= 0) {
		error = devm_request_irq(&pdev->dev, irq, ocspi_interrupt, 0,
					       pdev->name, hw);
		if (error < 0) {
			dev_err(&pdev->dev, "request_irq failed\n");
			goto out;
		}
		hw->polled_mode = 0;
	} else {
		hw->polled_mode = 1;
	}
	
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST;
	master->bus_num = pdev->id;
printk("bus_num = %d (pdev)\n", master->bus_num);
	if (master->bus_num == -1) {
printk("memres = %X\n", (unsigned)memres->start);
		/* Construct bus number based on low bits of base address */
		master->bus_num = (memres->start >> 24) & 0x3;
printk("bus_num = %d (memres)\n", master->bus_num);
	}
	master->num_chipselect = OCSPI_NUM_CHIPSELECTS;
	master->setup = ocspi_setup;
	master->transfer = ocspi_transfer;
	master->dev.of_node = pdev->dev.of_node;

	dev_set_drvdata(&pdev->dev, master);

	/* This is wrong. Should use wishbone bus frequency. */
	hw->max_speed = cpuinfo.clock_frequency >> 1;
	hw->min_speed = cpuinfo.clock_frequency >> 16;

	/* Make sure any SS is deasserted in idle state */
	ocspi_clear_cs(hw, 0);

	error = spi_register_master(master);
	if (error)
		goto out;

	printk(KERN_INFO "Geopebble SPI ADC controller (c) 2013 ORSoC AB\n");

	return 0;

out:
	if (hw->workqueue)
		destroy_workqueue(hw->workqueue);
	spi_master_put(master);
	return error;
}

static int __devexit ocspi_remove(struct platform_device *pdev)
{
	struct spi_master *master = dev_get_drvdata(&pdev->dev);
	struct ocspi *hw = spi_master_get_devdata(master);

	cancel_work_sync(&hw->work);

	spi_unregister_master(master);
	destroy_workqueue(hw->workqueue);

	return 0;
}

static struct of_device_id ocspi_match[] = {
        {
                .compatible = "opencores,spi-geopebble",
        },
        {},
};
MODULE_DEVICE_TABLE(of, ocspi_match);

static struct platform_driver ocspi_driver = {
	.probe = ocspi_probe,
	.remove = __devexit_p(ocspi_remove),
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = ocspi_match
	}
};

static int __init ocspi_init(void)
{
	return platform_driver_register(&ocspi_driver);
}
module_init(ocspi_init);

static void __exit ocspi_exit(void)
{
	platform_driver_unregister(&ocspi_driver);
}
module_exit(ocspi_exit);

MODULE_DESCRIPTION("OpenCores SPI driver");
MODULE_AUTHOR("Henrik Nordstrom <henrik@henriknordstrom.net>");
MODULE_LICENSE("GPL");
