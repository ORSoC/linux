/*
 * spi_opencores.c -- OpenCores SPI controller driver
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
#include <asm/cpuinfo.h>

#define DRIVER_NAME			"oc_spi"

#define OCSPI_NUM_CHIPSELECTS		8 

#define OCSPI_REG_DATA0			0x00
#define OCSPI_REG_DATA1			0x04
#define OCSPI_REG_DATA2			0x08
#define OCSPI_REG_DATA3			0x0c
#define OCSPI_REG_CTRL			0x10
#define OCSPI_REG_DIVIDER		0x14
#define OCSPI_REG_SS			0x18

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

	/* deassert any old CS. !cs to reset if hw and internal state mismatch */
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
		else if (wordlen == 2)
			txdata[0] = *(u16 *)txbuf;
		else
			memcpy(txdata, txbuf, wordlen);
		txbuf += wordlen;
		*txbufp = txbuf;
	}
	ocspi_write(hw, OCSPI_REG_DATA0, txdata[0]);
	ocspi_write(hw, OCSPI_REG_DATA1, txdata[1]);
	ocspi_write(hw, OCSPI_REG_DATA2, txdata[2]);
	ocspi_write(hw, OCSPI_REG_DATA3, txdata[3]);
}

static void ocspi_read_rx(struct ocspi *hw, void **rxbufp, int wordlen)
{
	u8 *rxbuf = *rxbufp;
	u32 rxdata[4];

	rxdata[0] = ocspi_read(hw, OCSPI_REG_DATA0);
	rxdata[1] = ocspi_read(hw, OCSPI_REG_DATA1);
	rxdata[2] = ocspi_read(hw, OCSPI_REG_DATA2);
	rxdata[3] = ocspi_read(hw, OCSPI_REG_DATA3);
	if (rxbuf) {
		if (wordlen == 1)
			*(u8 *)rxbuf = rxdata[0];
		else if (wordlen == 2)
			*(u16 *)rxbuf = rxdata[0];
		else
			memcpy(rxbuf, rxdata, wordlen);
		rxbuf += wordlen;
		*rxbufp = rxbuf;
	}
}

static int ocspi_busy(struct ocspi *hw)
{
	return ocspi_read(hw, OCSPI_REG_CTRL) & OCSPI_CTRL_GO_BSY;
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
		const void *txbuf = t->tx_buf;
		void *rxbuf = t->rx_buf;
		u32 speed_hz = t->speed_hz ? : spi->max_speed_hz;
		u8 bits_per_word = t->bits_per_word ? : spi->bits_per_word;
		int ctrl = 0;
		int len = t->len;
		int wordlen;

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

		/* is this needed? I think spi->bits_per_word is always set */
		bits_per_word = bits_per_word ? : 8;

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
			err = wait_event_interruptible(hw->wait, !ocspi_busy(hw));
			if (err)
				goto out;
			ocspi_read_rx(hw, &rxbuf, wordlen);
			len -= wordlen;
			m->actual_length += wordlen;
		}
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
		if (t->bits_per_word < 0 || t->bits_per_word > 128)
			return -EINVAL;

		if (t->len == 0)
			return -EINVAL;
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
		dev_err(&pdev->dev, "platform_get_irq error\n");
		return -ENODEV;
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
	
	error = devm_request_irq(&pdev->dev, irq, ocspi_interrupt, 0,
				       pdev->name, hw);
	if (error < 0) {
		dev_err(&pdev->dev, "request_irq failed\n");
		goto out;
	}
	
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST; /* SPI_NO_CS? */
	master->bus_num = pdev->id;
	if (master->bus_num == -1)
	       master->bus_num = (memres->start >> 24) & 0x4;
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

	printk(KERN_INFO "OpenCores SPI controller (c) 2013 ORSoC AB\n");

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
                .compatible = "opencores,spi",
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
