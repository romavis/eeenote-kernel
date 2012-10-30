/*
 * linux/drivers/mmc/host/pxamci_multi.c - PXA3xx MCI driver 
 * with multiple cards per controller support
 *
 * Based on original pxamci driver written by Russell King
 * 
 * Also it supports multi-descriptor DMA transfers
 * and returning correct bytes_xfered value
 * 
 * Copyright (C) 2012 Roman Dobrodiy
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/gfp.h>
#include <linux/slab.h>
#include <linux/semaphore.h>

#include <asm/sizes.h>

#include <mach/hardware.h>
#include <mach/dma.h>
#include <mach/mmc-multi.h>
#include <mach/pxa3xx-regs.h>

#include "pxamci.h"
#include "pxamci_multi.h"

#define DRIVER_NAME	"pxamci-multi"

#define CLKRT_OFF	(~0)

/* Timeouts in jiffies */
#define BUSY_TIMEOUT	50
#define CLKOFF_TIMEOUT	5
				
/* 
 * Max number of segments(descriptors) for DMA:
 * We allocate PAGE_SIZE-sized chunk of space for descriptors
 * each descriptor takes 16 bytes => divide PAGE_SIZE by 16
 */
#define MAX_DMA_SEGS		(PAGE_SIZE >> 4)
#define MAX_DMA_SEG_SIZE	8191
#define MAX_MMC_BLK_SIZE	2048
#define MAX_MMC_BLKS		65535

struct pxamci_multi_host {
	struct mmc_host		**hosts;

	struct pxamci_multi_slot **slots;
	unsigned int		slot_count;
	unsigned int		slot_selected;	/* Currently selected slot */
	
	spinlock_t		lock;
	struct semaphore	slot_lock;
	wait_queue_head_t	wq;
	
	/* Flag set by IRQ */
	unsigned int		irq_flag;
	unsigned int		cmd_stat;
	unsigned int		data_stat;
	
	/* Hardware */
	struct resource		*res;
	void __iomem		*base;
	struct clk		*clk;
	unsigned long		clkrate;
	unsigned int		clock_enabled;
	
	int			irq;
	int			dma;
	unsigned int		imask;
	
	/* Device struct */
	struct pxamci_multi_platform_data *pdata;
	struct device		*dev;
	
	/* DMA related */
	dma_addr_t		sg_dma;
	struct pxa_dma_desc	*sg_cpu;
	//unsigned int		dma_len;

	unsigned int		dma_dir;
	unsigned int		dma_drcmrrx;
	unsigned int		dma_drcmrtx;
};

/* 
 * Slot related data
 * This resides in mmc_host's private field
 */
struct pxamci_multi_slot {
	struct pxamci_multi_host *host;
	unsigned int id;			/* Slot index */
	
	unsigned int		power_mode;	/* Slot power mode */
	unsigned int		clkrt;		/* Slot CLKRT register */
	unsigned int		cmdat;		/* Slot command attributes */
	
	unsigned int		desired_clock;	/* set_ios clock */
	unsigned int		real_clock;	/* real supplied clock */
};

static inline int pxamci_multi_set_power(struct pxamci_multi_host *host,
				unsigned int on)
{
	if (gpio_is_valid(host->pdata->gpio_power))
		gpio_set_value(host->pdata->gpio_power,\
			       !!on ^ host->pdata->gpio_power_invert);
	if (host->pdata->setpower)
		host->pdata->setpower(host->dev, on);

	return 0;
}

static void pxamci_multi_enable_irq(struct pxamci_multi_host *host,\
				unsigned int mask)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->imask &= ~mask;
	writel(host->imask, host->base + MMC_I_MASK);
	spin_unlock_irqrestore(&host->lock, flags);
}

static void pxamci_multi_disable_irq(struct pxamci_multi_host *host, unsigned int mask)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->imask |= mask;
	writel(host->imask, host->base + MMC_I_MASK);
	spin_unlock_irqrestore(&host->lock, flags);
}

/*
 * Setup DMA for data transfer
 */
static void pxamci_multi_setup_data(struct pxamci_multi_host *host,\
				struct mmc_data *data)
{
	unsigned int nob = data->blocks;
	unsigned long long clks;
	unsigned int timeout;
	bool not_aligned = 0;
	bool clipped_seg = 0;
	u32 dcmd;
	unsigned int dma_len;
	unsigned int desc, seg;
	unsigned int length;
	unsigned int addr;
	
	if (data->flags & MMC_DATA_STREAM)
		nob = 0x1;

	writel(nob, host->base + MMC_NOB);
	writel(data->blksz, host->base + MMC_BLKLEN);

	if (data->flags & MMC_DATA_READ) {
		host->dma_dir = DMA_FROM_DEVICE;
		dcmd = DCMD_INCTRGADDR | DCMD_FLOWSRC;
		DRCMR(host->dma_drcmrtx) = 0;
		DRCMR(host->dma_drcmrrx) = host->dma | DRCMR_MAPVLD;
		/* Also take into account a read timeout */
		clks = (unsigned long long)data->timeout_ns * host->clkrate;
		do_div(clks, 1000000000UL);
		timeout = (unsigned int)clks + (data->timeout_clks <<\
				host->slots[host->slot_selected]->clkrt);
		writel((timeout + 255) / 256, host->base + MMC_RDTO);
	} else {
		host->dma_dir = DMA_TO_DEVICE;
		dcmd = DCMD_INCSRCADDR | DCMD_FLOWTRG;
		DRCMR(host->dma_drcmrrx) = 0;
		DRCMR(host->dma_drcmrtx) = host->dma | DRCMR_MAPVLD;
	}

	dcmd |= DCMD_BURST32 | DCMD_WIDTH1;

	dma_len = dma_map_sg(host->dev, data->sg, data->sg_len,
				   host->dma_dir);
	/* 
	 * dma_map_sg may merge scatterlist segments in rare cases
	 * We should check for that, as resulting segment size can be
	 * larger that MAX_DMA_SEG_SIZE
	 */
	for (seg = 0, desc = 0; seg < dma_len; desc++) {
		
		if (clipped_seg) {
			addr += MAX_DMA_SEG_SIZE;
			length = sg_dma_address(&data->sg[seg]) +\
				sg_dma_len(&data->sg[seg]) - addr;
				
			if(length <= MAX_DMA_SEG_SIZE)
				clipped_seg = 0;
		} else {
			length = sg_dma_len(&data->sg[seg]);
			addr = sg_dma_address(&data->sg[seg]);
			
			if(length > MAX_DMA_SEG_SIZE) {
				clipped_seg = 1;
				length = MAX_DMA_SEG_SIZE;
			}
		}
		
		host->sg_cpu[desc].dcmd = dcmd | length;
		
		/* Partially filled TXFIFO - explicit swapping required */
		if ((length & 31) && (data->flags & MMC_DATA_WRITE))
			host->sg_cpu[desc].dcmd |= DCMD_ENDIRQEN;

		/* Not aligned to 8-byte boundary? */
		if (addr & 0x7)
			not_aligned = 1;
		
		if (data->flags & MMC_DATA_READ) {
			host->sg_cpu[desc].dsadr = host->res->start +\
						   MMC_RXFIFO;
			host->sg_cpu[desc].dtadr = addr;
		} else {
			host->sg_cpu[desc].dsadr = addr;
			host->sg_cpu[desc].dtadr = host->res->start +\
						   MMC_TXFIFO;
		}
		host->sg_cpu[desc].ddadr = host->sg_dma + (desc + 1) *
					   sizeof(struct pxa_dma_desc);
		if(!clipped_seg)
			seg++;
	}
	host->sg_cpu[desc - 1].ddadr = DDADR_STOP;
	wmb();

	/*
	 * The PXA DMA controller encounters overhead when working with
	 * unaligned (to 8-byte boundaries) data, so switch on byte alignment
	 * mode only if we have unaligned data.
	 */
	if (not_aligned)
		DALGN |= (1 << host->dma);
	else
		DALGN &= ~(1 << host->dma);
	
	/* Start DMA channel */
	DDADR(host->dma) = host->sg_dma;
	DCSR(host->dma) = DCSR_RUN;
}

static void pxamci_multi_dma_irq(int dma, void *devid)
{
	struct pxamci_multi_host *host = devid;
	int dcsr = DCSR(dma);
	DCSR(dma) = dcsr & ~DCSR_STOPIRQEN;
	
	if (dcsr & DCSR_ENDINTR) {
		writel(BUF_PART_FULL, host->base + MMC_PRTBUF);
	} else {
		dev_err(host->dev, "DMA error on channel %d (DCSR=%#x)\n",
			dma, dcsr);
		host->irq_flag |= MCI_FLAG_DATA_DONE;
		host->data_stat = STAT_READ_TIME_OUT;
		wake_up_interruptible(&host->wq);
	}
}

static irqreturn_t pxamci_multi_detect_irq(int irq, void *devid)
{
	struct pxamci_multi_slot *slot = mmc_priv(devid);
	struct pxamci_multi_host *host = slot->host;
	
	mmc_detect_change(devid,\
			msecs_to_jiffies(host->pdata->detect_delay_ms));
	return IRQ_HANDLED;
}

static irqreturn_t pxamci_multi_irq(int irq, void *devid)
{
	struct pxamci_multi_host *host = devid;
	unsigned int ireg;
	unsigned int stat;
	int wake = 0;
	
	ireg = readl(host->base + MMC_I_REG) & ~readl(host->base + MMC_I_MASK);
	stat = readl(host->base + MMC_STAT);
	
	dev_dbg(host->dev, "irq %08x stat %08x\n", ireg, stat);
	
	if (ireg & END_CMD_RES) {
		host->irq_flag |= MCI_FLAG_CMD_DONE;
		host->cmd_stat = stat;
		wake = 1;
		host->imask |= END_CMD_RES;
		writel(host->imask, host->base + MMC_I_MASK);
	}
	if (ireg & DATA_TRAN_DONE) {
		host->irq_flag |= MCI_FLAG_DATA_DONE;
		host->data_stat = stat;
		wake = 1;
		host->imask |= DATA_TRAN_DONE;
		writel(host->imask, host->base + MMC_I_MASK);
	}
	if (ireg & CLK_IS_OFF) {
		host->irq_flag |= MCI_FLAG_CLK_OFF;
		wake = 1;
		host->imask |= CLK_IS_OFF;
		writel(host->imask, host->base + MMC_I_MASK);
	}
	if (ireg & PRG_DONE) {
		host->irq_flag |= MCI_FLAG_PRG_DONE;
		wake = 1;
		host->imask |= PRG_DONE;
		writel(host->imask, host->base + MMC_I_MASK);
	}
	
	if (wake) {
		wake_up_interruptible(&host->wq);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

/*
 * Wait till card becomes not busy (PRG_DONE)
 * Returns 0 if OK, -ETIMEDOUT in case of a timeout, -ERESTARTSYS if interrupted
 */
static int pxamci_multi_handle_busy(struct pxamci_multi_host *host)
{
	int err;
	int ret = 0;
	
	host->irq_flag = 0;
	pxamci_multi_enable_irq(host, PRG_DONE);
	err = wait_event_interruptible_timeout(host->wq,\
			host->irq_flag & MCI_FLAG_PRG_DONE, BUSY_TIMEOUT);
	pxamci_multi_disable_irq(host, PRG_DONE);
	
	if (!err)
		ret = -ETIMEDOUT;
	if (err < 0)
		ret = -ERESTARTSYS;
	
	if (ret)
		dev_err(host->dev, "Busy of slot %d not handled: %d\n",\
			host->slot_selected, ret);
	return ret;
}

/*
 * Stop MMCLK and wait till it stops (CLK_IS_OFF)
 * Returns 0 if OK, -ETIMEDOUT in case of a timeout, -ERESTARTSYS if interrupted
 */
static int pxamci_multi_stop_clock(struct pxamci_multi_host *host)
{
	int err;
	int ret = 0;
	host->irq_flag = 0;
	if (!(readl(host->base + MMC_STRPCL) & STAT_CLK_EN))
		goto out;
	
	pxamci_multi_enable_irq(host, CLK_IS_OFF);
	writel(STOP_CLOCK, host->base + MMC_STRPCL);
	err = wait_event_interruptible_timeout(host->wq,\
			host->irq_flag & MCI_FLAG_CLK_OFF, CLKOFF_TIMEOUT);
	pxamci_multi_disable_irq(host, CLK_IS_OFF);
	

	if (!err)
		ret = -ETIMEDOUT;
	if (err < 0)
		ret = -ERESTARTSYS;
	
out:
	if (ret)
		dev_err(host->dev, "MMCLK for slot %d not stopped: %d\n",\
			host->slot_selected, ret);
	return ret;
}

/*
 * Setup a command, issue it, wait for completion and detect errors
 * Returns 0 if OK and errnum (as passed in mmc_command) in case of an error
 */
static int pxamci_multi_issue_cmd(struct pxamci_multi_host *host,\
				struct mmc_command *cmd, struct mmc_data *data)
{
	int ret = 0;
	int err;
	
	unsigned int i;
	unsigned int v;
	
	unsigned int cmdat = host->slots[host->slot_selected]->cmdat;
	
	switch (mmc_resp_type(cmd)) {
		case MMC_RSP_R1B:
			cmdat |= CMDAT_BUSY;
		case MMC_RSP_R1:
			cmdat |= CMDAT_RESP_SHORT;
			break;
		case MMC_RSP_R3:
			cmdat |= CMDAT_RESP_R3;
			break;
		case MMC_RSP_R2:
			cmdat |= CMDAT_RESP_R2;
			break;
	}
	
	/* If there is data following CMD */
	if (data) {
		cmdat &= ~CMDAT_BUSY;
		cmdat |= CMDAT_DATAEN | CMDAT_DMAEN;
		if(data->flags & MMC_DATA_WRITE)
			cmdat |= CMDAT_WRITE;
	}
	
	err = pxamci_multi_stop_clock(host);
	if (err) {
		ret = err;
		goto out;
	}
	
	writel(host->slots[host->slot_selected]->clkrt, host->base + MMC_CLKRT);
	writel(cmd->opcode, host->base + MMC_CMD);
	writel(cmd->arg >> 16, host->base + MMC_ARGH);
	writel(cmd->arg & 0xffff, host->base + MMC_ARGL);
	writel(cmdat, host->base + MMC_CMDAT);
	
	host->irq_flag = 0;
	pxamci_multi_enable_irq(host, END_CMD_RES);
	
	/* Issue CMD */
	writel(START_CLOCK, host->base + MMC_STRPCL);
	
	/* Wait for interrupt */
	err = wait_event_interruptible(host->wq,\
				host->irq_flag & MCI_FLAG_CMD_DONE);
	pxamci_multi_disable_irq(host, END_CMD_RES);
	
	if (err) {
		ret = -ERESTARTSYS;
		goto out;
	}
	/* Read response */
	v = readl(host->base + MMC_RES) & 0xffff;
	for (i = 0; i < 4; i++) {
		unsigned int w1 = readl(host->base + MMC_RES) & 0xffff;
		unsigned int  w2 = readl(host->base + MMC_RES) & 0xffff;
		cmd->resp[i] = v << 24 | w1 << 8 | w2 >> 8;
		v = w2;
	}
	/* Handle CMD errors */
	if (host->cmd_stat & STAT_TIME_OUT_RESPONSE) {
		ret = -ETIMEDOUT;
		goto out;
	}
	if (host->cmd_stat & (STAT_FLASH_ERR | STAT_RES_CRC_ERR)) {
		ret = -EILSEQ;
		goto out;
	}
	/* If busy signaling after CMD is possible, wait for PRG_DONE */
	if (cmdat & CMDAT_BUSY) {
		err = pxamci_multi_handle_busy(host);
		if(err)
			dev_err(host->dev, "busy signaling timed out\n");
	}
out:
	cmd->error = ret;
	return ret;
}

/*
 * Wait till data transfer completes and deal with errors
 * DMA is mapped here, so unmap it
 * Returns 0 if OK and errnum (as passed to mmc_data) in case of an error
 */
static int pxamci_multi_handle_data(struct pxamci_multi_host *host,\
				struct mmc_data *data)
{
	int err;
	
	unsigned int rem_blks;
	
	pxamci_multi_enable_irq(host, DATA_TRAN_DONE);
	err = wait_event_interruptible(host->wq,\
				host->irq_flag & MCI_FLAG_DATA_DONE);
	pxamci_multi_disable_irq(host, DATA_TRAN_DONE);
	
	if (err) {
		data->error = -ERESTARTSYS;
		goto out;
	}
	
	/* Handle data transfer errors */
	if (host->data_stat & (STAT_FLASH_ERR | STAT_CRC_READ_ERROR |\
				STAT_CRC_WRITE_ERROR))
		data->error = -EILSEQ;
	if (host->data_stat & STAT_READ_TIME_OUT)
		data->error = -ETIMEDOUT;
	
	rem_blks = readl(host->base + MMC_BLKS_REM);
	data->bytes_xfered = data->blksz * (data->blocks - rem_blks);
	
	/* Handle busy if needed */
	if (data->flags & MMC_DATA_WRITE &&\
				!(host->data_stat & STAT_READ_TIME_OUT)) {
		err = pxamci_multi_handle_busy(host);
		if(err)dev_err(host->dev, "busy signaling timed out\n");
	}
	
out:
	DCSR(host->dma) = 0;
	dma_unmap_sg(host->dev, data->sg, data->sg_len,
				host->dma_dir);
	return data->error;
}

/*
 * Request from mmc stack core handling
 */
static void pxamci_multi_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct pxamci_multi_slot *slot = mmc_priv(mmc);
	struct pxamci_multi_host *host = slot->host;
	
	int err;
	
	/* 
	 * Dedicate controller to one mmc_host
	 * wait until lock is released (other request is processed)
	 * and get the lock 
	 */
	err = down_interruptible(&host->slot_lock);
	
	if (err) {
		mrq->cmd->error = -ERESTARTSYS;
		goto out_sema_up;
	}
	/* Select appropriate slot here */
	if (slot->id != host->slot_selected &&\
			host->pdata->select(host->dev, slot->id)) {
		dev_err(host->dev, "slot %d selection error, dropping lock\n",\
				slot->id);
		mrq->cmd->error = -ENXIO;
		goto out_sema;
	}
	
	host->slot_selected = slot->id;
	
	/* Setup DMA */
	if (mrq->data)
		pxamci_multi_setup_data(host, mrq->data);
	/* Issue CMD */
	err = pxamci_multi_issue_cmd(host, mrq->cmd, mrq->data);
	if (err)
		goto out_data;
	
	/* Deal with data: wait for data transfer completion */
	if (mrq->data) {
		err = pxamci_multi_handle_data(host, mrq->data);
		if(err)
			goto out_data;
		
		/* Finally, issue stop cmd */
		if (mrq->stop) {
			err = pxamci_multi_issue_cmd(host, mrq->stop, NULL);
			if(err)goto out_data;
		}
	}
	/* 
	 * If request succeeded, don't init card anymore
	 */
	host->slots[host->slot_selected]->cmdat &= ~(CMDAT_INIT);
	
out_data:
out_sema:
	up(&host->slot_lock);
out_sema_up:
	mmc_request_done(mmc, mrq);
}

static int pxamci_multi_get_ro(struct mmc_host *mmc)
{
	struct pxamci_multi_slot *slot = mmc_priv(mmc);
	struct pxamci_multi_host *host = slot->host;

	if (gpio_is_valid(host->pdata->slot_info[slot->id].gpio_card_ro)) {
		if (host->pdata->slot_info[slot->id].gpio_card_ro_invert)
			return !gpio_get_value(\
				host->pdata->slot_info[slot->id].gpio_card_ro);
		else
			return gpio_get_value(\
				host->pdata->slot_info[slot->id].gpio_card_ro);
	}
	if (host->pdata->slot_info[slot->id].get_ro)
		return !!host->pdata->slot_info[slot->id].get_ro(host->dev,\
				slot->id);
	/*
	 * Slot doesn't support Read-Only detection
	 */
	return -ENOSYS;
}

static void pxamci_multi_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct pxamci_multi_slot *slot = mmc_priv(mmc);
	struct pxamci_multi_host *host = slot->host;
	unsigned int i;
	int ret;
	
	ret = down_interruptible(&host->slot_lock);
	if (ret) {
		dev_err(host->dev, "set_ios: down_interruptible interrupted\n");
		return;
	}
	
	if (ios->clock) {
		if (!host->clock_enabled) {
			clk_enable(host->clk);
			host->clock_enabled = 1;
			dev_dbg(host->dev, "enabling clock\n");
		}
		
		if (ios->clock == 26000000) {
			/* to support 26MHz */
			slot->clkrt = 7;
			slot->real_clock = 26000000;
		} else {
			unsigned int shift =\
				fls(1 + (host->clkrate - 1) / ios->clock) - 1;
			/*
			 * clk might result in a lower divisor than we
			 * desire.  check for that condition and adjust
			 * as appropriate.
			 */
			if ((host->clkrate >> shift) > ios->clock)
				++shift;
			if (shift > 6)
				shift = 6;
			slot->clkrt = shift;
			slot->real_clock = (host->clkrate >> shift);
		}
		
	} else {
		pxamci_multi_stop_clock(host);
		
		if (slot->clkrt != CLKRT_OFF) {
			int disable_clock = 1;
			
			slot->clkrt = CLKRT_OFF;
			slot->real_clock = 0;
			
			for(i = 0; i < host->slot_count; ++i)
				if(host->slots[i]->clkrt != CLKRT_OFF){
					disable_clock = 0;
					break;
				}
			if (disable_clock && host->clock_enabled) {
				clk_disable(host->clk);
				host->clock_enabled = 0;
				dev_dbg(host->dev, "disabling clock\n");
			}
		}
	}
	slot->desired_clock = ios->clock;
	
	slot->power_mode = ios->power_mode;
	if(ios->power_mode == MMC_POWER_UP){
		slot->cmdat |= CMDAT_INIT;
		ret = pxamci_multi_set_power(host, 1);
		if (ret)
			dev_err(host->dev, "unable to power up\n");
	} else if (ios->power_mode == MMC_POWER_OFF) {
		int power_off=1;
		for(i = 0; i < host->slot_count; ++i)
			if (host->slots[i]->power_mode != MMC_POWER_OFF) {
				power_off = 0;
				break;
			}
		
		if (power_off) {
			ret = pxamci_multi_set_power(host, 0);
			if (ret)
				dev_err(host->dev, "unable to power down\n");
		}
	}
	
	if (ios->bus_width == MMC_BUS_WIDTH_4)
		slot->cmdat |= CMDAT_SD_4DAT;
	else
		slot->cmdat &= ~CMDAT_SD_4DAT;
	
	up(&host->slot_lock);
}

static const struct mmc_host_ops pxamci_multi_ops = {
	.request		= pxamci_multi_request,
	.get_ro			= pxamci_multi_get_ro,
	.set_ios		= pxamci_multi_set_ios,
};

static int pxamci_multi_probe(struct platform_device *pdev)
{
	unsigned int i, top;
	struct pxamci_multi_host *host = NULL;
	struct resource *r, *dmarx, *dmatx;
	int ret, irq, gpio_cd, gpio_ro, gpio_power;

	struct mmc_host *mmc;
	struct pxamci_multi_slot *slot;
	
	if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "must supply platform_data\n");
		return -EINVAL;
	}
	
	/* Get resources */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!r || irq < 0)
		return -ENXIO;

	r = request_mem_region(r->start, SZ_4K, DRIVER_NAME);
	if (!r)
		return -EBUSY;

	/* Allocate pxamci_multi_host struct and fill it */
	host = kzalloc(sizeof(struct pxamci_multi_host), GFP_KERNEL);
	if (!host) {
		dev_err(&pdev->dev, "can't allocate memory for host\n");
		return -ENOMEM;
	}
	
	host->dev = &pdev->dev;
	host->pdata = pdev->dev.platform_data;
	host->slot_count = host->pdata->slot_count;
	if (!host->slot_count) {
		dev_err(&pdev->dev, "no slots\n");
		ret = -EINVAL;
		goto out_get_clk;
	}
	
	host->dma = -1;
	host->clk = clk_get(host->dev, NULL);
	
	if (IS_ERR(host->clk)) {
		dev_err(&pdev->dev, "can't get MCI clock reference\n");
		ret = -EBUSY;
		goto out_get_clk;
	}
	
	host->clkrate = clk_get_rate(host->clk);
	host->res = r;
	host->irq = irq;
	host->imask = MMC_I_MASK_ALL;
	host->clock_enabled = 0;
	
	/* Allocate page for DMA */
	host->sg_cpu = dma_alloc_coherent(&pdev->dev, PAGE_SIZE,\
					&host->sg_dma, GFP_KERNEL);
	if (!host->sg_cpu) {
		dev_err(&pdev->dev, "can't allocate DMA page\n");
		ret = -ENOMEM;
		goto out_alloc_dma_page;
	}
	
	/* Request resources */
	host->dma = pxa_request_dma(DRIVER_NAME, DMA_PRIO_LOW,\
					pxamci_multi_dma_irq, host);
	if (host->dma < 0) {
		dev_err(&pdev->dev, "can't request DMA\n");
		ret = -EBUSY;
		goto out_req_dma;
	}
	
	ret = request_irq(host->irq, pxamci_multi_irq, 0, DRIVER_NAME, host);
	if (ret) {
		dev_err(&pdev->dev, "can't request MCI IRQ\n");
		ret = -EBUSY;
		goto out_req_irq;
	}
	
	platform_set_drvdata(pdev, host);
	
	dmarx = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!dmarx) {
		dev_err(&pdev->dev, "can't get DMA Rx resource\n");
		ret = -ENXIO;
		goto out_req_dmarx;
	}
	host->dma_drcmrrx = dmarx->start;
	
	dmatx = platform_get_resource(pdev, IORESOURCE_DMA, 1);
	if (!dmatx) {
		dev_err(&pdev->dev, "can't get DMA Tx resource\n");
		ret = -ENXIO;
		goto out_req_dmatx;
	}
	host->dma_drcmrtx = dmatx->start;
	
	/* MMIO base */
	host->base = ioremap(r->start, SZ_4K);
	if (!host->base) {
		dev_err(&pdev->dev, "can't remap MMIO\n");
		ret = -ENOMEM;
		goto out_remap_mmio;
	}
	
	/* Locks and WQ */
	spin_lock_init(&host->lock);
	sema_init(&host->slot_lock, 1);
	init_waitqueue_head(&host->wq);
	/*
	 * Ensure that the host controller clock is stopped, and setup
	 * with our defaults.
	 */
	pxamci_multi_stop_clock(host);
	writel(0, host->base + MMC_SPI);
	writel(64, host->base + MMC_RESTO);
	writel(host->imask, host->base + MMC_I_MASK);

	/* mmc_hosts and slots */
	
	host->slots = kzalloc(sizeof(struct pxamci_multi_slot*) *\
				host->slot_count, GFP_KERNEL);
	if (!host->slots) {
		dev_err(&pdev->dev, "can't allocate memory for slots\n");
		ret = -ENOMEM;
		goto out_alloc_slots;
	}
	host->hosts = kzalloc(sizeof(struct mmc_host*) * host->slot_count,\
				GFP_KERNEL);
	if (!host->hosts) {
		dev_err(&pdev->dev, "can't allocate memory for hosts\n");
		ret = -ENOMEM;
		goto out_alloc_hosts;
	}
	
	for(i = 0; i < host->slot_count; ++i)
	{
		mmc = mmc_alloc_host(sizeof(struct pxamci_multi_slot),\
					&pdev->dev);
		if (!mmc) {
			dev_err(&pdev->dev, "can't allocate mmc_host\n");
			ret = -ENOMEM;
			goto out_process_slots;
		}
		host->hosts[i] = mmc;
		
		mmc->ops = &pxamci_multi_ops;
		mmc->max_segs = MAX_DMA_SEGS;
		mmc->max_seg_size = MAX_DMA_SEG_SIZE;
		mmc->max_blk_size = MAX_MMC_BLK_SIZE;
		mmc->max_blk_count = MAX_MMC_BLKS;
		mmc->max_req_size = min(mmc->max_blk_size * mmc->max_blk_count,
					mmc->max_segs * mmc->max_seg_size);
		mmc->ocr_avail = host->pdata->ocr_mask;
		
		mmc->f_min = (host->clkrate + 63) / 64;
		mmc->f_max = 26000000;
		
		slot=mmc_priv(mmc);
		host->slots[i] = slot;
		
		slot->host = host;
		slot->id = i;
		slot->clkrt = CLKRT_OFF;
		slot->cmdat = 0;
		
		mmc->caps = MMC_CAP_WAIT_WHILE_BUSY | MMC_CAP_4_BIT_DATA |\
			    MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED;
		
		if (!host->pdata->slot_info[i].removable)
			mmc->caps |= MMC_CAP_NONREMOVABLE;
		
		gpio_cd = host->pdata->slot_info[i].gpio_card_detect;
		gpio_ro = host->pdata->slot_info[i].gpio_card_ro;
		
		if (gpio_is_valid(gpio_ro)) {
			ret = gpio_request(gpio_ro, "mmc card read only");
			if (ret) {
				dev_err(host->dev,\
					"failed gpio_ro %d request\n", gpio_ro);
				goto out_process_slots;
			}
			gpio_direction_input(gpio_ro);
		}
		if (gpio_is_valid(gpio_cd)) {
			ret = gpio_request(gpio_cd, "mmc card detect");
			if (ret) {
				dev_err(host->dev,\
					"failed gpio_cd %d request\n", gpio_cd);
				goto out_process_slots;
			}
			gpio_direction_input(gpio_cd);
			
			ret = request_irq(gpio_to_irq(gpio_cd),\
					  pxamci_multi_detect_irq,
					  IRQF_TRIGGER_RISING |\
					  IRQF_TRIGGER_FALLING,
					  "mmc card detect", mmc);
			if (ret) {
				dev_err(host->dev,\
					"failed to request card detect IRQ\n");
				goto out_process_slots;
			}
		}
	}

	/* MMC bus power gpio */
	gpio_power = host->pdata->gpio_power;
	
	if (gpio_is_valid(gpio_power)) {
		if ((ret = gpio_request(gpio_power, "mmc bus power")))
		{
			dev_err(&pdev->dev,\
				"failed gpio_power %d request\n", gpio_power);
			goto out_gpio_power;
		}
		gpio_direction_output(gpio_power,\
				      host->pdata->gpio_power_invert);
	}
	
	/* Call mach-specific init proc */
	if (host->pdata->init)
		host->pdata->init(&pdev->dev, pxamci_multi_detect_irq,\
				  host->hosts);
	
	/* Select first slot */
	if(host->pdata->select)
	{
		if((ret = host->pdata->select(host->dev, 0)))
		{
			dev_err(host->dev, "slot 0 selection error\n");
			goto out_select_slot;
		}
	}
	else
	{
		dev_err(host->dev, "slot select callback not defined\n");
		ret = -EINVAL;
		goto out_select_slot;
	}
	host->slot_selected = 0;

	/* Add hosts to mmc core */
	for(i = 0; i < host->slot_count; ++i)
		if((ret = mmc_add_host(host->hosts[i])))
		{
			dev_err(host->dev, "unable to add mmc_host\n");
			goto out_add_hosts;
		}
	
	printk(KERN_INFO "pxamci-multi attached to %s (slots: %d)\n",\
		dev_name(&pdev->dev), host->slot_count);
	
	return 0;
	
/* Freeing resources in case of error */
out_add_hosts:
	top = i;
	for(i = 0; i < top; ++i)
		mmc_free_host(host->hosts[i]);
	
out_select_slot:
	if (host->pdata && host->pdata->exit)
		host->pdata->exit(&pdev->dev, host->hosts);
	if(gpio_is_valid(gpio_power))
		gpio_free(gpio_power);
	
out_gpio_power:
out_process_slots:
	for(i = 0; i < host->slot_count; ++i) {	
		if (gpio_is_valid(host->pdata->slot_info[i].gpio_card_detect)) {
			free_irq(gpio_to_irq(\
				 host->pdata->slot_info[i].gpio_card_detect),\
				 host->hosts[i]);
			gpio_free(host->pdata->slot_info[i].gpio_card_detect);
		}
		if (gpio_is_valid(host->pdata->slot_info[i].gpio_card_ro))
			gpio_free(host->pdata->slot_info[i].gpio_card_ro);
	}
	kfree(host->hosts);
	
out_alloc_hosts:
	kfree(host->slots);
	
out_alloc_slots:
	iounmap(host->base);
	
out_remap_mmio:
out_req_dmatx:
out_req_dmarx:
	free_irq(host->irq, host);
	
out_req_irq:
	pxa_free_dma(host->dma);
	
out_req_dma:
	dma_free_coherent(&pdev->dev, PAGE_SIZE, host->sg_cpu, host->sg_dma);
	
out_alloc_dma_page:
	clk_put(host->clk);
	
out_get_clk:
	release_resource(r);
	kfree(host);
	
	return ret;
}

static int pxamci_multi_remove(struct platform_device *pdev)
{
	struct pxamci_multi_host *host = platform_get_drvdata(pdev);
	unsigned int i;
	
	platform_set_drvdata(pdev, NULL);

	if (!host)
		return 0;
	
	for(i = 0; i < host->slot_count; ++i) {
		mmc_free_host(host->hosts[i]);
		if (gpio_is_valid(host->pdata->slot_info[i].gpio_card_detect)) {
			gpio_free(host->pdata->slot_info[i].gpio_card_detect);
			free_irq(gpio_to_irq(\
				 host->pdata->slot_info[i].gpio_card_detect),\
				 host->hosts[i]);
		}
		if (gpio_is_valid(host->pdata->slot_info[i].gpio_card_ro))
			gpio_free(host->pdata->slot_info[i].gpio_card_ro);
	}
	if (gpio_is_valid(host->pdata->gpio_power))
		gpio_free(host->pdata->gpio_power);
	
	if (host->pdata->exit)
		host->pdata->exit(&pdev->dev, host->hosts);
	pxamci_multi_stop_clock(host);
	writel(TXFIFO_WR_REQ|RXFIFO_RD_REQ|CLK_IS_OFF|STOP_CMD|
	       END_CMD_RES|PRG_DONE|DATA_TRAN_DONE,
	       host->base + MMC_I_MASK);
	
	DRCMR(host->dma_drcmrrx) = 0;
	DRCMR(host->dma_drcmrtx) = 0;
	free_irq(host->irq, host);
	pxa_free_dma(host->dma);
	iounmap(host->base);
	dma_free_coherent(&pdev->dev, PAGE_SIZE, host->sg_cpu, host->sg_dma);
	if(host->clock_enabled)
		clk_disable(host->clk);
	clk_put(host->clk);
	release_resource(host->res);
	kfree(host);

	return 0;
}

#ifdef CONFIG_PM
static int pxamci_multi_suspend(struct device *dev)
{
	struct pxamci_multi_host *host = dev_get_drvdata(dev);
	int ret = 0;
	unsigned int i;
	
	for(i = 0; i < host->slot_count; ++i)
		if(mmc_suspend_host(host->hosts[i])){
			dev_err(host->dev, "host %d suspending error\n", i);
			ret = -ENXIO;
		}

	return ret;
}

static int pxamci_multi_resume(struct device *dev)
{
	struct pxamci_multi_host *host = dev_get_drvdata(dev);
	int ret = 0;
	unsigned int i;
	/* Select slot 0, then resume mmc hosts */
	if((ret = host->pdata->select(host->dev, 0))){
		dev_err(host->dev, "slot 0 selection error on resume\n");
		return ret;
	}
	host->slot_selected = 0;
	
	for(i = host->slot_count; i > 0 ; --i)
		if(mmc_resume_host(host->hosts[i-1])){
			dev_err(host->dev, "host %d resume error\n", i-1);
			ret = -ENXIO;
		}
	
	return ret;
}

static const struct dev_pm_ops pxamci_multi_pm_ops = {
	.suspend	= pxamci_multi_suspend,
	.resume		= pxamci_multi_resume,
};
#endif

static struct platform_driver pxamci_multi_driver = {
	.probe		= pxamci_multi_probe,
	.remove		= pxamci_multi_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pxamci_multi_pm_ops,
#endif
	},
};

static int __init pxamci_multi_init(void)
{
	return platform_driver_register(&pxamci_multi_driver);
}

static void __exit pxamci_multi_exit(void)
{
	platform_driver_unregister(&pxamci_multi_driver);
}

module_init(pxamci_multi_init);
module_exit(pxamci_multi_exit);

MODULE_DESCRIPTION("PXA MCI driver, multiple cards support");
MODULE_AUTHOR("Roman Dobrodiy <ztcoils@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pxamci-multi");
