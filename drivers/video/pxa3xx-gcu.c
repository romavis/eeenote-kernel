/*
 *  pxa3xx-gcu.c - Linux kernel module for PXA3xx graphics controllers
 *
 *  This driver needs a DirectFB counterpart in user space, communication
 *  is handled via mmap()ed memory areas and an ioctl.
 *
 *  Copyright (c) 2009 Daniel Mack <daniel@caiaq.de>
 *  Copyright (c) 2009 Janine Kropp <nin@directfb.org>
 *  Copyright (c) 2009 Denis Oliver Kropp <dok@directfb.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * WARNING: This controller is attached to System Bus 2 of the PXA which
 * needs its arbiter to be enabled explicitly (CKENB & 1<<9).
 * There is currently no way to do this from Linux, so you need to teach
 * your bootloader for now.
 */

/*
 * 31 Aug 2012 (C) Roman Dobrodiy
 * Some code rewritten to make things simpler
 * Instruction batches are organized in ring buffer-like structure
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/list.h>

#include "pxa3xx-gcu.h"

#define DRV_NAME	"pxa3xx-gcu"
#define MISCDEV_MINOR	197

#define REG_GCCR	0x00
#define GCCR_SYNC_CLR	(1 << 9)
#define GCCR_BP_RST	(1 << 8)
#define GCCR_ABORT	(1 << 6)
#define GCCR_STOP	(1 << 4)

#define REG_GCISCR	0x04
#define REG_GCIECR	0x08
#define REG_GCRBBR	0x20
#define REG_GCRBLR	0x24
#define REG_GCRBHR	0x28
#define REG_GCRBTR	0x2C
#define REG_GCRBEXHR	0x30

#define IE_EOB		(1 << 0)
#define IE_EEOB		(1 << 5)
#define IE_BF		(1 << 2)
#define IE_ALL		0xff

#define SHARED_SIZE	PAGE_ALIGN(sizeof(struct pxa3xx_gcu_shared))

/* #define PXA3XX_GCU_DEBUG */
/* #define PXA3XX_GCU_DEBUG_TIMER */

#ifdef PXA3XX_GCU_DEBUG
#define QDUMP(msg)					\
	do {						\
		QPRINT(priv, KERN_DEBUG, msg);		\
	} while (0)
#else
#define QDUMP(msg)	do {} while (0)
#endif

#define QERROR(msg)					\
	do {						\
		QPRINT(priv, KERN_ERR, msg);		\
	} while (0)

struct pxa3xx_gcu_batch {
	struct pxa3xx_gcu_batch *next;
	u32			*ptr;
	dma_addr_t		 phys;
	unsigned long		 length;
	int			 filled;
};

struct pxa3xx_gcu_buffer_priv {
	unsigned int		id;	/* Unique buffer ID */
	unsigned int		ref_vma;
	unsigned int		ref_fd;

	void			*ptr;	/* Kernel space vmem ptr */
	unsigned long		phys;	/* Address in physical memory */
	unsigned int		size;	/* Length of a buffer */

	struct list_head	list;
};

struct pxa3xx_gcu_priv {
	void __iomem		 *mmio_base;
	struct clk		 *clk;
	struct pxa3xx_gcu_shared *shared;
	dma_addr_t		  shared_phys;
	struct resource		 *resource_mem;
	struct miscdevice	  misc_dev;
	struct file_operations	  misc_fops;
	wait_queue_head_t	  wait_idle;
	wait_queue_head_t	  wait_free;
	spinlock_t		  spinlock;
	struct timeval 		  base_time;

	struct pxa3xx_gcu_batch *free;		/* Currently free batch */

	struct pxa3xx_gcu_batch *exec_first;	/* First batch in exec chain */
	struct pxa3xx_gcu_batch *exec_last;	/* Last batch in exec chain */
	struct pxa3xx_gcu_batch *filled_last;	/* Last filled batch
						 * yet not in exec chain */

	struct pxa3xx_gcu_batch *first;		/* Ring buffer head */
	struct pxa3xx_gcu_batch *last;		/* Ring buffer tail */

	/* Scratchpad buffer list */
	struct list_head 	buffers;
};

static inline unsigned long
gc_readl(struct pxa3xx_gcu_priv *priv, unsigned int off)
{
	return __raw_readl(priv->mmio_base + off);
}

static inline void
gc_writel(struct pxa3xx_gcu_priv *priv, unsigned int off, unsigned long val)
{
	__raw_writel(val, priv->mmio_base + off);
}

#define QPRINT(priv, level, msg)					\
	do {								\
		struct timeval tv;					\
		struct pxa3xx_gcu_shared *shared = priv->shared;	\
		u32 base = gc_readl(priv, REG_GCRBBR);			\
									\
		do_gettimeofday(&tv);					\
									\
		printk(level "%ld.%03ld.%03ld - %-17s: %-21s (%s, "	\
			"STATUS "					\
			"0x%02lx, B 0x%08lx [%ld], E %5ld, H %5ld, "	\
			"T %5ld)\n",					\
			tv.tv_sec - priv->base_time.tv_sec,		\
			tv.tv_usec / 1000, tv.tv_usec % 1000,		\
			__func__, msg,					\
			shared->hw_running ? "running" : "   idle",	\
			gc_readl(priv, REG_GCISCR),			\
			gc_readl(priv, REG_GCRBBR),			\
			gc_readl(priv, REG_GCRBLR),			\
			(gc_readl(priv, REG_GCRBEXHR) - base) / 4,	\
			(gc_readl(priv, REG_GCRBHR) - base) / 4,	\
			(gc_readl(priv, REG_GCRBTR) - base) / 4);	\
	} while (0)

static void
pxa3xx_gcu_reset(struct pxa3xx_gcu_priv *priv)
{
	struct pxa3xx_gcu_batch *batch;
	QDUMP("RESET");

	/* disable interrupts */
	gc_writel(priv, REG_GCIECR, 0);

	/* reset hardware */
	gc_writel(priv, REG_GCCR, GCCR_ABORT);
	gc_writel(priv, REG_GCCR, 0);

	memset(priv->shared, 0, SHARED_SIZE);
	priv->shared->buffer_phys = priv->shared_phys;
	priv->shared->magic = PXA3XX_GCU_SHARED_MAGIC;


	do_gettimeofday(&priv->base_time);
	/* set up the ring buffer pointers */
	gc_writel(priv, REG_GCRBLR, 0);
	gc_writel(priv, REG_GCRBBR, priv->shared_phys);
	gc_writel(priv, REG_GCRBTR, priv->shared_phys);

	/* reset all batches */
	batch = priv->first;
	do {
		batch->filled = 0;
		batch = batch->next;
	} while (batch != priv->first);
	priv->free = priv->first;

	/* enable all IRQs except EOB & BF */
	gc_writel(priv, REG_GCIECR, IE_ALL & ~(IE_EOB | IE_BF));
}

static void
dump_whole_state(struct pxa3xx_gcu_priv *priv)
{
	struct pxa3xx_gcu_shared *sh = priv->shared;
	u32 base = gc_readl(priv, REG_GCRBBR);

	QDUMP("DUMP");

	printk(KERN_DEBUG "== PXA3XX-GCU DUMP ==\n"
		"%s, STATUS 0x%02lx, B 0x%08lx [%ld], E %5ld, H %5ld, T %5ld\n",
		sh->hw_running ? "running" : "idle   ",
		gc_readl(priv, REG_GCISCR),
		gc_readl(priv, REG_GCRBBR),
		gc_readl(priv, REG_GCRBLR),
		(gc_readl(priv, REG_GCRBEXHR) - base) / 4,
		(gc_readl(priv, REG_GCRBHR) - base) / 4,
		(gc_readl(priv, REG_GCRBTR) - base) / 4);
}

static void
run_ready(struct pxa3xx_gcu_priv *priv)
{
	unsigned int num = 0;
	struct pxa3xx_gcu_shared *shared = priv->shared;
	struct pxa3xx_gcu_batch	*ready = priv->exec_first;

	QDUMP("Start");

	BUG_ON(!ready);

	shared->buffer[num++] = 0x05000000;

	while(1) {
		shared->buffer[num++] = 0x00000001;
		shared->buffer[num++] = ready->phys;
		if(!ready->next->filled || ready->next == priv->exec_first)
			break;
		ready = ready->next;
	}

	priv->exec_last = ready;
	shared->buffer[num++] = 0x05000000;

	gc_writel(priv, REG_GCRBLR, 0);
	shared->hw_running = 1;

	/* ring base address */
	gc_writel(priv, REG_GCRBBR, shared->buffer_phys);

	/* ring tail address */
	gc_writel(priv, REG_GCRBTR, shared->buffer_phys + num * 4);

	/* ring length */
	gc_writel(priv, REG_GCRBLR, ((num + 63) & ~63) * 4);
}

static irqreturn_t
pxa3xx_gcu_handle_irq(int irq, void *ctx)
{
	struct pxa3xx_gcu_priv *priv = ctx;
	struct pxa3xx_gcu_shared *shared = priv->shared;
	struct pxa3xx_gcu_batch *buffer;
	u32 status = gc_readl(priv, REG_GCISCR) & IE_ALL;

	QDUMP("-Interrupt");

	if (!status)
		return IRQ_NONE;

	spin_lock(&priv->spinlock);
	shared->num_interrupts++;

	if (status & IE_EEOB) {
		QDUMP(" [EEOB]");

		buffer = priv->exec_first;
		do {
			buffer->filled = 0;
			buffer = buffer->next;
		} while (buffer != priv->exec_last);
		buffer->filled = 0;

		priv->free = priv->exec_first;

		wake_up_all(&priv->wait_free);

		if (buffer->next->filled) {
			priv->exec_first = buffer->next;
			run_ready(priv);
		} else {
			/* There is no more data prepared by the userspace.
			 * Set hw_running = 0 and wait for the next userspace
			 * kick-off */
			shared->num_idle++;
			shared->hw_running = 0;

			QDUMP(" '-> Idle.");

			/* set ring buffer length to zero */
			gc_writel(priv, REG_GCRBLR, 0);

			wake_up_all(&priv->wait_idle);
		}

		shared->num_done++;
	} else {
		QERROR(" [???]");
		dump_whole_state(priv);
	}

	/* Clear the interrupt */
	gc_writel(priv, REG_GCISCR, status);
	spin_unlock(&priv->spinlock);

	return IRQ_HANDLED;
}

static int
pxa3xx_gcu_wait_idle(struct pxa3xx_gcu_priv *priv)
{
	int ret = 0;

	QDUMP("Waiting for idle...");

	/* Does not need to be atomic. There's a lock in user space,
	 * but anyhow, this is just for statistics. */
	priv->shared->num_wait_idle++;

	while (priv->shared->hw_running) {
		int num = priv->shared->num_interrupts;
		u32 rbexhr = gc_readl(priv, REG_GCRBEXHR);

		ret = wait_event_interruptible_timeout(priv->wait_idle,
					!priv->shared->hw_running, HZ*4);

		if (ret < 0)
			break;

		if (ret > 0)
			continue;

		if (gc_readl(priv, REG_GCRBEXHR) == rbexhr &&
		priv->shared->num_interrupts == num) {
			QERROR("TIMEOUT");
			ret = -ETIMEDOUT;
			break;
		}
	}

	QDUMP("done");

	return ret;
}

static int
pxa3xx_gcu_wait_free(struct pxa3xx_gcu_priv *priv)
{
	int ret = 0;

	QDUMP("Waiting for free...");

	/* Does not need to be atomic. There's a lock in user space,
	 * but anyhow, this is just for statistics. */
	priv->shared->num_wait_free++;

	while (!priv->free) {
		u32 rbexhr = gc_readl(priv, REG_GCRBEXHR);

		ret = wait_event_interruptible_timeout(priv->wait_free,
						priv->free, HZ*4);

		if (ret < 0)
			break;

		if (ret > 0)
			continue;

		if (gc_readl(priv, REG_GCRBEXHR) == rbexhr) {
			QERROR("TIMEOUT");
			ret = -ETIMEDOUT;
			break;
		}
	}

	QDUMP("done");

	return ret;
}

/* Scratchpad buffer ops */

/*
 * Buffer "garbage collection"
 * Try to free memory and remove buffer
 * Buffer is freed if and only if ref_fd == 0 && ref_vma == 0
 * i.e. it is not referenced by any struct file* and mmap-ed
 */
static void pxa3xx_gcu_buffer_cleanup(struct pxa3xx_gcu_buffer_priv *buffer)
{
	printk(KERN_INFO "pxa3xx-gcu: cleanup %d fdref %d vmaref %d",\
		buffer->id, buffer->ref_fd, buffer->ref_vma);
	if(buffer->ref_vma || buffer->ref_fd)
		return;
	/* Remove it from the buffer list */
	list_del(&buffer->list);
	/* Free mem */
	free_pages_exact(buffer->ptr, buffer->size);
}

/* Misc device layer */

static ssize_t
pxa3xx_gcu_misc_write(struct file *filp, const char *buff,
		size_t count, loff_t *offp)
{
	int ret;
	unsigned long flags;
	struct pxa3xx_gcu_batch	*buffer;
	struct pxa3xx_gcu_priv *priv =
		container_of(filp->f_op, struct pxa3xx_gcu_priv, misc_fops);

	int words = count / 4;

	/* Does not need to be atomic. There's a lock in user space,
	* but anyhow, this is just for statistics. */
	priv->shared->num_writes++;

	priv->shared->num_words += words;

	/* Last word reserved for batch buffer end command */
	if (words >= PXA3XX_GCU_BATCH_WORDS)
		return -E2BIG;

	/* Wait for a free buffer */
	if (!priv->free) {
		ret = pxa3xx_gcu_wait_free(priv);
		if (ret < 0)
			return ret;
	}

	/*
	 * Get buffer from free list
	 */
	spin_lock_irqsave(&priv->spinlock, flags);

	buffer = priv->free;
	priv->free = buffer->next;
	/* If there are no free buffers left in fact */
	if(priv->free->filled)
		priv->free = NULL;

	spin_unlock_irqrestore(&priv->spinlock, flags);


	/* Copy data from user into buffer */
	ret = copy_from_user(buffer->ptr, buff, words * 4);
	if (ret) {
		spin_lock_irqsave(&priv->spinlock, flags);
		priv->free = buffer;
		spin_unlock_irqrestore(&priv->spinlock, flags);
		return -EFAULT;
	}

	buffer->length = words;

	/* Append batch buffer end command */
	buffer->ptr[words] = 0x01000000;

	/*
	 * Add buffer to filled list, and probably run batches
	 */
	spin_lock_irqsave(&priv->spinlock, flags);

	buffer->filled = 1;
	if (priv->exec_first == NULL)
		priv->exec_first = buffer;

	priv->filled_last = buffer;

	if (!priv->shared->hw_running)
		run_ready(priv);

	spin_unlock_irqrestore(&priv->spinlock, flags);

	return words * 4;
}


static long
pxa3xx_gcu_misc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned long flags;
	struct pxa3xx_gcu_priv *priv =
		container_of(filp->f_op, struct pxa3xx_gcu_priv, misc_fops);
	struct pxa3xx_gcu_buffer buff;
	struct pxa3xx_gcu_buffer_priv *buff_priv, *file_buff;

	file_buff = (struct pxa3xx_gcu_buffer_priv*) filp->private_data;

	switch (cmd) {
	case PXA3XX_GCU_IOCTL_RESET:
		spin_lock_irqsave(&priv->spinlock, flags);
		pxa3xx_gcu_reset(priv);
		spin_unlock_irqrestore(&priv->spinlock, flags);
		return 0;

	case PXA3XX_GCU_IOCTL_WAIT_IDLE:
		return pxa3xx_gcu_wait_idle(priv);

	case PXA3XX_GCU_IOCTL_B_GETINFO:
		if(file_buff == NULL)
			return -ENXIO;

		buff.id = file_buff->id;
		buff.length = file_buff->size;
		buff.phys = file_buff->phys;
		return copy_to_user((void *) arg, &buff,\
				sizeof(struct pxa3xx_gcu_buffer));

	case PXA3XX_GCU_IOCTL_B_ATTACH:
		list_for_each_entry(buff_priv, &priv->buffers, list)
			if(buff_priv->id == (unsigned int) arg) {
				/* Found requested buffer */
				if(file_buff) {
					file_buff->ref_fd--;
					pxa3xx_gcu_buffer_cleanup(file_buff);
				}
				buff_priv->ref_fd++;
				filp->private_data = buff_priv;
				return 0;
			}
		/* Requested ID not found */
		return -ENXIO;

	case PXA3XX_GCU_IOCTL_B_ALLOC:
		buff_priv = kzalloc(sizeof(struct pxa3xx_gcu_buffer_priv),\
					GFP_KERNEL);
		if(buff_priv == NULL)
			return -ENOMEM;

		buff_priv->size = (unsigned int) arg;
		if(buff_priv->size > PXA3XX_GCU_BUFFER_MAXSZ) {
			kfree(buff_priv);
			return -EINVAL;
		}
		buff_priv->ptr = alloc_pages_exact(buff_priv->size, GFP_KERNEL);
		if(buff_priv->ptr == NULL) {
			kfree(buff_priv);
			return -ENOMEM;
		}
		buff_priv->phys = virt_to_phys(buff_priv->ptr);

		/* Here UID generation comes.. Ha ha */
		buff_priv->id = (unsigned int) buff_priv->phys;
		list_add(&buff_priv->list, &priv->buffers);

		/* Dereference old buffer */
		if(file_buff) {
			file_buff->ref_fd--;
			pxa3xx_gcu_buffer_cleanup(file_buff);
		}

		buff_priv->ref_fd++;
		filp->private_data = buff_priv;

		return 0;

	case PXA3XX_GCU_IOCTL_B_FREE:
		if(file_buff == NULL)
			return -ENXIO;
		file_buff->ref_fd--;
		pxa3xx_gcu_buffer_cleanup(file_buff);
		filp->private_data = NULL;

		return 0;
	}

	return -ENOSYS;
}

/* This is needed to count VMA references to buffers */
static void pxa3xx_gcu_vma_open(struct vm_area_struct *vma)
{
	struct pxa3xx_gcu_buffer_priv *buff_priv;
	buff_priv = (struct pxa3xx_gcu_buffer_priv*) vma->vm_private_data;
	BUG_ON(buff_priv == NULL);
	buff_priv->ref_vma++;
	printk(KERN_INFO "pxa3xx-gcu: buff %d fdref %d vmaref %d\n",\
		buff_priv->id, buff_priv->ref_fd, buff_priv->ref_vma);
}

static void pxa3xx_gcu_vma_close(struct vm_area_struct *vma)
{
	struct pxa3xx_gcu_buffer_priv *buff_priv;
	buff_priv = (struct pxa3xx_gcu_buffer_priv*) vma->vm_private_data;
	BUG_ON(buff_priv == NULL);
	buff_priv->ref_vma--;
	printk(KERN_INFO "pxa3xx-gcu: buff %d fdref %d vmaref %d\n",\
		buff_priv->id, buff_priv->ref_fd, buff_priv->ref_vma);
	pxa3xx_gcu_buffer_cleanup(buff_priv);
}

static struct vm_operations_struct pxa3xx_gcu_vma_ops = {
	.open = pxa3xx_gcu_vma_open,
	.close = pxa3xx_gcu_vma_close,
};

static int
pxa3xx_gcu_misc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned int size = vma->vm_end - vma->vm_start;
	struct pxa3xx_gcu_priv *priv =
		container_of(filp->f_op, struct pxa3xx_gcu_priv, misc_fops);
	struct pxa3xx_gcu_buffer_priv *file_buff;
	
	switch (vma->vm_pgoff) {
	case 0:
		/* hand out the shared data area */
		if (size != SHARED_SIZE)
			return -EINVAL;

		return dma_mmap_coherent(NULL, vma,
			priv->shared, priv->shared_phys, size);

	case SHARED_SIZE >> PAGE_SHIFT:
		/* hand out the MMIO base for direct register access
		 * from userspace */
		if (size != resource_size(priv->resource_mem))
			return -EINVAL;

		vma->vm_flags |= VM_IO;
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

		return io_remap_pfn_range(vma, vma->vm_start,
				priv->resource_mem->start >> PAGE_SHIFT,
				size, vma->vm_page_prot);
	case 1 + (SHARED_SIZE >> PAGE_SHIFT):
		/* Try to mmap buffer associated with filp */
		file_buff = (struct pxa3xx_gcu_buffer_priv*) filp->private_data;
		if(file_buff == NULL)
			return -ENXIO;
		if(size != file_buff->size)
			return -EINVAL;

		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		vma->vm_ops = &pxa3xx_gcu_vma_ops;
		vma->vm_private_data = filp->private_data;

		return remap_pfn_range(vma, vma->vm_start,
				file_buff->phys >> PAGE_SHIFT,
				size, vma->vm_page_prot);
		pxa3xx_gcu_vma_open(vma);
	}

	return -EINVAL;
}


#ifdef PXA3XX_GCU_DEBUG_TIMER
static struct timer_list pxa3xx_gcu_debug_timer;

static void pxa3xx_gcu_debug_timedout(unsigned long ptr)
{
	struct pxa3xx_gcu_priv *priv = (struct pxa3xx_gcu_priv *) ptr;

	QERROR("Timer DUMP");

	/* init the timer structure */
	init_timer(&pxa3xx_gcu_debug_timer);
	pxa3xx_gcu_debug_timer.function = pxa3xx_gcu_debug_timedout;
	pxa3xx_gcu_debug_timer.data = ptr;
	pxa3xx_gcu_debug_timer.expires = jiffies + 5*HZ; /* one second */

	add_timer(&pxa3xx_gcu_debug_timer);
}

static void pxa3xx_gcu_init_debug_timer(void)
{
	pxa3xx_gcu_debug_timedout((unsigned long) &pxa3xx_gcu_debug_timer);
}
#else
static inline void pxa3xx_gcu_init_debug_timer(void) {}
#endif

static int
add_buffer(struct platform_device *dev,
	struct pxa3xx_gcu_priv *priv)
{
	struct pxa3xx_gcu_batch *buffer;

	buffer = kzalloc(sizeof(struct pxa3xx_gcu_batch), GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	buffer->ptr = dma_alloc_coherent(&dev->dev, PXA3XX_GCU_BATCH_WORDS * 4,
					&buffer->phys, GFP_KERNEL);
	if (!buffer->ptr) {
		kfree(buffer);
		return -ENOMEM;
	}

	if(priv->first == NULL)
		priv->first = priv->last = buffer;

	buffer->next = priv->first;
	priv->last->next = buffer;
	priv->last = buffer;

	if(priv->free == NULL)
		priv->free = buffer;
	return 0;
}

static void
free_buffers(struct platform_device *dev,
	struct pxa3xx_gcu_priv *priv)
{
	struct pxa3xx_gcu_batch *next, *buffer = priv->first;

	if(!buffer)
		return;
	do {
		next = buffer->next;

		dma_free_coherent(&dev->dev, PXA3XX_GCU_BATCH_WORDS * 4,
				buffer->ptr, buffer->phys);

		kfree(buffer);

		buffer = next;
	} while (buffer != priv->first);

	priv->free = NULL;
	priv->first = NULL;
	priv->last = NULL;
}

static int __devinit
pxa3xx_gcu_probe(struct platform_device *dev)
{
	int i, ret, irq;
	struct resource *r;
	struct pxa3xx_gcu_priv *priv;

	priv = kzalloc(sizeof(struct pxa3xx_gcu_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	for (i = 0; i < 8; i++) {
		ret = add_buffer(dev, priv);
		if (ret) {
			dev_err(&dev->dev, "failed to allocate DMA memory\n");
			goto err_free_priv;
		}
	}

	INIT_LIST_HEAD(&priv->buffers);
	init_waitqueue_head(&priv->wait_idle);
	init_waitqueue_head(&priv->wait_free);
	spin_lock_init(&priv->spinlock);

	/* we allocate the misc device structure as part of our own allocation,
	 * so we can get a pointer to our priv structure later on with
	 * container_of(). This isn't really necessary as we have a fixed minor
	 * number anyway, but this is to avoid statics. */

	priv->misc_fops.owner	= THIS_MODULE;
	priv->misc_fops.write	= pxa3xx_gcu_misc_write;
	priv->misc_fops.unlocked_ioctl = pxa3xx_gcu_misc_ioctl;
	priv->misc_fops.mmap	= pxa3xx_gcu_misc_mmap;

	priv->misc_dev.minor	= MISCDEV_MINOR,
	priv->misc_dev.name	= DRV_NAME,
	priv->misc_dev.fops	= &priv->misc_fops,

	/* register misc device */
	ret = misc_register(&priv->misc_dev);
	if (ret < 0) {
		dev_err(&dev->dev, "misc_register() for minor %d failed\n",
			MISCDEV_MINOR);
		goto err_free_priv;
	}

	/* handle IO resources */
	r = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&dev->dev, "no I/O memory resource defined\n");
		ret = -ENODEV;
		goto err_misc_deregister;
	}

	if (!request_mem_region(r->start, resource_size(r), dev->name)) {
		dev_err(&dev->dev, "failed to request I/O memory\n");
		ret = -EBUSY;
		goto err_misc_deregister;
	}

	priv->mmio_base = ioremap_nocache(r->start, resource_size(r));
	if (!priv->mmio_base) {
		dev_err(&dev->dev, "failed to map I/O memory\n");
		ret = -EBUSY;
		goto err_free_mem_region;
	}

	/* allocate dma memory */
	priv->shared = dma_alloc_coherent(&dev->dev, SHARED_SIZE,
					&priv->shared_phys, GFP_KERNEL);

	if (!priv->shared) {
		dev_err(&dev->dev, "failed to allocate DMA memory\n");
		ret = -ENOMEM;
		goto err_free_io;
	}

	/* enable the clock */
	priv->clk = clk_get(&dev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(&dev->dev, "failed to get clock\n");
		ret = -ENODEV;
		goto err_free_dma;
	}

	ret = clk_enable(priv->clk);
	if (ret < 0) {
		dev_err(&dev->dev, "failed to enable clock\n");
		goto err_put_clk;
	}

	/* request the IRQ */
	irq = platform_get_irq(dev, 0);
	if (irq < 0) {
		dev_err(&dev->dev, "no IRQ defined\n");
		ret = -ENODEV;
		goto err_put_clk;
	}

	ret = request_irq(irq, pxa3xx_gcu_handle_irq,
			0, DRV_NAME, priv);
	if (ret) {
		dev_err(&dev->dev, "request_irq failed\n");
		ret = -EBUSY;
		goto err_put_clk;
	}

	platform_set_drvdata(dev, priv);
	priv->resource_mem = r;
	pxa3xx_gcu_reset(priv);
	pxa3xx_gcu_init_debug_timer();

	dev_info(&dev->dev, "registered @0x%p, DMA 0x%p (%d bytes), IRQ %d\n",
			(void *) r->start, (void *) priv->shared_phys,
			SHARED_SIZE, irq);
	return 0;

err_put_clk:
	clk_disable(priv->clk);
	clk_put(priv->clk);

err_free_dma:
	dma_free_coherent(&dev->dev, SHARED_SIZE,
			priv->shared, priv->shared_phys);

err_free_io:
	iounmap(priv->mmio_base);

err_free_mem_region:
	release_mem_region(r->start, resource_size(r));

err_misc_deregister:
	misc_deregister(&priv->misc_dev);

err_free_priv:
	platform_set_drvdata(dev, NULL);
	free_buffers(dev, priv);
	kfree(priv);
	return ret;
}

static int __devexit
pxa3xx_gcu_remove(struct platform_device *dev)
{
	struct pxa3xx_gcu_priv *priv = platform_get_drvdata(dev);
	struct resource *r = priv->resource_mem;

	pxa3xx_gcu_wait_idle(priv);

	misc_deregister(&priv->misc_dev);
	dma_free_coherent(&dev->dev, SHARED_SIZE,
			priv->shared, priv->shared_phys);
	iounmap(priv->mmio_base);
	release_mem_region(r->start, resource_size(r));
	platform_set_drvdata(dev, NULL);
	clk_disable(priv->clk);
	free_buffers(dev, priv);
	kfree(priv);

	return 0;
}

static struct platform_driver pxa3xx_gcu_driver = {
	.probe	  = pxa3xx_gcu_probe,
	.remove	 = __devexit_p(pxa3xx_gcu_remove),
	.driver	 = {
		.owner  = THIS_MODULE,
		.name   = DRV_NAME,
	},
};

module_platform_driver(pxa3xx_gcu_driver);

MODULE_DESCRIPTION("PXA3xx graphics controller unit driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(MISCDEV_MINOR);
MODULE_AUTHOR("Janine Kropp <nin@directfb.org>, "
		"Denis Oliver Kropp <dok@directfb.org>, "
		"Daniel Mack <daniel@caiaq.de>");
