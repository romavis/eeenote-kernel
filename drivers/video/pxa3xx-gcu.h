#ifndef __PXA3XX_GCU_H__
#define __PXA3XX_GCU_H__

#include <linux/types.h>

/* Number of 32bit words in display list (ring buffer). */
#define PXA3XX_GCU_BUFFER_WORDS  ((256 * 1024 - 256) / 4)

/* To be increased when breaking the ABI */
#define PXA3XX_GCU_SHARED_MAGIC  0x30000001

#define PXA3XX_GCU_BATCH_WORDS   8192
/* Scratchpad buffer limitations */
#define PXA3XX_GCU_BUFFER_MAXSZ  (2*1024*1024)

struct pxa3xx_gcu_shared {
	u32            buffer[PXA3XX_GCU_BUFFER_WORDS];

	bool           hw_running;

	unsigned long  buffer_phys;

	unsigned int   num_words;
	unsigned int   num_writes;
	unsigned int   num_done;
	unsigned int   num_interrupts;
	unsigned int   num_wait_idle;
	unsigned int   num_wait_free;
	unsigned int   num_idle;

	u32            magic;
};

struct pxa3xx_gcu_buffer {
	unsigned int   id;
	unsigned int   length;
	unsigned long  phys;
};
/* Initialization and synchronization.
 * Hardware is started upon write(). */
#define PXA3XX_GCU_IOCTL_RESET		_IO('G', 0)
#define PXA3XX_GCU_IOCTL_WAIT_IDLE	_IO('G', 2)

/* Params: ptr to struct pxa3xx_gcu_buffer */
#define PXA3XX_GCU_IOCTL_B_GETINFO	_IOR('G', 4, struct pxa3xx_gcu_buffer*)
/* Params: unsigned int buffer_id */
#define PXA3XX_GCU_IOCTL_B_ATTACH	_IOW('G', 5, unsigned int)
/* Params: unsigned int buffer_size */
#define PXA3XX_GCU_IOCTL_B_ALLOC	_IOW('G', 6, unsigned int)
#define PXA3XX_GCU_IOCTL_B_FREE		_IO('G', 7)

#endif /* __PXA3XX_GCU_H__ */

