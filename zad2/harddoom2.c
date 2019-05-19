// TODO remove unused headers
// TODO check last year's scoring
// TODO fix "--[tab]"
// TODO sync_queue_mutex -> queue_mutex
// TODO async :X
// TODO test suspend after making code async
// TODO readme note: suspend, needs to unmount p9
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/cdev.h>
#include <linux/cred.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/anon_inodes.h>
#include <linux/completion.h>
#include "harddoom2.h"
#include "doomcode2.h"
#include "doomdev2.h"

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("HardDoom ][ tm device");

#define HARDDOOM2_MAX_DEVICES	256
#define HARDDOOM2_DMA_MASK		DMA_BIT_MASK(40)
#define HARDDOOM2_CMD_SETUP_SURF_DST_PT_IDX		1
#define HARDDOOM2_CMD_SETUP_SURF_SRC_PT_IDX		2
#define HARDDOOM2_CMD_SETUP_TEXTURE_PT_IDX		3
#define HARDDOOM2_CMD_SETUP_FLAT_PT_IDX			4
#define HARDDOOM2_CMD_SETUP_TRANSLATION_PT_IDX	5
#define HARDDOOM2_CMD_SETUP_COLORMAP_PT_IDX		6
#define HARDDOOM2_CMD_SETUP_TRANMAP_PT_IDX		7
#define HARDDOOM2_CMD_SETUP_PT_SHIFT	8
#define HARDDOOM2_INTR_ERROR(i)			(0x10 << (i))
#define HARDDOOM2_INTR_ERRORS_CNT		12
#define HARDDOOM2_INTR_ERRORS_MASK		0xfff0
#define HARDDOOM2_TEXTURE_LIMIT_UNITS	64
#define HARDDOOM2_FUZZ_POS_MAX			55
#define DOOMDEV2_NO_AVAILABLE_MINOR		(-1)
#define DOOMDEV2_DMA_BUFFER_NO_VALUE	(-1)
#define DOOMDEV2_MIN_SURFACE_SIZE 		1
#define DOOMDEV2_SURFACE_WIDTH_DIVIDER	64
#define DOOMDEV2_MAX_SURFACE_SIZE 		2048
#define DOOMDEV2_MIN_BUFFER_SIZE		1
#define DOOMDEV2_MAX_BUFFER_SIZE		(2048 * 2048)
#define DOOMDEV2_BUFFER_FLAT_SIZE		(1 << 12)
#define DOOMDEV2_BUFFER_COLORMAP_SIZE	256
#define DOOMDEV2_BUFFER_TRANMAP_SIZE	(1 << 16)
#define DOOMDEV2_CTX_BUFFERS_CNT 		(sizeof(struct doomdev2_ctx_buffers) / sizeof(struct doomdev2_ctx_buffers*))
#define HARDDOOM2_MAX_CMD_CNT			(DOOMDEV2_MAX_BUFFER_SIZE / sizeof(struct harddoom2_cmd_buf))
#define HARDDOOM2_ASYNC_CMD_INTERVAL	(HARDDOOM2_MAX_CMD_CNT / 32)

static const char harddoom2_name[] = { "harddoom2" };
static struct pci_driver harddoom2_driver;
static dev_t doomdev2_major;
static struct class doomdev2_class = {
	.name = harddoom2_name,
	.owner = THIS_MODULE,
};
static DECLARE_BITMAP(doomdev2_used_minors, HARDDOOM2_MAX_DEVICES);
static DEFINE_MUTEX(doomdev2_used_minors_mutex);


struct doomdev2_ctx_buffers {
	struct doomdev2_dma_buffer *surf_dst;
	struct doomdev2_dma_buffer *surf_src;
	struct doomdev2_dma_buffer *texture;
	struct doomdev2_dma_buffer *flat;
	struct doomdev2_dma_buffer *colormap;
	struct doomdev2_dma_buffer *translation;
	struct doomdev2_dma_buffer *tranmap;
};

struct harddoom2_device {
	struct pci_dev *pdev;
	void __iomem *bar0;
	struct cdev doomdev2;
	bool ready; // not ready when suspending

	struct doomdev2_dma_buffer *cmd_queue;
	uint32_t cmd_write_idx;
	uint32_t cmd_read_idx_cached;
	struct completion pong_async;

	struct doomdev2_ctx_buffers last_active_bufs; // note: NEVER deref pointers inside (we hold weak reference here)
	struct mutex sync_queue_mutex; // used for sending to queue and operating on last_active_bufs member (todo more semantics, explain)
	struct completion pong_sync;
};

struct doomdev2_ctx {
	struct harddoom2_device *hdev;

	struct doomdev2_ctx_buffers active_bufs;
	struct mutex active_bufs_mutex;
};

struct doomdev2_dma_page {
	void *phys_addr;
	dma_addr_t dma_handle;
};

// represents surfaces and buffers
struct doomdev2_dma_buffer {
	struct harddoom2_device *hdev;
	int32_t width;
	int32_t height; // if zero, then it's a buffer, not a surface
	struct file *file;

	size_t pages_cnt;
	struct doomdev2_dma_page pages[0];
};
#define DOOMDEV2_DMA_BUFFER_SIZE(dma_buf) \
	((dma_buf)->width * max(1, (dma_buf)->height))

struct harddoom2_cmd_buf {
	uint32_t w[HARDDOOM2_CMD_SEND_SIZE];
};

static irqreturn_t harddoom2_irq_handler(int _irq, void *dev) {
	static const char *errors[] = {
		"FE_ERROR",
		"CMD_OVERFLOW",
		"SURF_DST_OVERFLOW",
		"SURF_SRC_OVERFLOW",
		"PAGE_FAULT_CMD",
		"PAGE_FAULT_SURF_DST",
		"PAGE_FAULT_SURF_SRC",
		"PAGE_FAULT_TEXTURE",
		"PAGE_FAULT_FLAT",
		"PAGE_FAULT_TRANSLATION",
		"PAGE_FAULT_COLORMAP",
		"PAGE_FAULT_TRANMAP",
	};
	struct harddoom2_device *hdev = (struct harddoom2_device*) dev;
	uint32_t flags, error_flags, intr_enabled;
	int i;

	// read interruptions
	intr_enabled = ioread32(hdev->bar0 + HARDDOOM2_INTR_ENABLE);
	flags = ioread32(hdev->bar0 + HARDDOOM2_INTR);
	if (!(flags & intr_enabled)) {
		return IRQ_NONE;
	}

	// handle interruptions
	if (flags & HARDDOOM2_INTR_FENCE) {
		// todo
		BUG();
	}

	if (flags & HARDDOOM2_INTR_PONG_SYNC) {
		iowrite32(HARDDOOM2_INTR_PONG_SYNC, hdev->bar0 + HARDDOOM2_INTR);
		complete(&hdev->pong_sync);
	}

	if (flags & intr_enabled & HARDDOOM2_INTR_PONG_ASYNC) {
		iowrite32(HARDDOOM2_INTR_PONG_ASYNC, hdev->bar0 + HARDDOOM2_INTR);
		complete(&hdev->pong_async);
	}

	// handle errors
	error_flags = flags & HARDDOOM2_INTR_ERRORS_MASK;
	if (error_flags) {
		for (i = 0; i < HARDDOOM2_INTR_ERRORS_CNT; i++) {
			if (error_flags & HARDDOOM2_INTR_ERROR(i)) {
				dev_err(&hdev->pdev->dev, "Error: %s\n", errors[i]);
			}
		}
		iowrite32(error_flags, hdev->bar0 + HARDDOOM2_INTR);
		BUG();
	}

	return IRQ_HANDLED;
}

static struct doomdev2_dma_buffer *doomdev2_dma_buffer_alloc(struct harddoom2_device *hdev, size_t pages_cnt) {
	struct doomdev2_dma_buffer *dma_buf;
	size_t alloc_size;
	struct doomdev2_dma_page *page_it, *pages_end;

	// alloc describing structure
	alloc_size = sizeof(struct doomdev2_dma_buffer) + sizeof(struct doomdev2_dma_page) * pages_cnt;
	dma_buf = (struct doomdev2_dma_buffer*) kmalloc(alloc_size, GFP_KERNEL);
	if (!dma_buf) {
		return NULL;
	}
	dma_buf->hdev = hdev;
	dma_buf->width = DOOMDEV2_DMA_BUFFER_NO_VALUE;   // set later in other function
	dma_buf->height = DOOMDEV2_DMA_BUFFER_NO_VALUE;  // ^
	dma_buf->file = NULL;
	dma_buf->pages_cnt = pages_cnt;

	// alloc pages
	pages_end = dma_buf->pages + pages_cnt;
	for (page_it = dma_buf->pages; page_it < pages_end; page_it++) {
		page_it->phys_addr = dma_alloc_coherent(&hdev->pdev->dev, HARDDOOM2_PAGE_SIZE,
			&page_it->dma_handle, GFP_KERNEL | __GFP_ZERO);
		if (!page_it->phys_addr) {
			goto dma_alloc_err;
		}
		BUG_ON(((uint64_t)page_it->phys_addr) % HARDDOOM2_PAGE_SIZE != 0
			|| page_it->dma_handle % HARDDOOM2_PAGE_SIZE != 0);
	}

	return dma_buf;

dma_alloc_err:
	for (page_it--; page_it >= dma_buf->pages; page_it--) {
		dma_free_coherent(&hdev->pdev->dev, HARDDOOM2_PAGE_SIZE, page_it->phys_addr, page_it->dma_handle);
	}
	kfree(dma_buf);

	return NULL;
}

static void doomdev2_dma_buffer_del(struct doomdev2_dma_buffer *dma_buf) {
	struct doomdev2_dma_page *page_it, *pages_end;

	pages_end = dma_buf->pages + dma_buf->pages_cnt;
	for (page_it = dma_buf->pages; page_it < pages_end; page_it++) {
		dma_free_coherent(&dma_buf->hdev->pdev->dev, HARDDOOM2_PAGE_SIZE, page_it->phys_addr, page_it->dma_handle);
	}

	kfree(dma_buf);
}

static int doomdev2_dma_buffer_release(struct inode *_ino, struct file *filep) {
	struct doomdev2_dma_buffer *dma_buf = (struct doomdev2_dma_buffer*) filep->private_data;

	doomdev2_dma_buffer_del(dma_buf);

	return 0;
}

static loff_t doomdev2_dma_buffer_llseek(struct file *filep, loff_t off, int whence) {
	struct doomdev2_dma_buffer *dma_buf = (struct doomdev2_dma_buffer*) filep->private_data;
	size_t buf_size = DOOMDEV2_DMA_BUFFER_SIZE(dma_buf);
	loff_t ret;

	ret = mutex_lock_interruptible(&filep->f_pos_lock);
	if (ret) {
		return ret;
	}

	switch (whence) {
		case SEEK_SET:
			ret = off;
			break;
		case SEEK_CUR:
			ret = filep->f_pos + off;
			break;
		case SEEK_END:
			ret = buf_size + off;
			break;
		default:
			ret = -EINVAL;
			goto err_out;
	}

	filep->f_pos = ret;

err_out:
	mutex_unlock(&filep->f_pos_lock);

	return ret;
}


static ssize_t doomdev2_dma_buffer_io(struct file *filep, char __user *buff_read, const char __user *buff_write,
		size_t count, loff_t *offp) {
	struct doomdev2_dma_buffer *dma_buf = (struct doomdev2_dma_buffer*) filep->private_data;
	loff_t buf_size = DOOMDEV2_DMA_BUFFER_SIZE(dma_buf);
	ssize_t ret;

	BUG_ON((buff_read && buff_write) || (!buff_read && !buff_write));

	ret = mutex_lock_interruptible(&dma_buf->hdev->sync_queue_mutex);
	if (ret) {
		return ret;
	}
	if (!dma_buf->hdev->ready) {
		ret = -EAGAIN;
		goto exit_op;
	}

	ret = 0;

	if (*offp < 0) {
		ret = -EINVAL;
		goto exit_op;
	}

	while (*offp < buf_size && ret < count) {
		ssize_t to_copy = min(
			min(round_up(*offp + 1, HARDDOOM2_PAGE_SIZE), buf_size) - *offp,
			(loff_t) (count - ret));
		size_t page_idx = *offp / HARDDOOM2_PAGE_SIZE + 1; // +1 because of page table at idx 0
		ssize_t page_offset = *offp % HARDDOOM2_PAGE_SIZE;
		unsigned long not_copied = 0;

		if (buff_write) {
			not_copied = copy_from_user(dma_buf->pages[page_idx].phys_addr + page_offset, buff_write + ret, to_copy);
		}
		else {
			not_copied = copy_to_user(buff_read + ret, dma_buf->pages[page_idx].phys_addr + page_offset, to_copy);
		}
		ret += to_copy - not_copied;
		*offp += to_copy - not_copied;
		if (not_copied > 0) {
			if (ret == 0) {
				ret = -EFAULT;
			}
			goto exit_op;
		}
	}

	if (buff_write && ret == 0 && count > 0) {
		ret = -ENOSPC;
	}

exit_op:
	mutex_unlock(&dma_buf->hdev->sync_queue_mutex);

	return ret;
}

static ssize_t doomdev2_dma_buffer_read(struct file *filep, char __user *buff, size_t count, loff_t *offp) {
	return doomdev2_dma_buffer_io(filep, buff, NULL, count, offp);
}

static ssize_t doomdev2_dma_buffer_write(struct file *filep, const char __user *buff, size_t count, loff_t *offp) {
	return doomdev2_dma_buffer_io(filep, NULL, buff, count, offp);
}

static struct file_operations doomdev2_dma_buffer_fops = {
	.owner = THIS_MODULE,
	.release = doomdev2_dma_buffer_release,
	.llseek = doomdev2_dma_buffer_llseek,
	.read = doomdev2_dma_buffer_read,
	.write = doomdev2_dma_buffer_write,
};

static int doomdev2_dma_buffer_install_fd(struct doomdev2_dma_buffer *dma_buf) {
	int fd, err;
	struct file *f;
	int flags = O_RDWR;

	err = get_unused_fd_flags(flags);
	if (err < 0) {
		return err;
	}
	fd = err;

	f = anon_inode_getfile("doomdev2_dma_buffer", &doomdev2_dma_buffer_fops, dma_buf, flags);
	if (IS_ERR(f)) {
		err = PTR_ERR(f);
		goto err_put_unused_fd;
	}

	dma_buf->file = f;
	f->f_mode |= FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE;

	fd_install(fd, f);

	return fd;

err_put_unused_fd:
	put_unused_fd(fd);

	return err;
}

static void doomdev2_dma_buffer_format_page_table(struct doomdev2_dma_buffer *dma_buf, bool writable) {
	// todo (optimization) put page table on the last page if possible
	uint32_t *page_it, *pages_end;
	struct doomdev2_dma_page *dma_page_it;

	page_it = (uint32_t*) dma_buf->pages[0].phys_addr;
	pages_end = page_it + dma_buf->pages_cnt - 1;
	dma_page_it = &dma_buf->pages[1];
	for (; page_it < pages_end; page_it++, dma_page_it++) {
		*page_it = HARDDOOM2_PTE_VALID
			| (writable ? HARDDOOM2_PTE_WRITABLE : 0)
			| ((dma_page_it->dma_handle >> HARDDOOM2_PAGE_SHIFT) << HARDDOOM2_PTE_PHYS_SHIFT);
	}
}

static long doomdev2_dma_buffer_alloc_with_fd(struct harddoom2_device *hdev, size_t dma_alloc_size, int32_t width, int32_t height) {
	int err;
	size_t dma_pages_cnt;
	struct doomdev2_dma_buffer *dma_buf;

	dma_pages_cnt = DIV_ROUND_UP(dma_alloc_size, HARDDOOM2_PAGE_SIZE) + 1; // +1 for page table

	dma_buf = doomdev2_dma_buffer_alloc(hdev, dma_pages_cnt);
	if (!dma_buf) {
		return -ENOMEM;
	}

	dma_buf->width = width;
	dma_buf->height = height;
	doomdev2_dma_buffer_format_page_table(dma_buf, true);

	err = doomdev2_dma_buffer_install_fd(dma_buf); // must be called last, since it installs fd!
	if (err < 0) {
		goto err_fd_alloc;
	}

	return err; // return fd

err_fd_alloc:
	doomdev2_dma_buffer_del(dma_buf);

	return err;
}

static long doomdev2_ioctl_create_surface(struct harddoom2_device *hdev, struct doomdev2_ioctl_create_surface *data_cs) {
	size_t dma_alloc_size;

	if (data_cs->width < DOOMDEV2_MIN_SURFACE_SIZE || data_cs->width > DOOMDEV2_MAX_SURFACE_SIZE
			|| data_cs->height < DOOMDEV2_MIN_SURFACE_SIZE || data_cs->height > DOOMDEV2_MAX_SURFACE_SIZE) {
		return -EOVERFLOW;
	}
	if (data_cs->width % DOOMDEV2_SURFACE_WIDTH_DIVIDER != 0) {
		return -EINVAL;
	}
	dma_alloc_size = data_cs->width * data_cs->height;

	return doomdev2_dma_buffer_alloc_with_fd(hdev, dma_alloc_size, data_cs->width, data_cs->height);
}

static long doomdev2_ioctl_create_buffer(struct harddoom2_device *hdev, struct doomdev2_ioctl_create_buffer *data_cb) {
	if (data_cb->size < DOOMDEV2_MIN_BUFFER_SIZE || data_cb->size > DOOMDEV2_MAX_BUFFER_SIZE) {
		return -EOVERFLOW;
	}

	return doomdev2_dma_buffer_alloc_with_fd(hdev, data_cb->size, data_cb->size, 0);
}

static void doomdev2_ctx_buffers_ref_count_manip(struct doomdev2_ctx_buffers *bufs, int diff) {
	if (diff == +1) {
		bufs->surf_dst    ? get_file(bufs->surf_dst->file) : 0;
		bufs->surf_src    ? get_file(bufs->surf_src->file) : 0;
		bufs->texture     ? get_file(bufs->texture->file) : 0;
		bufs->flat        ? get_file(bufs->flat->file) : 0;
		bufs->colormap    ? get_file(bufs->colormap->file) : 0;
		bufs->translation ? get_file(bufs->translation->file) : 0;
		bufs->tranmap     ? get_file(bufs->tranmap->file) : 0;
	}
	else if (diff == -1) {
		bufs->surf_dst    ? fput(bufs->surf_dst->file) : 0;
		bufs->surf_src    ? fput(bufs->surf_src->file) : 0;
		bufs->texture     ? fput(bufs->texture->file) : 0;
		bufs->flat        ? fput(bufs->flat->file) : 0;
		bufs->colormap    ? fput(bufs->colormap->file) : 0;
		bufs->translation ? fput(bufs->translation->file) : 0;
		bufs->tranmap     ? fput(bufs->tranmap->file) : 0;
	}
	else {
		BUG();
	}
}

static long doomdev2_ioctl_setup(struct doomdev2_ctx *ctx, struct doomdev2_ioctl_setup *data_s) {
	int ret = 0;
	typeof(ctx->active_bufs) tmp_bufs;
	struct fd fds[DOOMDEV2_CTX_BUFFERS_CNT];
	int i;

	inline int validate_set_buf(struct doomdev2_dma_buffer **dst, int32_t fd, struct fd *fd_out,
			bool is_surface, size_t size_divider, bool exact_size) {
		struct doomdev2_dma_buffer *dma_buf;
		struct fd f;
		int err;

		if (fd != -1) {
			size_t size;
			f = fdget(fd);
			if (!f.file) {
				return -EBADF;
			}
			if (f.file->f_op != &doomdev2_dma_buffer_fops) {
				err = -EINVAL;
				goto err_fdput;
			}
			dma_buf = (struct doomdev2_dma_buffer*) f.file->private_data;
			if (dma_buf->hdev != ctx->hdev) {
				err = -EINVAL;
				goto err_fdput;
			}
			if (is_surface ? dma_buf->height <= 0 : dma_buf->height > 0) {
				err = -EINVAL;
				goto err_fdput;
			}
			size = DOOMDEV2_DMA_BUFFER_SIZE(dma_buf);
			if (exact_size ? size != size_divider : size % size_divider != 0) {
				err = -EINVAL;
				goto err_fdput;
			}
			// alignment is ensured by dma_alloc_coherent
			*fd_out = f; // dma_buf is okay, fd need to be released by caller
		}
		else {
			dma_buf = NULL;
		}

		*dst = dma_buf;
		return 0;

	err_fdput:
		fdput(f);
		return err;
	}

	memset(&tmp_bufs, 0, sizeof(tmp_bufs));
	memset(fds, 0, sizeof(fds));

	// validate fds
#define DOOMDEV2_FD_CHECK(buf_name, no, ...) do { \
		ret = validate_set_buf(&tmp_bufs.buf_name, data_s->buf_name##_fd, fds + (no), __VA_ARGS__); \
		if (ret) { \
			goto exit_release_fds; \
		} \
	} while (0)

	DOOMDEV2_FD_CHECK(surf_dst,    0, true,  1, false);
	DOOMDEV2_FD_CHECK(surf_src,    1, true,  1, false);
	DOOMDEV2_FD_CHECK(texture,     2, false, 1, false);
	DOOMDEV2_FD_CHECK(flat,        3, false, DOOMDEV2_BUFFER_FLAT_SIZE,     false);
	DOOMDEV2_FD_CHECK(colormap,    4, false, DOOMDEV2_BUFFER_COLORMAP_SIZE, false);
	DOOMDEV2_FD_CHECK(translation, 5, false, DOOMDEV2_BUFFER_COLORMAP_SIZE, false);
	DOOMDEV2_FD_CHECK(tranmap,     6, false, DOOMDEV2_BUFFER_TRANMAP_SIZE, true);
#undef DOOMDEV2_FD_CHECK

	// symbolic context switch (change active buffers, not send SETUP command)
	ret = mutex_lock_interruptible(&ctx->active_bufs_mutex);
	if (ret) {
		goto exit_release_fds;
	}

	// increase ref count on new context
	doomdev2_ctx_buffers_ref_count_manip(&tmp_bufs, +1);

	// then decrease ref count on old context
	doomdev2_ctx_buffers_ref_count_manip(&ctx->active_bufs, -1);

	// set them as active buffers in ctx
	ctx->active_bufs = tmp_bufs;

	mutex_unlock(&ctx->active_bufs_mutex);

exit_release_fds:
	for (i = 0; i < DOOMDEV2_CTX_BUFFERS_CNT; i++) {
		if (fds[i].file) { // not empty
			fdput(fds[i]);
		}
	}

	return ret;
}

static long doomdev2_ioctl(struct file *filep, unsigned int cmd, unsigned long arg) {
	long ret;
	void __user *arg_ptr = (void __user*) arg;
	struct doomdev2_ctx *ctx = (struct doomdev2_ctx*) filep->private_data;

	struct doomdev2_ioctl_create_surface data_cs;
	struct doomdev2_ioctl_create_buffer data_cb;
	struct doomdev2_ioctl_setup data_s;

	switch (cmd) {
		case DOOMDEV2_IOCTL_CREATE_SURFACE:
			ret = copy_from_user(&data_cs, arg_ptr, sizeof(data_cs));
			if (ret) {
				return -EFAULT;
			}
			ret = doomdev2_ioctl_create_surface(ctx->hdev, &data_cs);
			break;
		case DOOMDEV2_IOCTL_CREATE_BUFFER:
			ret = copy_from_user(&data_cb, arg_ptr, sizeof(data_cb));
			if (ret) {
				return -EFAULT;
			}
			ret = doomdev2_ioctl_create_buffer(ctx->hdev, &data_cb);
			break;
		case DOOMDEV2_IOCTL_SETUP:
			ret = copy_from_user(&data_s, arg_ptr, sizeof(data_s));
			if (ret) {
				return -EFAULT;
			}
			ret = doomdev2_ioctl_setup(ctx, &data_s);
			break;
		default:
			ret = -ENOTTY;
	}

	return ret;
}

static int doomdev2_open(struct inode *ino, struct file *filep) {
	struct harddoom2_device *hdev;
	struct doomdev2_ctx *ctx = (struct doomdev2_ctx*) kmalloc(sizeof(struct doomdev2_ctx), GFP_KERNEL);
	if (!ctx) {
		printk(KERN_ERR "harddoom2: Cannot allocate memory (doomdev2_open)\n");
		return -ENOMEM;
	}

	hdev = container_of(ino->i_cdev, struct harddoom2_device, doomdev2);
	ctx->hdev = hdev;
	memset(&ctx->active_bufs, 0, sizeof(ctx->active_bufs));
	mutex_init(&ctx->active_bufs_mutex);

	filep->private_data = ctx;

	return 0;
}

static int doomdev2_release(struct inode *_ino, struct file *filep) {
	struct doomdev2_ctx *ctx = (struct doomdev2_ctx*) filep->private_data;

	mutex_lock(&ctx->active_bufs_mutex); // we can't fail in destructor & theoretically no one should access this no more
	doomdev2_ctx_buffers_ref_count_manip(&ctx->active_bufs, -1);
	mutex_unlock(&ctx->active_bufs_mutex);

	kfree(ctx);

	filep->private_data = NULL;

	return 0;
}

static int doomdev2_prepare_harddoom2_cmd(struct doomdev2_ctx *ctx, struct harddoom2_cmd_buf *buf_cmd,
		struct doomdev2_cmd *dev_cmd) {
	struct doomdev2_ctx_buffers *bufs = &ctx->active_bufs; // assumption: the mutex is locked
	int32_t width, height; // 32 bit to avoid overflows and underflows
	int32_t x_a, y_a, x_b, y_b; // as above
	uint32_t flags;
	uint32_t translation_idx, colormap_idx, flat_idx; // as above
	uint16_t texture_limit;
	uint16_t fuzz_start, fuzz_end;
	uint8_t fuzz_pos;

	memset(buf_cmd, 0, sizeof(*buf_cmd));

#define HARDDOOM2_ENSURE(cond) if (!(cond)) { printk(KERN_DEBUG "ensure failed at %d\n", __LINE__); return -EINVAL; } // todo remove printk

	switch (dev_cmd->type) {
	case DOOMDEV2_CMD_TYPE_COPY_RECT:
		x_a = dev_cmd->copy_rect.pos_dst_x;
		y_a = dev_cmd->copy_rect.pos_dst_y;
		x_b = dev_cmd->copy_rect.pos_src_x;
		y_b = dev_cmd->copy_rect.pos_src_y;
		width = dev_cmd->copy_rect.width;
		height = dev_cmd->copy_rect.height;
		flags = HARDDOOM2_CMD_FLAG_INTERLOCK;

		HARDDOOM2_ENSURE(bufs->surf_dst && bufs->surf_src);
		HARDDOOM2_ENSURE(x_a + width - 1 < bufs->surf_dst->width
			&& y_a + height - 1 < bufs->surf_dst->height
			&& x_b + width - 1 < bufs->surf_src->width
			&& y_b + height - 1 < bufs->surf_src->height);
		if (bufs->surf_dst == bufs->surf_src) {
			HARDDOOM2_ENSURE(
				(x_a + width  - 1 < x_b || x_b + width  - 1 < x_a) && // one rectangle is on the left to the other one, and
				(y_a + height - 1 < y_b || y_b + height - 1 < y_a)    // one rectangle is under the other one
			);
		}

		buf_cmd->w[0] = HARDDOOM2_CMD_TYPE_COPY_RECT | flags;
		buf_cmd->w[2] = HARDDOOM2_CMD_W2(x_a, y_a, 0);
		buf_cmd->w[3] = HARDDOOM2_CMD_W3(x_b, y_b);
		buf_cmd->w[6] = HARDDOOM2_CMD_W6_A(width, height, 0);
		break;

	case DOOMDEV2_CMD_TYPE_FILL_RECT:
		x_a = dev_cmd->fill_rect.pos_x;
		y_a = dev_cmd->fill_rect.pos_y;
		width = dev_cmd->fill_rect.width;
		height = dev_cmd->fill_rect.height;

		HARDDOOM2_ENSURE(bufs->surf_dst);
		HARDDOOM2_ENSURE(x_a + width - 1 < bufs->surf_dst->width
			&& y_a + height - 1 < bufs->surf_dst->height);

		buf_cmd->w[0] = HARDDOOM2_CMD_TYPE_FILL_RECT;
		buf_cmd->w[2] = HARDDOOM2_CMD_W2(x_a, y_a, 0);
		buf_cmd->w[6] = HARDDOOM2_CMD_W6_A(width, height, dev_cmd->fill_rect.fill_color);
		break;

	case DOOMDEV2_CMD_TYPE_DRAW_LINE:
		x_a = dev_cmd->draw_line.pos_a_x;
		y_a = dev_cmd->draw_line.pos_a_y;
		x_b = dev_cmd->draw_line.pos_b_x;
		y_b = dev_cmd->draw_line.pos_b_y;

		HARDDOOM2_ENSURE(bufs->surf_dst
			&& x_a < bufs->surf_dst->width && y_a < bufs->surf_dst->height
			&& x_b < bufs->surf_dst->width && y_b < bufs->surf_dst->height);

		buf_cmd->w[0] = HARDDOOM2_CMD_TYPE_DRAW_LINE;
		buf_cmd->w[2] = HARDDOOM2_CMD_W2(x_a, y_a, 0);
		buf_cmd->w[3] = HARDDOOM2_CMD_W3(x_b, y_b);
		buf_cmd->w[6] = HARDDOOM2_CMD_W6_A(0, 0, dev_cmd->draw_line.fill_color);
		break;

	case DOOMDEV2_CMD_TYPE_DRAW_BACKGROUND:
		x_a = dev_cmd->draw_background.pos_x;
		y_a = dev_cmd->draw_background.pos_y;
		width = dev_cmd->draw_background.width;
		height = dev_cmd->draw_background.height;
		flat_idx = dev_cmd->draw_background.flat_idx;

		HARDDOOM2_ENSURE(bufs->surf_dst && bufs->flat);
		HARDDOOM2_ENSURE(x_a + width - 1 < bufs->surf_dst->width
			&& y_a + height - 1 < bufs->surf_dst->height);
		HARDDOOM2_ENSURE(flat_idx * DOOMDEV2_BUFFER_FLAT_SIZE < bufs->flat->width);

		buf_cmd->w[0] = HARDDOOM2_CMD_TYPE_DRAW_BACKGROUND;
		buf_cmd->w[2] = HARDDOOM2_CMD_W2(x_a, y_a, flat_idx);
		buf_cmd->w[6] = HARDDOOM2_CMD_W6_A(width, height, 0);
		break;

	case DOOMDEV2_CMD_TYPE_DRAW_COLUMN:
		x_a = x_b = dev_cmd->draw_column.pos_x;
		y_a = dev_cmd->draw_column.pos_a_y;
		y_b = dev_cmd->draw_column.pos_b_y;
		flags = 0;
		translation_idx = colormap_idx = 0;

		HARDDOOM2_ENSURE(bufs->surf_dst && bufs->texture);
		HARDDOOM2_ENSURE(x_a < bufs->surf_dst->width
			&& y_a <= y_b && y_b < bufs->surf_dst->height);

		// data after buffer not guarded by limit (<64 bytes) is initialized with zeros anyway
		texture_limit = (bufs->texture->width - 1) / HARDDOOM2_TEXTURE_LIMIT_UNITS;

		if (dev_cmd->draw_column.flags & DOOMDEV2_CMD_FLAGS_TRANSLATE) {
			flags |= HARDDOOM2_CMD_FLAG_TRANSLATION;
			translation_idx = dev_cmd->draw_column.translation_idx;
			HARDDOOM2_ENSURE(bufs->translation
				&& translation_idx * DOOMDEV2_BUFFER_COLORMAP_SIZE < bufs->translation->width);
		}
		if (dev_cmd->draw_column.flags & DOOMDEV2_CMD_FLAGS_COLORMAP) {
			flags |= HARDDOOM2_CMD_FLAG_COLORMAP;
			colormap_idx = dev_cmd->draw_column.colormap_idx;
			HARDDOOM2_ENSURE(bufs->colormap
				&& colormap_idx * DOOMDEV2_BUFFER_COLORMAP_SIZE < bufs->colormap->width);
		}
		if (dev_cmd->draw_column.flags & DOOMDEV2_CMD_FLAGS_TRANMAP) {
			flags |= HARDDOOM2_CMD_FLAG_TRANMAP;
			HARDDOOM2_ENSURE(bufs->tranmap);
		}

		buf_cmd->w[0] = DOOMDEV2_CMD_TYPE_DRAW_COLUMN | flags;
		buf_cmd->w[1] = HARDDOOM2_CMD_W1(translation_idx, colormap_idx);
		buf_cmd->w[2] = HARDDOOM2_CMD_W2(x_a, y_a, 0);
		buf_cmd->w[3] = HARDDOOM2_CMD_W3(x_b, y_b);
		buf_cmd->w[4] = dev_cmd->draw_column.ustart;
		buf_cmd->w[5] = dev_cmd->draw_column.ustep;
		buf_cmd->w[6] = HARDDOOM2_CMD_W6_B_EXTR_TEXTURE_OFFSET(dev_cmd->draw_column.texture_offset);
		buf_cmd->w[7] = HARDDOOM2_CMD_W7_B(texture_limit, dev_cmd->draw_column.texture_height);
		break;

	case DOOMDEV2_CMD_TYPE_DRAW_SPAN:
		y_a = y_b = dev_cmd->draw_span.pos_y;
		x_a = dev_cmd->draw_span.pos_a_x;
		x_b = dev_cmd->draw_span.pos_b_x;
		flat_idx = dev_cmd->draw_span.flat_idx;
		flags = 0;
		translation_idx = colormap_idx = 0;

		HARDDOOM2_ENSURE(bufs->surf_dst && bufs->flat);
		HARDDOOM2_ENSURE(x_a <= x_b && x_b < bufs->surf_dst->width
			&& y_a < bufs->surf_dst->height);
		HARDDOOM2_ENSURE(flat_idx * DOOMDEV2_BUFFER_FLAT_SIZE < bufs->flat->width);

		if (dev_cmd->draw_span.flags & DOOMDEV2_CMD_FLAGS_TRANSLATE) {
			flags |= HARDDOOM2_CMD_FLAG_TRANSLATION;
			translation_idx = dev_cmd->draw_span.translation_idx;
			HARDDOOM2_ENSURE(bufs->translation
				&& translation_idx * DOOMDEV2_BUFFER_COLORMAP_SIZE < bufs->translation->width);
		}
		if (dev_cmd->draw_span.flags & DOOMDEV2_CMD_FLAGS_COLORMAP) {
			flags |= HARDDOOM2_CMD_FLAG_COLORMAP;
			colormap_idx = dev_cmd->draw_span.colormap_idx;
			HARDDOOM2_ENSURE(bufs->colormap
				&& colormap_idx * DOOMDEV2_BUFFER_COLORMAP_SIZE < bufs->colormap->width);
		}
		if (dev_cmd->draw_span.flags & DOOMDEV2_CMD_FLAGS_TRANMAP) {
			flags |= HARDDOOM2_CMD_FLAG_TRANMAP;
			HARDDOOM2_ENSURE(bufs->tranmap);
		}

		buf_cmd->w[0] = HARDDOOM2_CMD_TYPE_DRAW_SPAN | flags;
		buf_cmd->w[1] = HARDDOOM2_CMD_W1(translation_idx, colormap_idx);
		buf_cmd->w[2] = HARDDOOM2_CMD_W2(x_a, y_a, flat_idx);
		buf_cmd->w[3] = HARDDOOM2_CMD_W3(x_b, y_b);
		buf_cmd->w[4] = dev_cmd->draw_span.ustart;
		buf_cmd->w[5] = dev_cmd->draw_span.ustep;
		buf_cmd->w[6] = dev_cmd->draw_span.vstart;
		buf_cmd->w[7] = dev_cmd->draw_span.vstep;
		break;

	case DOOMDEV2_CMD_TYPE_DRAW_FUZZ:
		x_a = x_b = dev_cmd->draw_fuzz.pos_x;
		y_a = dev_cmd->draw_fuzz.pos_a_y;
		y_b = dev_cmd->draw_fuzz.pos_b_y;
		colormap_idx = dev_cmd->draw_fuzz.colormap_idx;
		fuzz_start = dev_cmd->draw_fuzz.fuzz_start;
		fuzz_end = dev_cmd->draw_fuzz.fuzz_end;
		fuzz_pos = dev_cmd->draw_fuzz.fuzz_pos;
		flags = HARDDOOM2_CMD_FLAG_COLORMAP;

		HARDDOOM2_ENSURE(bufs->surf_dst && bufs->colormap);
		HARDDOOM2_ENSURE(x_a < bufs->surf_dst->width
			&& fuzz_start <= y_a && y_a <= y_b && y_b <= fuzz_end && fuzz_end < bufs->surf_dst->height);
		HARDDOOM2_ENSURE(fuzz_pos <= HARDDOOM2_FUZZ_POS_MAX);
		HARDDOOM2_ENSURE(colormap_idx * DOOMDEV2_BUFFER_COLORMAP_SIZE < bufs->colormap->width);

		buf_cmd->w[0] = HARDDOOM2_CMD_TYPE_DRAW_FUZZ | flags;
		buf_cmd->w[1] = HARDDOOM2_CMD_W1(0, colormap_idx);
		buf_cmd->w[2] = HARDDOOM2_CMD_W2(x_a, y_a, 0);
		buf_cmd->w[3] = HARDDOOM2_CMD_W3(x_b, y_b);
		buf_cmd->w[6] = HARDDOOM2_CMD_W6_C(fuzz_start, fuzz_end, fuzz_pos);
		break;
	}

#undef HARDDOOM2_ENSURE

	return 0;
}

static void harddoom2_enqueue_cmd(struct harddoom2_device *hdev, struct harddoom2_cmd_buf *buf) {
	// find free slot
	uint32_t bytes_pos = hdev->cmd_write_idx * sizeof(*buf);
	uint32_t page_idx = bytes_pos / HARDDOOM2_PAGE_SIZE + 1; // +1 because of page table
	uint32_t page_offset = bytes_pos % HARDDOOM2_PAGE_SIZE;
	uint32_t next_write_idx = (hdev->cmd_write_idx + 1) % HARDDOOM2_MAX_CMD_CNT;

	// wait for space in queue, if there's none
	if (next_write_idx == hdev->cmd_read_idx_cached) {
		iowrite32(HARDDOOM2_INTR_PONG_ASYNC, hdev->bar0 + HARDDOOM2_INTR);
		hdev->cmd_read_idx_cached = ioread32(hdev->bar0 + HARDDOOM2_CMD_READ_IDX);
		if (next_write_idx == hdev->cmd_read_idx_cached) {
			uint32_t intr_flags = ioread32(hdev->bar0 + HARDDOOM2_INTR_ENABLE);
			iowrite32(intr_flags | HARDDOOM2_INTR_PONG_ASYNC, hdev->bar0 + HARDDOOM2_INTR_ENABLE);
			iowrite32(hdev->cmd_write_idx, hdev->bar0 + HARDDOOM2_CMD_WRITE_IDX);
			wait_for_completion(&hdev->pong_async);
			iowrite32(intr_flags & ~HARDDOOM2_INTR_PONG_ASYNC, hdev->bar0 + HARDDOOM2_INTR_ENABLE);
			hdev->cmd_read_idx_cached = ioread32(hdev->bar0 + HARDDOOM2_CMD_READ_IDX);
		}
	}

	// add async flag, if it's the time:
	if (hdev->cmd_write_idx % HARDDOOM2_ASYNC_CMD_INTERVAL == 0) {
		buf->w[0] |= HARDDOOM2_CMD_FLAG_PING_ASYNC;
	}

	// add to queue, assumption: mutex to queue is locked
	memcpy(hdev->cmd_queue->pages[page_idx].phys_addr + page_offset, buf, sizeof(*buf));
	hdev->cmd_write_idx = next_write_idx;

	// note: you need to update write idx on harddoom device yourself, this function don't have to do this
}

static ssize_t doomdev2_write(struct file *filep, const char __user *user_buff, size_t count, loff_t *offp) {
	// todo (optimization) preprocess commands, then lock the mutex, change context, and send commands
	struct doomdev2_ctx *ctx = (struct doomdev2_ctx*) filep->private_data;
	struct harddoom2_cmd_buf cmd_buf;
	struct doomdev2_cmd dev_cmd;
	uint32_t flags, sdwidth, sswidth;
	ssize_t ret;

	if (*offp != 0 || count % sizeof(struct doomdev2_cmd) != 0) {
		return -EINVAL;
	}

	if (count == 0) {
		return 0;
	}

	ret = mutex_lock_interruptible(&ctx->hdev->sync_queue_mutex);
	if (ret) {
		return ret;
	}
	if (!ctx->hdev->ready) {
		ret = -EAGAIN;
		goto exit_op;
	}

	// context switch
	ret = mutex_lock_interruptible(&ctx->active_bufs_mutex);
	if (ret) {
		goto exit_op;
	}

#define HARDDOOM2_SETUP_HELPER(buf_name, buf_flag, buf_w_idx, append_code) do { \
		if (ctx->active_bufs.buf_name && ctx->active_bufs.buf_name != ctx->hdev->last_active_bufs.buf_name) { \
			ctx->hdev->last_active_bufs.buf_name = ctx->active_bufs.buf_name; \
			flags |= (buf_flag); \
			cmd_buf.w[(buf_w_idx)] = ctx->active_bufs.buf_name->pages[0].dma_handle >> HARDDOOM2_CMD_SETUP_PT_SHIFT; \
			append_code; \
		} \
	} while(0)

	memset(&cmd_buf, 0, sizeof(cmd_buf));
	sdwidth = sswidth = 0;
	flags = 0;
	HARDDOOM2_SETUP_HELPER(surf_dst, HARDDOOM2_CMD_FLAG_SETUP_SURF_DST, HARDDOOM2_CMD_SETUP_SURF_DST_PT_IDX,
		{ sdwidth = ctx->active_bufs.surf_dst->width; });
	HARDDOOM2_SETUP_HELPER(surf_src, HARDDOOM2_CMD_FLAG_SETUP_SURF_SRC, HARDDOOM2_CMD_SETUP_SURF_SRC_PT_IDX,
		{ sswidth = ctx->active_bufs.surf_src->width; });
	HARDDOOM2_SETUP_HELPER(texture,     HARDDOOM2_CMD_FLAG_SETUP_TEXTURE,     HARDDOOM2_CMD_SETUP_TEXTURE_PT_IDX, );
	HARDDOOM2_SETUP_HELPER(flat,        HARDDOOM2_CMD_FLAG_SETUP_FLAT,        HARDDOOM2_CMD_SETUP_FLAT_PT_IDX, );
	HARDDOOM2_SETUP_HELPER(translation, HARDDOOM2_CMD_FLAG_SETUP_TRANSLATION, HARDDOOM2_CMD_SETUP_TRANSLATION_PT_IDX, );
	HARDDOOM2_SETUP_HELPER(colormap,    HARDDOOM2_CMD_FLAG_SETUP_COLORMAP,    HARDDOOM2_CMD_SETUP_COLORMAP_PT_IDX, );
	HARDDOOM2_SETUP_HELPER(tranmap,     HARDDOOM2_CMD_FLAG_SETUP_TRANMAP,     HARDDOOM2_CMD_SETUP_TRANMAP_PT_IDX, );
	cmd_buf.w[0] = HARDDOOM2_CMD_W0_SETUP(HARDDOOM2_CMD_TYPE_SETUP, flags, sdwidth, sswidth);
#undef HARDDOOM2_SETUP_HELPER

	if (flags) {
		harddoom2_enqueue_cmd(ctx->hdev, &cmd_buf);
	}

	// process command by command
	ret = 0;

	while (ret < count) {
		int err = copy_from_user(&dev_cmd, user_buff + ret, sizeof(dev_cmd));
		if (err) {
			if (ret == 0) {
				ret = -EFAULT;
			}
			break;
		}

		err = doomdev2_prepare_harddoom2_cmd(ctx, &cmd_buf, &dev_cmd);
		if (err) {
			dev_info(&ctx->hdev->pdev->dev, "Got invalid command, cmd_type = 0x%x\n", dev_cmd.type); // todo remove
			if (ret == 0) {
				ret = err;
			}
			break;
		}
		harddoom2_enqueue_cmd(ctx->hdev, &cmd_buf);
		ret += sizeof(dev_cmd);
	}

	// we need to access active buffers while validating commands
	// todo (optimization) -- make a copy, and remember about ref counts!
	mutex_unlock(&ctx->active_bufs_mutex);


	if (ret > 0) {
		// wait until commands are processed
		memset(&cmd_buf, 0, sizeof(cmd_buf));
		cmd_buf.w[0] = HARDDOOM2_CMD_W0_SETUP(HARDDOOM2_CMD_TYPE_SETUP, HARDDOOM2_CMD_FLAG_PING_SYNC, 0, 0); // todo change to fence in async code
		harddoom2_enqueue_cmd(ctx->hdev, &cmd_buf);
		iowrite32(ctx->hdev->cmd_write_idx, ctx->hdev->bar0 + HARDDOOM2_CMD_WRITE_IDX);
		wait_for_completion(&ctx->hdev->pong_sync);
	}
	else {
		printk(KERN_DEBUG "Some error occured, ret=%ld\n", ret); // todo remove (?)
	}


exit_op:
	mutex_unlock(&ctx->hdev->sync_queue_mutex);

	return ret;
}

static struct file_operations doomdev2_fops = {
	.owner = THIS_MODULE,
	.open = doomdev2_open,
	.release = doomdev2_release,
	.unlocked_ioctl = doomdev2_ioctl,
	.compat_ioctl = doomdev2_ioctl,
	.llseek = no_llseek,
	.write = doomdev2_write,
};


static dev_t doomdev2_alloc_minor(void) {
	dev_t ret = DOOMDEV2_NO_AVAILABLE_MINOR;
	int minor;

	mutex_lock(&doomdev2_used_minors_mutex); // not called by user, no need for _interruptible
	minor = find_next_zero_bit(doomdev2_used_minors, HARDDOOM2_MAX_DEVICES, 0);
	if (minor < HARDDOOM2_MAX_DEVICES) { // success
		bitmap_set(doomdev2_used_minors, minor, 1);
		ret = doomdev2_major + minor;
	}
	mutex_unlock(&doomdev2_used_minors_mutex);

	return ret;
}

static void doomdev2_dealloc_minor(dev_t minor) {
	mutex_lock(&doomdev2_used_minors_mutex);  // better not to fail in destructor & not called by user
	bitmap_clear(doomdev2_used_minors, minor - doomdev2_major, 1);
	mutex_unlock(&doomdev2_used_minors_mutex);
}


static void harddoom2_enable(struct harddoom2_device *hdev) {
	uint32_t code_size, it;

	// assertions
	BUG_ON(hdev->cmd_write_idx != 0 || hdev->cmd_read_idx_cached != 0);
	for (it = 0; it < sizeof(hdev->last_active_bufs); it++) {
		if (((uint8_t*) &hdev->last_active_bufs)[it]) {
			BUG();
		}
	}

	// boot sequence
	iowrite32(0, hdev->bar0 + HARDDOOM2_FE_CODE_ADDR);
	code_size = ARRAY_SIZE(doomcode2);
	for (it = 0; it < code_size; it++) {
		iowrite32(doomcode2[it], hdev->bar0 + HARDDOOM2_FE_CODE_WINDOW);
	}
	iowrite32(HARDDOOM2_RESET_ALL, hdev->bar0 + HARDDOOM2_RESET);
	iowrite32(hdev->cmd_queue->pages[0].dma_handle >> HARDDOOM2_CMD_SETUP_PT_SHIFT, hdev->bar0 + HARDDOOM2_CMD_PT);
	iowrite32(HARDDOOM2_MAX_CMD_CNT, hdev->bar0 + HARDDOOM2_CMD_SIZE);
	iowrite32(hdev->cmd_read_idx_cached, hdev->bar0 + HARDDOOM2_CMD_READ_IDX);
	iowrite32(hdev->cmd_write_idx, hdev->bar0 + HARDDOOM2_CMD_WRITE_IDX);
	iowrite32(HARDDOOM2_INTR_MASK, hdev->bar0 + HARDDOOM2_INTR);
	iowrite32(HARDDOOM2_INTR_MASK & ~HARDDOOM2_INTR_PONG_ASYNC, hdev->bar0 + HARDDOOM2_INTR_ENABLE);
	// todo init FENCE_COUNTER
	iowrite32(HARDDOOM2_ENABLE_ALL, hdev->bar0 + HARDDOOM2_ENABLE);
}

static void harddoom2_disable(struct harddoom2_device *hdev) {
	iowrite32(0, hdev->bar0 + HARDDOOM2_ENABLE);
	iowrite32(0, hdev->bar0 + HARDDOOM2_INTR_ENABLE);
	ioread32(hdev->bar0 + HARDDOOM2_ENABLE);
}

static int harddoom2_probe(struct pci_dev *pdev, const struct pci_device_id *_id) {
	int ret = 0;
	int minor_zero_based;
	bool region_request_succeeded = false;
	dev_t minor;
	void __iomem *bar0;
	struct harddoom2_device *hdev;
	struct device *sysfs_dev;

	// enable device, map memory
	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Cannot enable PCI device: %d", ret);
		return ret;
	}

	ret = pci_request_regions(pdev, harddoom2_driver.name);
	if (ret) {
		dev_err(&pdev->dev, "Cannot request PCI regions: %d", ret);
		goto err_disable_pci_device;
	}
	region_request_succeeded = true;

	bar0 = pci_iomap(pdev, 0, 0);
	if (!bar0) {
		dev_err(&pdev->dev, "Failed: pci_iomap");
		ret = -ENOMEM;
		goto err_disable_pci_device;
	}

	// alloc drvdata
	hdev = (struct harddoom2_device*) kmalloc(sizeof(struct harddoom2_device), GFP_KERNEL);
	if (!hdev) {
		dev_err(&pdev->dev, "Failed: kmalloc for driver data");
		ret = -ENOMEM;
		goto err_unmap_pci_io;
	}
	hdev->pdev = pdev;
	hdev->bar0 = bar0;
	hdev->ready = true;
	hdev->cmd_queue = NULL; // init below
	hdev->cmd_write_idx = hdev->cmd_read_idx_cached = 0;
	init_completion(&hdev->pong_async);
	memset(&hdev->last_active_bufs, 0, sizeof(hdev->last_active_bufs));
	mutex_init(&hdev->sync_queue_mutex);
	init_completion(&hdev->pong_sync);
	pci_set_drvdata(pdev, hdev);

	// set up dma
	pci_set_master(pdev);
	ret = pci_set_dma_mask(pdev, HARDDOOM2_DMA_MASK);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set PCI DMA mask");
		goto err_pci_clear_master;
	}
	ret = pci_set_consistent_dma_mask(pdev, HARDDOOM2_DMA_MASK);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set consistent PCI DMA mask");
		goto err_pci_clear_master;
	}

	// allocate cmd queue buffer
	hdev->cmd_queue = doomdev2_dma_buffer_alloc(hdev, DOOMDEV2_MAX_BUFFER_SIZE / HARDDOOM2_PAGE_SIZE + 1); // +1 for page table
	if (!hdev->cmd_queue) {
		dev_err(&pdev->dev, "Failed: allocation of dma buffer for command queue");
		ret = -ENOMEM;
		goto err_pci_clear_master;
	}
	doomdev2_dma_buffer_format_page_table(hdev->cmd_queue, false);

	// register interruption handler
	ret = request_irq(pdev->irq, harddoom2_irq_handler, IRQF_SHARED, harddoom2_name, hdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed: request_irq");
		goto err_del_cmd_queue;
	}

	// boot sequence
	harddoom2_enable(hdev);

	// prepare chrdev (after booting device)
	cdev_init(&hdev->doomdev2, &doomdev2_fops);
	minor = doomdev2_alloc_minor();
	minor_zero_based = minor - doomdev2_major;
	if (minor == DOOMDEV2_NO_AVAILABLE_MINOR) {
		dev_err(&pdev->dev, "Failed: no available minor to register new device");
		ret = -ENOSPC;
		goto err_disable_harddom2;
	}
	ret = cdev_add(&hdev->doomdev2, minor, 1);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add cdev, minor %d", minor_zero_based);
		goto err_dealloc_doomdev2_minor;
	}
	sysfs_dev = device_create(&doomdev2_class, &pdev->dev, minor, NULL, "doom%d", minor_zero_based);
	if (IS_ERR(sysfs_dev)) {
		dev_err(&pdev->dev, "Failed to create sysfs device, minor %d", minor_zero_based);
		ret = PTR_ERR(sysfs_dev);
		goto err_del_cdev;
	}

	return ret;

err_del_cdev:
	cdev_del(&hdev->doomdev2);
err_dealloc_doomdev2_minor:
	doomdev2_dealloc_minor(minor);
err_disable_harddom2:
	harddoom2_disable(hdev);
	free_irq(pdev->irq, hdev);
err_del_cmd_queue:
	doomdev2_dma_buffer_del(hdev->cmd_queue);
err_pci_clear_master:
	pci_clear_master(pdev);
	kfree(hdev);
err_unmap_pci_io:
	pci_iounmap(pdev, bar0);
err_disable_pci_device:
	pci_disable_device(pdev);
	if (region_request_succeeded) {
		pci_release_regions(pdev); // should be called after pci_disable_device()
	}

	return ret;
}

static void harddoom2_remove(struct pci_dev *pdev) {
	struct harddoom2_device *hdev = (struct harddoom2_device*) pci_get_drvdata(pdev);

	// "unboot" device
	harddoom2_disable(hdev);
	free_irq(hdev->pdev->irq, hdev);

	// destroy chrdev
	device_destroy(&doomdev2_class, hdev->doomdev2.dev);
	cdev_del(&hdev->doomdev2);
	doomdev2_dealloc_minor(hdev->doomdev2.dev);

	// dealloc cmd queue
	doomdev2_dma_buffer_del(hdev->cmd_queue);

	// release pci device
	pci_clear_master(pdev);
	pci_iounmap(pdev, hdev->bar0);
	pci_disable_device(pdev);
	pci_release_regions(pdev); // should be called after pci_disable_device()
	kfree(hdev);
}

static int harddoom2_suspend(struct pci_dev *dev, pm_message_t state) {
	struct harddoom2_device *hdev = (struct harddoom2_device*) pci_get_drvdata(dev);
	struct harddoom2_cmd_buf cmd_buf;
	int ret = 0;

	if (state.event != PM_EVENT_SUSPEND) {
		dev_warn(&dev->dev, "Warning: suspending, got state 0x%x, expected 0x%x", state.event, PM_EVENT_SUSPEND);
	}

	// don't allow to put more commands
	ret = mutex_lock_interruptible(&hdev->sync_queue_mutex);
	if (ret) {
		return ret;
	}
	BUG_ON(!hdev->ready);

	// wait until all commands are executed
	memset(&cmd_buf, 0, sizeof(cmd_buf));
	cmd_buf.w[0] = HARDDOOM2_CMD_W0_SETUP(HARDDOOM2_CMD_TYPE_SETUP, HARDDOOM2_CMD_FLAG_PING_SYNC, 0, 0);
	harddoom2_enqueue_cmd(hdev, &cmd_buf);
	iowrite32(hdev->cmd_write_idx, hdev->bar0 + HARDDOOM2_CMD_WRITE_IDX);
	wait_for_completion(&hdev->pong_sync);

	// mark flag device not available (todo handle it in other places!)
	hdev->ready = false;

	// "save" device state
	hdev->cmd_write_idx = hdev->cmd_read_idx_cached = 0;
	memset(&hdev->last_active_bufs, 0, sizeof(hdev->last_active_bufs));

	harddoom2_disable(hdev);

	mutex_unlock(&hdev->sync_queue_mutex);

	return ret;
}

static int harddoom2_resume(struct pci_dev *dev) {
	struct harddoom2_device *hdev = (struct harddoom2_device*) pci_get_drvdata(dev);
	int ret = 0;

	ret = mutex_lock_interruptible(&hdev->sync_queue_mutex); // should it be interruptable?
	if (ret) {
		return ret;
	}
	BUG_ON(hdev->ready);

	hdev->ready = true;
	harddoom2_enable(hdev);

	mutex_unlock(&hdev->sync_queue_mutex);

	return ret;
}

static const struct pci_device_id harddoom2_pci_tbl[] = {
	{ PCI_DEVICE(HARDDOOM2_VENDOR_ID, HARDDOOM2_DEVICE_ID) },
	{ 0, },
};

static struct pci_driver harddoom2_driver = {
    .name = harddoom2_name,
    .id_table = harddoom2_pci_tbl,
	.probe = harddoom2_probe,
	.remove = harddoom2_remove,
	.suspend = harddoom2_suspend,
	.resume = harddoom2_resume,
};

static int harddoom2_init(void) {
	int ret = 0;
	
	ret = alloc_chrdev_region(&doomdev2_major, 0, HARDDOOM2_MAX_DEVICES, harddoom2_name);
	if (ret) {
		return ret;
	}

	ret = class_register(&doomdev2_class);
	if (ret) {
		goto err_0;
	}

	ret = pci_register_driver(&harddoom2_driver);
	if (ret) {
		goto err_1;
	}

	return ret;

err_1:
	class_unregister(&doomdev2_class);
err_0:
	unregister_chrdev_region(doomdev2_major, HARDDOOM2_MAX_DEVICES);
	return ret;
}

static void harddoom2_cleanup(void) {
	pci_unregister_driver(&harddoom2_driver);
	class_unregister(&doomdev2_class);
	unregister_chrdev_region(doomdev2_major, HARDDOOM2_MAX_DEVICES);
}

module_init(harddoom2_init);
module_exit(harddoom2_cleanup);
