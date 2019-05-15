// TODO remove unused headers
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
// todo "i pilnować wysyłanych poleceń, czyli:
	// weryfikować, że wysyłane współrzędne y mieszczą się w wysokości buforów ramek, (x też warto!)
	// limitować odczyt tekseli z tekstur kolumnowych przez użycie TEXTURE_LIMIT,
	// weryfikować, że FLAT_IDX, COLORMAP_IDX, TRANSLATION_IDX mieszczą się w rozmiarze odpowiednich atlasów."

static const char harddoom2_name[] = { "harddoom2" };
static struct pci_driver harddoom2_driver;
static dev_t doomdev2_major;
static struct class doomdev2_class = {
	.name = harddoom2_name,
	.owner = THIS_MODULE,
};
static DECLARE_BITMAP(doomdev2_used_minors, HARDDOOM2_MAX_DEVICES);
static DEFINE_MUTEX(doomdev2_used_minors_mutex);
static volatile uint32_t harddoom2_intr_error_flags;


struct doomdev2_ctx_buffers {
	// struct doomdev2_dma_buffer *cmd;
	struct doomdev2_dma_buffer *surf_dst;
	struct doomdev2_dma_buffer *surf_src;
	struct doomdev2_dma_buffer *texture;
	struct doomdev2_dma_buffer *flat;
	struct doomdev2_dma_buffer *colormap;
	struct doomdev2_dma_buffer *translation;
	struct doomdev2_dma_buffer *tranmap;
};

struct harddoom2_pcidevdata {
	struct pci_dev *pdev;
	void __iomem *bar0;
	struct cdev doomdev2;

	struct doomdev2_ctx_buffers last_active_bufs; // note: NEVER deref pointers inside (we hold weak reference here)
	struct mutex sync_queue_mutex; // used for sending to queue and operating on last_active_bufs member
	struct completion pong_sync;
};

struct doomdev2_ctx {
	struct harddoom2_pcidevdata *pddata;
	struct list_head buffers;
	struct mutex buffers_mutex; // todo some RW lock instead?
	struct kref kref;

	struct doomdev2_ctx_buffers active_bufs;
	struct mutex active_bufs_mutex;
};

struct doomdev2_dma_page {
	void *phys_addr;
	dma_addr_t dma_handle;
};

// represents surfaces and buffers
struct doomdev2_dma_buffer {
	struct doomdev2_ctx *ctx;
	int32_t width;
	int32_t height; // if zero, then it's a buffer, not a surface
	int32_t fd;
	struct file *file;
	struct list_head list;

	size_t pages_cnt;
	struct doomdev2_dma_page pages[0];
};
#define DOOMDEV2_DMA_BUFFER_SIZE(dma_buf) \
	((dma_buf)->width * max(1, (dma_buf)->height))

struct harddoom2_cmd_buf {
	uint32_t w[HARDDOOM2_CMD_SEND_SIZE];
};

static irqreturn_t harddoom2_irq_handler(int _irq, void *dev) {
	struct harddoom2_pcidevdata *pddata = (struct harddoom2_pcidevdata*) dev;
	uint32_t flags, error_flags;

	// read interruptions
	flags = ioread32(pddata->bar0 + HARDDOOM2_INTR);
	if (!flags) {
		return IRQ_NONE;
	}

	// handle interruptions
	if (flags & HARDDOOM2_INTR_FENCE) {
		// todo
	}

	if (flags & HARDDOOM2_INTR_PONG_SYNC) {
		complete(&pddata->pong_sync);
	}

	if (flags & HARDDOOM2_INTR_PONG_ASYNC) {
		// todo
	}

	error_flags = flags & HARDDOOM2_INTR_ERRORS_MASK;
	// __atomic_store(&harddoom2_intr_error_flags, &error_flags, __ATOMIC_SEQ_CST);
	harddoom2_intr_error_flags |= error_flags; // todo make this op atomic

	// disable interruptions (mark as done)
	iowrite32(flags, pddata->bar0 + HARDDOOM2_INTR);

	return IRQ_HANDLED;
}

static void doomdev2_ctx_deallocate(struct kref *kref) {
	struct doomdev2_ctx *ctx = container_of(kref, struct doomdev2_ctx, kref);
	kfree(ctx);
}

static int doomdev2_ctx_register_buffer(struct doomdev2_ctx *ctx, struct doomdev2_dma_buffer *dma_buf) {
	int ret = 0;

	ret = mutex_lock_interruptible(&ctx->buffers_mutex);
	if (ret) {
		return ret;
	}
	list_add_tail(&dma_buf->list, &ctx->buffers);
	mutex_unlock(&ctx->buffers_mutex);

	kref_get(&ctx->kref);

	return ret;
}

static void doomdev2_ctx_unregister_buffer(struct doomdev2_dma_buffer *dma_buf) {
	mutex_lock(&dma_buf->ctx->buffers_mutex); // better not to fail in destructor
	list_del(&dma_buf->list);
	mutex_unlock(&dma_buf->ctx->buffers_mutex);

	kref_put(&dma_buf->ctx->kref, doomdev2_ctx_deallocate);
}

static struct doomdev2_dma_buffer *doomdev2_dma_alloc_buffer(struct doomdev2_ctx *ctx, size_t pages_cnt) {
	struct doomdev2_dma_buffer *dma_buf;
	size_t alloc_size;
	struct doomdev2_dma_page *page_it, *pages_end;

	// alloc describing structure
	alloc_size = sizeof(struct doomdev2_dma_buffer) + sizeof(struct doomdev2_dma_page) * pages_cnt;
	dma_buf = (struct doomdev2_dma_buffer*) kmalloc(alloc_size, GFP_KERNEL);
	if (!dma_buf) {
		return NULL;
	}
	dma_buf->ctx = ctx;
	dma_buf->width = DOOMDEV2_DMA_BUFFER_NO_VALUE;   // set later in other function
	dma_buf->height = DOOMDEV2_DMA_BUFFER_NO_VALUE;  // ^
	dma_buf->fd = DOOMDEV2_DMA_BUFFER_NO_VALUE;      // ^
	dma_buf->file = NULL;
	INIT_LIST_HEAD(&dma_buf->list);
	dma_buf->pages_cnt = pages_cnt;

	// alloc pages
	pages_end = dma_buf->pages + pages_cnt;
	for (page_it = dma_buf->pages; page_it < pages_end; page_it++) {
		page_it->phys_addr = dma_alloc_coherent(&ctx->pddata->pdev->dev, HARDDOOM2_PAGE_SIZE,
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
		dma_free_coherent(&ctx->pddata->pdev->dev, HARDDOOM2_PAGE_SIZE, page_it->phys_addr, page_it->dma_handle);
	}
	kfree(dma_buf);

	return NULL;
}

static void doomdev2_dma_dealloc_buffer(struct doomdev2_dma_buffer *dma_buf) {
	struct doomdev2_dma_page *page_it, *pages_end;

	pages_end = dma_buf->pages + dma_buf->pages_cnt;
	for (page_it = dma_buf->pages; page_it < pages_end; page_it++) {
		dma_free_coherent(&dma_buf->ctx->pddata->pdev->dev, HARDDOOM2_PAGE_SIZE, page_it->phys_addr, page_it->dma_handle);
	}

	if (dma_buf->fd != DOOMDEV2_DMA_BUFFER_NO_VALUE) { // todo how about cmd buffer?
		doomdev2_ctx_unregister_buffer(dma_buf);
	}

	kfree(dma_buf);
}

static int doomdev2_dma_buffer_release(struct inode *_ino, struct file *filep) {
	struct doomdev2_dma_buffer *dma_buf = (struct doomdev2_dma_buffer*) filep->private_data;

	doomdev2_dma_dealloc_buffer(dma_buf);

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

	ret = mutex_lock_interruptible(&dma_buf->ctx->pddata->sync_queue_mutex);
	if (ret) {
		return ret;
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
	mutex_unlock(&dma_buf->ctx->pddata->sync_queue_mutex);

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

static int doomdev2_dma_alloc_install_fd(struct doomdev2_dma_buffer *dma_buf) {
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

	dma_buf->fd = fd;
	dma_buf->file = f;
	f->f_mode |= FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE;

	err = doomdev2_ctx_register_buffer(dma_buf->ctx, dma_buf);
	if (err) {
		goto err_put_unused_fd;
	}

	fd_install(fd, f);

	return fd;

err_put_unused_fd:
	put_unused_fd(fd);

	return err;
}

static void doomdev2_dma_format_page_table(struct doomdev2_dma_buffer *dma_buf, bool writable) {
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

static long doomdev2_dma_buffer_create(struct doomdev2_ctx *ctx, size_t dma_alloc_size, int32_t width, int32_t height) {
	int err;
	size_t dma_pages_cnt;
	struct doomdev2_dma_buffer *dma_buf;

	dma_pages_cnt = DIV_ROUND_UP(dma_alloc_size, HARDDOOM2_PAGE_SIZE) + 1; // +1 for page table

	dma_buf = doomdev2_dma_alloc_buffer(ctx, dma_pages_cnt);
	if (!dma_buf) {
		return -ENOMEM;
	}

	dma_buf->width = width;
	dma_buf->height = height;
	doomdev2_dma_format_page_table(dma_buf, true);

	err = doomdev2_dma_alloc_install_fd(dma_buf); // must be called last, since it installs fd!
	if (err < 0) {
		goto err_fd_alloc;
	}

	return err; // return fd

err_fd_alloc:
	doomdev2_dma_dealloc_buffer(dma_buf);

	return err;
}

static long doomdev2_ioctl_create_surface(struct doomdev2_ctx *ctx, struct doomdev2_ioctl_create_surface *data_cs) {
	size_t dma_alloc_size;

	if (data_cs->width < DOOMDEV2_MIN_SURFACE_SIZE || data_cs->width > DOOMDEV2_MAX_SURFACE_SIZE
			|| data_cs->height < DOOMDEV2_MIN_SURFACE_SIZE || data_cs->height > DOOMDEV2_MAX_SURFACE_SIZE) {
		return -EOVERFLOW;
	}
	if (data_cs->width % DOOMDEV2_SURFACE_WIDTH_DIVIDER != 0) {
		return -EINVAL;
	}
	dma_alloc_size = data_cs->width * data_cs->height;

	return doomdev2_dma_buffer_create(ctx, dma_alloc_size, data_cs->width, data_cs->height);
}

static long doomdev2_ioctl_create_buffer(struct doomdev2_ctx *ctx, struct doomdev2_ioctl_create_buffer *data_cb) {
	if (data_cb->size < DOOMDEV2_MIN_BUFFER_SIZE || data_cb->size > DOOMDEV2_MAX_BUFFER_SIZE) {
		return -EOVERFLOW;
	}

	return doomdev2_dma_buffer_create(ctx, data_cb->size, data_cb->size, 0);
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

	inline struct doomdev2_dma_buffer *find_fd(int32_t fd) {
		// assumption: buffers_mutex is locked
		struct doomdev2_dma_buffer *it;

		list_for_each_entry(it, &ctx->buffers, list) {
			if (it->fd == fd) {
				return it;
			}
		}

		return NULL;
	}

	inline int validate_set_buf(struct doomdev2_dma_buffer **dst, int32_t fd,
			bool is_surface, size_t size_divider, bool exact_size) {
		struct doomdev2_dma_buffer *dma_buf;

		if (fd != -1) {
			size_t size;
			dma_buf = find_fd(fd);
			if (!dma_buf) {
				return -EINVAL;
			}
			if (is_surface ? dma_buf->height <= 0 : dma_buf->height > 0) {
				return -EINVAL;
			}
			size = DOOMDEV2_DMA_BUFFER_SIZE(dma_buf);
			if (exact_size ? size != size_divider : size % size_divider != 0) {
				return -EINVAL;
			}
			// alignment is ensured by dma_alloc_coherent
		}
		else {
			dma_buf = NULL;
		}

		*dst = dma_buf;
		return 0;
	}

	ret = mutex_lock_interruptible(&ctx->buffers_mutex);
	if (ret) {
		return ret;
	}

	memset(&tmp_bufs, 0, sizeof(tmp_bufs));

	// validate fds
#define DOOMDEV2_FD_CHECK(buf_name, ...) do { \
		ret = validate_set_buf(&tmp_bufs.buf_name, data_s->buf_name##_fd, __VA_ARGS__); \
		if (ret) { \
			goto err_unlock_buffers; \
		} \
	} while (0)

	DOOMDEV2_FD_CHECK(surf_dst,    true,  1, false);
	DOOMDEV2_FD_CHECK(surf_src,    true,  1, false);
	DOOMDEV2_FD_CHECK(texture,     false, 1, false);
	DOOMDEV2_FD_CHECK(flat,        false, DOOMDEV2_BUFFER_FLAT_SIZE,     false);
	DOOMDEV2_FD_CHECK(colormap,    false, DOOMDEV2_BUFFER_COLORMAP_SIZE, false);
	DOOMDEV2_FD_CHECK(translation, false, DOOMDEV2_BUFFER_COLORMAP_SIZE, false);
	DOOMDEV2_FD_CHECK(tranmap,     false, DOOMDEV2_BUFFER_TRANMAP_SIZE, true);
#undef DOOMDEV2_FD_CHECK

	// symbolic context switch (change active buffers, not send SETUP command)
	ret = mutex_lock_interruptible(&ctx->active_bufs_mutex);
	if (ret) {
		goto err_unlock_buffers;
	}

	// increase ref count on new context
	doomdev2_ctx_buffers_ref_count_manip(&tmp_bufs, +1);

	// then decrease ref count on old context
	doomdev2_ctx_buffers_ref_count_manip(&ctx->active_bufs, -1);

	// set them as active buffers in ctx
	ctx->active_bufs = tmp_bufs;

	mutex_unlock(&ctx->active_bufs_mutex);

err_unlock_buffers:
	mutex_unlock(&ctx->buffers_mutex);

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
			ret = doomdev2_ioctl_create_surface(ctx, &data_cs);
			break;
		case DOOMDEV2_IOCTL_CREATE_BUFFER:
			ret = copy_from_user(&data_cb, arg_ptr, sizeof(data_cb));
			if (ret) {
				return -EFAULT;
			}
			ret = doomdev2_ioctl_create_buffer(ctx, &data_cb);
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

	if (IS_ERR_VALUE(ret)) {
		dev_printk("debug", &ctx->pddata->pdev->dev, "dev ioctl, cmd=%x, ret=%lx", cmd, ret);
	}

	return ret;
}

static int doomdev2_open(struct inode *ino, struct file *filep) {
	struct harddoom2_pcidevdata *pddata;
	struct doomdev2_ctx *ctx = (struct doomdev2_ctx*) kmalloc(sizeof(struct doomdev2_ctx), GFP_KERNEL);
	if (!ctx) {
		printk(KERN_ERR "harddoom2: Cannot allocate memory (doomdev2_open)\n");
		return -ENOMEM;
	}

	pddata = container_of(ino->i_cdev, struct harddoom2_pcidevdata, doomdev2);
	ctx->pddata = pddata;
	INIT_LIST_HEAD(&ctx->buffers);
	mutex_init(&ctx->buffers_mutex);
	kref_init(&ctx->kref);
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

	kref_put(&ctx->kref, doomdev2_ctx_deallocate);

	filep->private_data = NULL;

	return 0;
}

static int doomdev2_prepare_harddoom2_cmd(struct doomdev2_ctx *ctx, struct harddoom2_cmd_buf *buf_cmd,
		struct doomdev2_cmd *dev_cmd) {
	struct doomdev2_ctx_buffers *bufs = &ctx->active_bufs; // assumption: the mutex is locked
	uint16_t width, height;
	uint16_t x_a, y_a, x_b, y_b;
	uint32_t flags;
	uint16_t translation_idx, colormap_idx/*, flat_idx*/;
	uint16_t texture_limit;

	memset(buf_cmd, 0, sizeof(*buf_cmd));

#define HARDDOOM2_ENSURE(cond) if (!(cond)) { return -EINVAL; }

	switch (dev_cmd->type) {
	case DOOMDEV2_CMD_TYPE_COPY_RECT:
		break;

	case DOOMDEV2_CMD_TYPE_FILL_RECT:
		x_a = dev_cmd->fill_rect.pos_x;
		y_a = dev_cmd->fill_rect.pos_y;
		width = dev_cmd->fill_rect.width;
		height = dev_cmd->fill_rect.height;

		HARDDOOM2_ENSURE(bufs->surf_dst
			&& x_a < bufs->surf_dst->width && y_a < bufs->surf_dst->height
			&& width > 0 && height > 0);
		width = min(width, (uint16_t) (bufs->surf_dst->width - x_a));
		height = min(height, (uint16_t) (bufs->surf_dst->height - y_a));

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
		buf_cmd->w[4] = dev_cmd->draw_column.ustart; // todo validate (how?)
		buf_cmd->w[5] = dev_cmd->draw_column.ustep;  // todo validate (how?)
		buf_cmd->w[6] = HARDDOOM2_CMD_W6_B_EXTR_TEXTURE_OFFSET(dev_cmd->draw_column.texture_offset);
		buf_cmd->w[7] = HARDDOOM2_CMD_W7_B(texture_limit, dev_cmd->draw_column.texture_height);
		break;

	case DOOMDEV2_CMD_TYPE_DRAW_SPAN: // todo (!!!) debug PAGE_FAULT_TEXTURE (make page table read only, and fix page table ('twas working before refactor))
		// y_a = y_b = dev_cmd->draw_span.pos_y;
		// x_a = dev_cmd->draw_span.pos_a_x;
		// x_b = dev_cmd->draw_span.pos_b_x;
		// flat_idx = dev_cmd->draw_span.flat_idx;
		// flags = 0;
		// translation_idx = colormap_idx = 0;

		// HARDDOOM2_ENSURE(bufs->surf_dst && bufs->flat);
		// HARDDOOM2_ENSURE(x_a <= x_b && x_b < bufs->surf_dst->width
		// 	&& y_a < bufs->surf_dst->height);
		// HARDDOOM2_ENSURE(flat_idx * DOOMDEV2_BUFFER_FLAT_SIZE < bufs->flat->width);

		// if (dev_cmd->draw_span.flags & DOOMDEV2_CMD_FLAGS_TRANSLATE) {
		// 	flags |= HARDDOOM2_CMD_FLAG_TRANSLATION;
		// 	translation_idx = dev_cmd->draw_span.translation_idx;
		// 	HARDDOOM2_ENSURE(bufs->translation
		// 		&& translation_idx * DOOMDEV2_BUFFER_COLORMAP_SIZE < bufs->translation->width);
		// }
		// if (dev_cmd->draw_span.flags & DOOMDEV2_CMD_FLAGS_COLORMAP) {
		// 	flags |= HARDDOOM2_CMD_FLAG_COLORMAP;
		// 	colormap_idx = dev_cmd->draw_span.colormap_idx;
		// 	HARDDOOM2_ENSURE(bufs->colormap
		// 		&& colormap_idx * DOOMDEV2_BUFFER_COLORMAP_SIZE < bufs->colormap->width);
		// }
		// if (dev_cmd->draw_span.flags & DOOMDEV2_CMD_FLAGS_TRANMAP) {
		// 	flags |= HARDDOOM2_CMD_FLAG_TRANMAP;
		// 	HARDDOOM2_ENSURE(bufs->tranmap);
		// }

		// buf_cmd->w[0] = DOOMDEV2_CMD_TYPE_DRAW_SPAN | flags;
		// buf_cmd->w[1] = HARDDOOM2_CMD_W1(translation_idx, colormap_idx);
		// buf_cmd->w[2] = HARDDOOM2_CMD_W2(x_a, y_a, flat_idx);
		// buf_cmd->w[3] = HARDDOOM2_CMD_W3(x_b, y_b);
		// buf_cmd->w[4] = dev_cmd->draw_span.ustart; // todo validate (how?)
		// buf_cmd->w[5] = dev_cmd->draw_span.ustep;  // todo validate (how?)
		// buf_cmd->w[6] = dev_cmd->draw_span.vstart; // todo validate (how?)
		// buf_cmd->w[7] = dev_cmd->draw_span.vstep;  // todo validate (how?)
		break;

	case DOOMDEV2_CMD_TYPE_DRAW_FUZZ:
		break;
	}

#undef HARDDOOM2_ENSURE

	// todo delete later; tmp filler
	if (!buf_cmd->w[0]) {
		buf_cmd->w[0] = HARDDOOM2_CMD_TYPE_SETUP;
	}

	// todo tmp flag for sync code
	buf_cmd->w[0] |= HARDDOOM2_CMD_FLAG_INTERLOCK;

	return 0;
}

static inline void harddoom2_send_cmd(struct harddoom2_pcidevdata *pddata, struct harddoom2_cmd_buf *buf) {
	int i;

	for (i = 0; i < HARDDOOM2_CMD_SEND_SIZE; i++) {
		iowrite32(buf->w[i], pddata->bar0 + HARDDOOM2_CMD_SEND(i)); // happily assuming there's space (todo fix)
	}
}

static void harddoom2_handle_intr_errors(struct harddoom2_pcidevdata *pddata) {
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
	uint32_t flags = 0;
	int i;

	// todo make these two atomic
	flags = harddoom2_intr_error_flags;
	harddoom2_intr_error_flags = 0;

	for (i = 0; i < HARDDOOM2_INTR_ERRORS_CNT; i++) {
		if (flags & HARDDOOM2_INTR_ERROR(i)) {
			dev_err(&pddata->pdev->dev, "Error: %s\n", errors[i]);
			// flags &= ~HARDDOOM2_INTR_ERROR(i); // todo remove
		}
	}
}

static ssize_t doomdev2_write(struct file *filep, const char __user *user_buff, size_t count, loff_t *offp) {
	struct doomdev2_ctx *ctx = (struct doomdev2_ctx*) filep->private_data;
	struct harddoom2_cmd_buf cmd_buf;
	struct doomdev2_cmd dev_cmd;
	uint32_t flags, sdwidth, sswidth;
	ssize_t ret;

	harddoom2_handle_intr_errors(ctx->pddata);

	if (*offp != 0 || count % sizeof(struct doomdev2_cmd) != 0) {
		return -EINVAL;
	}

	if (count == 0) {
		return 0;
	}

	ret = mutex_lock_interruptible(&ctx->pddata->sync_queue_mutex);
	if (ret) {
		return ret;
	}

	// context switch
	ret = mutex_lock_interruptible(&ctx->active_bufs_mutex);
	if (ret) {
		goto exit_op;
	}

#define HARDDOOM2_SETUP_HELPER(buf_name, buf_flag, buf_w_idx, append_code) do { \
		if (ctx->active_bufs.buf_name && ctx->active_bufs.buf_name != ctx->pddata->last_active_bufs.buf_name) { \
			ctx->pddata->last_active_bufs.buf_name = ctx->active_bufs.buf_name; \
			flags |= (buf_flag); \
			cmd_buf.w[(buf_w_idx)] = ctx->active_bufs.buf_name->pages[0].dma_handle >> HARDDOOM2_CMD_SETUP_PT_SHIFT; \
			append_code; \
		} \
	} while(0)

	memset(&cmd_buf, 0, sizeof(cmd_buf));
	sdwidth = sswidth = 0;
	flags = HARDDOOM2_CMD_FLAG_INTERLOCK;
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

	harddoom2_send_cmd(ctx->pddata, &cmd_buf);

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
			if (ret == 0) {
				ret = err;
			}
			break;
		}
		harddoom2_send_cmd(ctx->pddata, &cmd_buf);
		ret += sizeof(dev_cmd);
	}

	// we need to access active buffers while validating commands
	// todo (optimization) -- make a copy, and remember about ref counts!
	mutex_unlock(&ctx->active_bufs_mutex);

	// wait until commands are processed
	memset(&cmd_buf, 0, sizeof(cmd_buf));
	cmd_buf.w[0] = HARDDOOM2_CMD_W0_SETUP(HARDDOOM2_CMD_TYPE_SETUP,
		HARDDOOM2_CMD_FLAG_INTERLOCK | HARDDOOM2_CMD_FLAG_PING_SYNC, 0, 0);
	harddoom2_send_cmd(ctx->pddata, &cmd_buf);
	wait_for_completion(&ctx->pddata->pong_sync);

exit_op:
	mutex_unlock(&ctx->pddata->sync_queue_mutex);

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


static int harddoom2_probe(struct pci_dev *pdev, const struct pci_device_id *_id) {
	int ret = 0;
	int minor_zero_based;
	dev_t minor;
	uint32_t code_it, code_size;
	void __iomem *bar0;
	struct harddoom2_pcidevdata *pddata;
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
		goto err_0;
	}

	bar0 = pci_iomap(pdev, 0, 0);
	if (!bar0) {
		dev_err(&pdev->dev, "Failed: pci_iomap");
		ret = -ENOMEM;
		goto err_0;
	}

	// alloc drvdata
	pddata = (struct harddoom2_pcidevdata*) kmalloc(sizeof(struct harddoom2_pcidevdata), GFP_KERNEL);
	if (!pddata) {
		dev_err(&pdev->dev, "Failed: kmalloc for driver data");
		ret = -ENOMEM;
		goto err_1;
	}
	pddata->pdev = pdev;
	pddata->bar0 = bar0;
	memset(&pddata->last_active_bufs, 0, sizeof(pddata->last_active_bufs));
	mutex_init(&pddata->sync_queue_mutex);
	init_completion(&pddata->pong_sync);
	pci_set_drvdata(pdev, pddata);

	// set up dma
	pci_set_master(pdev);
	ret = pci_set_dma_mask(pdev, HARDDOOM2_DMA_MASK);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set PCI DMA mask");
		goto err_2;
	}
	ret = pci_set_consistent_dma_mask(pdev, HARDDOOM2_DMA_MASK);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set consistent PCI DMA mask");
		goto err_2;
	}

	// register interruption handler
	ret = request_irq(pdev->irq, harddoom2_irq_handler, IRQF_SHARED, harddoom2_name, pddata);
	if (ret) {
		dev_err(&pdev->dev, "Failed: request_irq");
		goto err_2;
	}

	// boot sequence
	iowrite32(0, bar0 + HARDDOOM2_FE_CODE_ADDR);
	code_size = ARRAY_SIZE(doomcode2);
	for (code_it = 0; code_it < code_size; code_it++) {
		iowrite32(doomcode2[code_it], bar0 + HARDDOOM2_FE_CODE_WINDOW);
	}
	iowrite32(HARDDOOM2_RESET_ALL, bar0 + HARDDOOM2_RESET);
	// todo init CMD_PT, CMD_SIZE and CMD_*_IDX,
	iowrite32(HARDDOOM2_INTR_MASK, bar0 + HARDDOOM2_INTR);
	iowrite32(HARDDOOM2_INTR_MASK, bar0 + HARDDOOM2_INTR_ENABLE);
	// todo init FENCE_COUNTER
	iowrite32(HARDDOOM2_ENABLE_ALL & ~HARDDOOM2_ENABLE_CMD_FETCH, bar0 + HARDDOOM2_ENABLE);

	// prepare chrdev (after booting device)
	cdev_init(&pddata->doomdev2, &doomdev2_fops);
	minor = doomdev2_alloc_minor();
	minor_zero_based = minor - doomdev2_major;
	if (minor == DOOMDEV2_NO_AVAILABLE_MINOR) {
		dev_err(&pdev->dev, "Failed: no available minor to register new device");
		ret = -ENOSPC;
		goto err_3;
	}
	ret = cdev_add(&pddata->doomdev2, minor, 1);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add cdev, minor %d", minor_zero_based);
		goto err_4;
	}
	sysfs_dev = device_create(&doomdev2_class, &pdev->dev, minor, NULL, "doom%d", minor_zero_based);
	if (IS_ERR(sysfs_dev)) {
		dev_err(&pdev->dev, "Failed to create sysfs device, minor %d", minor_zero_based);
		ret = PTR_ERR(sysfs_dev);
		goto err_5;
	}

	return ret;

err_5:
	cdev_del(&pddata->doomdev2);
err_4:
	doomdev2_dealloc_minor(minor);
err_3:
	iowrite32(0, pddata->bar0 + HARDDOOM2_ENABLE);
	iowrite32(0, pddata->bar0 + HARDDOOM2_INTR_ENABLE);
	ioread32(pddata->bar0 + HARDDOOM2_ENABLE);
	free_irq(pdev->irq, pddata);
err_2:
	pci_clear_master(pdev);
	kfree(pddata);
err_1:
	pci_iounmap(pdev, bar0);
err_0:
	pci_disable_device(pdev);
	pci_release_regions(pdev); // should be called after pci_disable_device()

	return ret;
}

static void harddoom2_remove(struct pci_dev *pdev) {
	struct harddoom2_pcidevdata *pddata = (struct harddoom2_pcidevdata*) pci_get_drvdata(pdev);

	// "unboot" device
	iowrite32(0, pddata->bar0 + HARDDOOM2_ENABLE);
	iowrite32(0, pddata->bar0 + HARDDOOM2_INTR_ENABLE);
	ioread32(pddata->bar0 + HARDDOOM2_ENABLE);
	free_irq(pddata->pdev->irq, pddata);

	// destroy chrdev
	device_destroy(&doomdev2_class, pddata->doomdev2.dev);
	cdev_del(&pddata->doomdev2); // mwk said "we don't have to care about opened fd"
	doomdev2_dealloc_minor(pddata->doomdev2.dev);

	// release pci device
	pci_clear_master(pdev);
	pci_iounmap(pdev, pddata->bar0);
	pci_disable_device(pdev);
	pci_release_regions(pdev); // should be called after pci_disable_device()
	kfree(pddata);
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
    // TODO
    // int  (*suspend) (struct pci_dev *dev, pm_message_t state);
    // int  (*resume) (struct pci_dev *dev);
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
