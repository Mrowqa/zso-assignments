// TODO remove unused
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/cred.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include "harddoom2.h"
#include "doomcode2.h"
#include "doomdev2.h"

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("HardDoom ][tm device");

#define HARDDOOM2_MAX_DEVICES	256
#define HARDDOOM2_DMA_MASK		DMA_BIT_MASK(40)
#define DOOMDEV2_NO_AVAILABLE_MINOR		(-1)
#define DOOMDEV2_MIN_SURFACE_SIZE 		1
#define DOOMDEV2_SURFACE_WIDTH_DIVIDER	64
#define DOOMDEV2_MAX_SURFACE_SIZE 		2048

static const char harddoom2_name[] = { "harddoom2" };
static struct pci_driver harddoom2_driver;
static dev_t doomdev2_major;
static struct class doomdev2_class = {
	.name = harddoom2_name,
	.owner = THIS_MODULE,
};
static DECLARE_BITMAP(doomdev2_used_minors, HARDDOOM2_MAX_DEVICES);
static DEFINE_MUTEX(doomdev2_used_minors_mutex);

struct harddoom2_pcidevdata {
	struct pci_dev *pdev;
	void __iomem *bar0;
	struct cdev doomdev2;
};

struct doomdev2_dma_page {
	void *phys_addr;
	dma_addr_t dma_handle;
};

// represents surfaces and buffers
struct doomdev2_dma_buffer {
	int32_t width;
	int32_t height;
	int32_t fd;

	size_t pages_cnt;
	struct doomdev2_dma_page pages[0];
};

struct doomdev2_ctx {
	struct harddoom2_pcidevdata *pddata;
	// todo for validation of buffers -> map fd to buffer, know it type and size
	// todo active buffers fds
	// todo allocated dma memory for dealloc
};

// todo remove width and height?
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
	dma_buf->width = -1;   // to be set by user
	dma_buf->height = -1;  // ^
	dma_buf->fd = -1;      // ^
	dma_buf->pages_cnt = pages_cnt;

	// alloc pages
	pages_end = dma_buf->pages + pages_cnt;
	for (page_it = dma_buf->pages; page_it < pages_end; page_it++) {
		page_it->phys_addr = dma_alloc_coherent(&ctx->pddata->pdev->dev, HARDDOOM2_PAGE_SIZE,
			&page_it->dma_handle, GFP_KERNEL | __GFP_ZERO);
		if (!page_it->phys_addr) {
			goto dma_alloc_err;
		}
	}

	return dma_buf;

dma_alloc_err:
	for (page_it--; page_it >= dma_buf->pages; page_it--) {
		dma_free_coherent(&ctx->pddata->pdev->dev, HARDDOOM2_PAGE_SIZE, page_it->phys_addr, page_it->dma_handle);
	}
	kfree(dma_buf);

	return NULL;
}

static void doomdev2_dma_dealloc_buffer(struct doomdev2_ctx *ctx, struct doomdev2_dma_buffer *dma_buf) {
	// todo
}

static void doomdev2_dma_format_page_table(struct doomdev2_dma_buffer *dma_buf) {
	// todo
}

static int doomdev2_open(struct inode *ino, struct file *filep) {
	int ret = 0;
	struct harddoom2_pcidevdata *pddata;
	struct doomdev2_ctx *ctx = (struct doomdev2_ctx*) kmalloc(sizeof(struct doomdev2_ctx), GFP_KERNEL);
	if (!ctx) {
		return -ENOMEM;
	}

	pddata = container_of(ino->i_cdev, struct harddoom2_pcidevdata, doomdev2);
	ctx->pddata = pddata;

	filep->private_data = ctx;

	return ret;
}

static int doomdev2_release(struct inode *_ino, struct file *filep) {
	// dealloc all data, dma, etc
	// todo

	kfree(filep->private_data);
	filep->private_data = NULL;

	return 0;
}

static long doomdev2_ioctl_create_surface(struct doomdev2_ctx *ctx, struct doomdev2_ioctl_create_surface *data_cs) {
	size_t dma_alloc_size;
	size_t dma_pages_cnt;
	struct doomdev2_dma_buffer *dma_buf;

	if (data_cs->width % DOOMDEV2_SURFACE_WIDTH_DIVIDER != 0
			|| data_cs->width < DOOMDEV2_MIN_SURFACE_SIZE || data_cs->width > DOOMDEV2_MAX_SURFACE_SIZE
			|| data_cs->height < DOOMDEV2_MIN_SURFACE_SIZE || data_cs->height > DOOMDEV2_MAX_SURFACE_SIZE) {
		return -EINVAL;
	}

	// allocate dma memory
	dma_alloc_size = data_cs->width * data_cs->height;
	dma_pages_cnt = dma_alloc_size / HARDDOOM2_PAGE_SIZE
		+ (dma_alloc_size % HARDDOOM2_PAGE_SIZE > 0)
		+ 1; // +1 for page table

	dma_buf = doomdev2_dma_alloc_buffer(ctx, dma_pages_cnt);
	if (!dma_buf) {
		return -ENOMEM;
	}
	dma_buf->width = data_cs->width; // todo as macro?
	dma_buf->height = data_cs->height; // todo macro for detecting buffer type?
	doomdev2_dma_format_page_table(dma_buf);

	// allocate fd
	// todo

	return 0; // ret fd
}

static long doomdev2_ioctl(struct file *filep, unsigned int cmd, unsigned long arg) {
	long ret;
	void __user *arg_ptr = (void __user*) arg;
	struct doomdev2_ctx *ctx = (struct doomdev2_ctx*) filep->private_data;

	struct doomdev2_ioctl_create_surface data_cs;
	// struct doomdev2_ioctl_create_buffer data_cb; // todo
	// struct doomdev2_ioctl_setup data_s; // todo

	switch (cmd) {
		case DOOMDEV2_IOCTL_CREATE_SURFACE:
			ret = copy_from_user(&data_cs, arg_ptr, sizeof(data_cs));
			if (!ret) {
				return -EFAULT;
			}
			ret = doomdev2_ioctl_create_surface(ctx, &data_cs);
			break;
		case DOOMDEV2_IOCTL_CREATE_BUFFER:
			ret = 0; // todo call
			break;
		case DOOMDEV2_IOCTL_SETUP:
			ret = 0; // todo call
			break;
		default:
			ret = -ENOTTY;
	}

	return ret;
}

static struct file_operations doomdev2_fops = {
	.owner = THIS_MODULE,
	.open = doomdev2_open,
	.release = doomdev2_release,
	.unlocked_ioctl = doomdev2_ioctl,
	.compat_ioctl = doomdev2_ioctl,
	// todo
	// loff_t (*llseek) (struct file *, loff_t, int);
    // ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
    // ssize_t (*write) (struct file *, const char __user *, size_t,
    //     loff_t *);
    // int (*mmap) (struct file *, struct vm_area_struct *);
    /* ... */
};


static dev_t doomdev2_alloc_minor(void) {
	dev_t ret = DOOMDEV2_NO_AVAILABLE_MINOR;
	int minor;

	mutex_lock(&doomdev2_used_minors_mutex);
	minor = find_next_zero_bit(doomdev2_used_minors, HARDDOOM2_MAX_DEVICES, 0);
	if (minor < HARDDOOM2_MAX_DEVICES) { // success
		bitmap_set(doomdev2_used_minors, minor, 1);
		ret = doomdev2_major + minor;
	}
	mutex_unlock(&doomdev2_used_minors_mutex);

	return ret;
}

static void doomdev2_dealloc_minor(dev_t minor) {
	mutex_lock(&doomdev2_used_minors_mutex);
	bitmap_clear(doomdev2_used_minors, minor - doomdev2_major, 1);
	mutex_unlock(&doomdev2_used_minors_mutex);
}


static int harddoom2_probe(struct pci_dev *pdev, const struct pci_device_id *_id) {
	int ret = 0;
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
	pci_set_drvdata(pdev, pddata);

	// set up dma
	pci_set_master(pdev);
	ret = pci_set_dma_mask(pdev, HARDDOOM2_DMA_MASK);
	if (!ret) {
		goto err_2;
	}
	ret = pci_set_consistent_dma_mask(pdev, HARDDOOM2_DMA_MASK);
	if (!ret) {
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
	// todo init INTR_ENABLE (0 by default)
	// todo init FENCE_COUNTER
	iowrite32(HARDDOOM2_ENABLE_ALL & ~HARDDOOM2_ENABLE_CMD_FETCH, bar0 + HARDDOOM2_ENABLE);

	// prepare chrdev (after booting device)
	cdev_init(&pddata->doomdev2, &doomdev2_fops);
	minor = doomdev2_alloc_minor();
	if (minor == DOOMDEV2_NO_AVAILABLE_MINOR) {
		ret = -ENOSPC;
		goto err_3;
	}
	ret = cdev_add(&pddata->doomdev2, minor, 1);
	if (ret) {
		goto err_4;
	}
	sysfs_dev = device_create(&doomdev2_class, &pdev->dev, minor, NULL /*todo*/, "doom%d", minor - doomdev2_major);
	if (IS_ERR(sysfs_dev)) {
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
    /* ... */
};

static int harddoom2_init(void)
{
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

static void harddoom2_cleanup(void)
{
	pci_unregister_driver(&harddoom2_driver);
	class_unregister(&doomdev2_class);
	unregister_chrdev_region(doomdev2_major, HARDDOOM2_MAX_DEVICES);
}

module_init(harddoom2_init);
module_exit(harddoom2_cleanup);
