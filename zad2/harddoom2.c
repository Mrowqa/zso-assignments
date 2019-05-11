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

MODULE_LICENSE("GPL");

#define HARDDOOM2_MAX_DEVICES 256
#define DOOMDEV_NO_AVAILABLE_MINOR (-1)

static const char harddoom2_name[] = { "harddoom2" };
static struct pci_driver harddoom2_driver;
static dev_t doomdev_major;
static struct class doomdev_class = {
	.name = harddoom2_name,
	.owner = THIS_MODULE,
};
static DECLARE_BITMAP(doomdev_used_minors, HARDDOOM2_MAX_DEVICES);
static DEFINE_MUTEX(doomdev_used_minors_mutex);

struct harddoom2_pcidevdata {
	void __iomem *bar0;
	struct cdev doom_cdev;
	struct device *doom_dev; // todo remove, not needed
};


static struct file_operations doomdev_fops = {
	.owner = THIS_MODULE,
	// loff_t (*llseek) (struct file *, loff_t, int);
    // ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
    // ssize_t (*write) (struct file *, const char __user *, size_t,
    //     loff_t *);
    // int (*unlocked_ioctl) (struct file *, unsigned int,
    //     unsigned long);
    // int (*compat_ioctl) (struct file *, unsigned int,
    //     unsigned long);
    // int (*mmap) (struct file *, struct vm_area_struct *);
    // int (*open) (struct inode *, struct file *);
    // int (*release) (struct inode *, struct file *);
    /* ... */
};


static dev_t doomdev_alloc_minor(void) {
	dev_t ret = DOOMDEV_NO_AVAILABLE_MINOR;
	int minor;

	mutex_lock(&doomdev_used_minors_mutex);
	minor = find_next_zero_bit(doomdev_used_minors, HARDDOOM2_MAX_DEVICES, 0);
	if (minor < HARDDOOM2_MAX_DEVICES) { // success
		bitmap_set(doomdev_used_minors, minor, 1);
		ret = doomdev_major + minor;
	}
	mutex_unlock(&doomdev_used_minors_mutex);

	return ret;
}

static void doomdev_dealloc_minor(dev_t minor) {
	mutex_lock(&doomdev_used_minors_mutex);
	bitmap_clear(doomdev_used_minors, minor - doomdev_major, 1);
	mutex_unlock(&doomdev_used_minors_mutex);
}


static int harddoom2_probe(struct pci_dev *pdev, const struct pci_device_id *_id) {
	int ret = 0;
	dev_t minor;
	uint32_t code_it, code_size;
	void __iomem *bar0;
	struct harddoom2_pcidevdata *pddata;

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
	pddata->bar0 = bar0;
	pci_set_drvdata(pdev, pddata);

	// prepare chrdev
	cdev_init(&pddata->doom_cdev, &doomdev_fops);
	minor = doomdev_alloc_minor();
	if (minor == DOOMDEV_NO_AVAILABLE_MINOR) {
		ret = -ENOSPC;
		goto err_2;
	}
	ret = cdev_add(&pddata->doom_cdev, minor, 1);
	if (ret) {
		goto err_3;
	}
	pddata->doom_dev = device_create(&doomdev_class, &pdev->dev, minor, NULL /*todo*/, "doom%d", minor - doomdev_major);
	if (IS_ERR(pddata->doom_dev)) {
		ret = PTR_ERR(pddata->doom_dev);
		goto err_4;
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

	return ret;

err_4:
	cdev_del(&pddata->doom_cdev);
err_3:
	doomdev_dealloc_minor(minor);
err_2:
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
	device_destroy(&doomdev_class, pddata->doom_cdev.dev);
	cdev_del(&pddata->doom_cdev); // mwk said "we don't have to care about opened fd"
	doomdev_dealloc_minor(pddata->doom_cdev.dev);

	// release pci device
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
    /* TODO
    int  (*suspend) (struct pci_dev *dev, pm_message_t state);
    int  (*resume) (struct pci_dev *dev);
    void (*shutdown) (struct pci_dev *dev); */
    /* ... */
};

static int harddoom2_init(void)
{
	int ret = 0;
	
	ret = alloc_chrdev_region(&doomdev_major, 0, HARDDOOM2_MAX_DEVICES, harddoom2_name);
	if (ret) {
		return ret;
	}

	ret = class_register(&doomdev_class);
	if (ret) {
		goto err_0;
	}

	ret = pci_register_driver(&harddoom2_driver);
	if (ret) {
		goto err_1;
	}

	return ret;

err_1:
	class_unregister(&doomdev_class);
err_0:
	unregister_chrdev_region(doomdev_major, HARDDOOM2_MAX_DEVICES);
	return ret;
}

static void harddoom2_cleanup(void)
{
	pci_unregister_driver(&harddoom2_driver);
	class_unregister(&doomdev_class);
	unregister_chrdev_region(doomdev_major, HARDDOOM2_MAX_DEVICES);
}

module_init(harddoom2_init);
module_exit(harddoom2_cleanup);
