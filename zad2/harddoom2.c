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

static struct pci_driver harddoom2_driver;

struct harddoom2_devctx {
	void __iomem *bar0;
};


static int harddoom2_probe(struct pci_dev *pdev, const struct pci_device_id *_id) {
	int ret = 0;
	uint32_t code_it, code_size;
	void __iomem *bar0;
	struct harddoom2_devctx *devctx;

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
		ret = -EFAULT;
		goto err_0;
	}

	// alloc drvdata and bind
	devctx = (struct harddoom2_devctx*) kmalloc(sizeof(struct harddoom2_devctx), GFP_KERNEL);
	if (!devctx) {
		dev_err(&pdev->dev, "Failed: kmalloc for driver data");
		ret = -ENOMEM;
		goto err_1;
	}
	devctx->bar0 = bar0;
	pci_set_drvdata(pdev, devctx);

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

	// todo do i need dma?

	return ret;

err_1:
	pci_iounmap(pdev, bar0);
err_0:
	pci_disable_device(pdev);
	pci_release_regions(pdev); // should be called after pci_disable_device()
	return ret;
}

static void harddoom2_remove(struct pci_dev *pdev) {
	struct harddoom2_devctx *devctx = (struct harddoom2_devctx*) pci_get_drvdata(pdev);

	// "unboot" device
	iowrite32(0, devctx->bar0 + HARDDOOM2_ENABLE);
	iowrite32(0, devctx->bar0 + HARDDOOM2_INTR_ENABLE);
	ioread32(devctx->bar0 + HARDDOOM2_ENABLE);

	pci_iounmap(pdev, devctx->bar0);
	pci_disable_device(pdev);
	pci_release_regions(pdev); // should be called after pci_disable_device()
	kfree(devctx);
}

static const struct pci_device_id harddoom2_pci_tbl[] = {
	{ PCI_DEVICE(HARDDOOM2_VENDOR_ID, HARDDOOM2_DEVICE_ID) },
	{ 0, },
};

static struct pci_driver harddoom2_driver = {
    .name = "HardDoom ][™", // todo make it var-name-friendly?
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
	int rc = 0;
	
	rc = pci_register_driver(&harddoom2_driver);
	if (rc) {
		return rc;
	}

	return 0;
}

static void harddoom2_cleanup(void)
{
	pci_unregister_driver(&harddoom2_driver);
}

module_init(harddoom2_init);
module_exit(harddoom2_cleanup);
