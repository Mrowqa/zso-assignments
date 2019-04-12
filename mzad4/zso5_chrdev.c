#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>

MODULE_LICENSE("GPL");

#define HELLO_IOCTL_SET_REPEATS _IO('H', 0x00)
#define HELLO_MAX_REPEATS 0x100

static const char hello_reply[] = "Hello, world!\n";
static size_t hello_len = sizeof hello_reply - 1;
static long hello_repeats = 1;
static dev_t hello_major;
static struct cdev hello_cdev;
static struct cdev hello_once_cdev;
static struct class hello_class = {
	.name = "hello",
	.owner = THIS_MODULE,
};
static struct device *hello_device;
static struct device *hello_once_device;

static ssize_t hello_once_read(struct file *file, char __user *buf, size_t count, loff_t *filepos)
{
	loff_t pos = *filepos;
	if (pos >= hello_len || pos < 0)
		return 0;
	if (count > hello_len - pos)
		count = hello_len - pos;
	if (copy_to_user(buf, hello_reply + pos, count))
		return -EFAULT;
	*filepos = pos + count;
	return count;
}

static ssize_t hello_read(struct file *file, char __user *buf, size_t count, loff_t *filepos)
{
	size_t file_len = hello_len * hello_repeats;
	loff_t pos = *filepos;
	loff_t end;
	if (pos >= file_len || pos < 0)
		return 0;
	if (count > file_len - pos)
		count = file_len - pos;
	end = pos + count;
	while (pos < end)
		if (put_user(hello_reply[pos++ % hello_len], buf++))
			return -EFAULT;
	*filepos = pos;
	return count;
}

static long hello_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	if (cmd != HELLO_IOCTL_SET_REPEATS)
		return -ENOTTY;
	if (arg > HELLO_MAX_REPEATS)
		return -EINVAL;
	hello_repeats = arg;
	return 0;
}

static int hello_open(struct inode *ino, struct file *filep);
static int hello_release(struct inode *ino, struct file *filep);

static struct file_operations hello_once_fops = {
	.owner = THIS_MODULE,
	.read = hello_once_read,
	.open = hello_open,
	.release = hello_release,
};

static struct file_operations hello_fops = {
	.owner = THIS_MODULE,
	.read = hello_read,
	.open = hello_open,
	.unlocked_ioctl = hello_ioctl,
	.compat_ioctl = hello_ioctl,
	.release = hello_release,
};

static int hello_open(struct inode *ino, struct file *filep)
{
	return 0;
}

static int hello_release(struct inode *ino, struct file *filep)
{
	return 0;
}

static int hello_init(void)
{
	int err;
	if ((err = alloc_chrdev_region(&hello_major, 0, 2, "hello")))
		goto err_alloc;
	cdev_init(&hello_cdev, &hello_fops);
	if ((err = cdev_add(&hello_cdev, hello_major, 1)))
		goto err_cdev;
	cdev_init(&hello_once_cdev, &hello_once_fops);
	if ((err = cdev_add(&hello_once_cdev, hello_major + 1, 1)))
		goto err_cdev_2;
	if ((err = class_register(&hello_class)))
		goto err_class;
	hello_device = device_create(&hello_class, 0, hello_major, 0, "hello");
	if (IS_ERR(hello_device)) {
		err = PTR_ERR(hello_device);
		goto err_device;
	}
	hello_once_device = device_create(&hello_class, 0, hello_major + 1, 0, "hello_once");
	if (IS_ERR(hello_once_device)) {
		err = PTR_ERR(hello_once_device);
		goto err_device_2;
	}

	return 0;

err_device_2:
	device_destroy(&hello_class, hello_major);
err_device:
	class_unregister(&hello_class);
err_class:
	cdev_del(&hello_once_cdev);
err_cdev_2:
	cdev_del(&hello_cdev);
err_cdev:
	unregister_chrdev_region(hello_major, 2);
err_alloc:
	return err;
}

static void hello_cleanup(void)
{
	device_destroy(&hello_class, hello_major + 1);
	device_destroy(&hello_class, hello_major);
	class_unregister(&hello_class);
	cdev_del(&hello_once_cdev);
	cdev_del(&hello_cdev);
	unregister_chrdev_region(hello_major, 2);
}

module_init(hello_init);
module_exit(hello_cleanup);
