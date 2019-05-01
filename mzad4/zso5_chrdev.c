#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/cred.h>
#include <linux/slab.h>

MODULE_LICENSE("GPL");

#define HELLO_IOCTL_SET_REPEATS _IO('H', 0x00)
#define HELLO_MAX_REPEATS 0x100
#define HELLO_MAX_WRITE_BYTES 32
#define HELLO_DEFAULT_REPEATS 1

static const char hello_reply[] = "Hello, world!\n";
static size_t hello_len = sizeof hello_reply - 1;
static dev_t hello_major;
static struct cdev hello_cdev;
static struct cdev hello_once_cdev;
static struct class hello_class = {
	.name = "hello",
	.owner = THIS_MODULE,
};
static struct device *hello_device;
static struct device *hello_once_device;


typedef struct repeats_map {
	struct list_head list;
	uid_t uid;
	long repeats;
} repeats_map_t;

static LIST_HEAD(hello_repeats_map);
static DEFINE_MUTEX(hello_repeats_mutex);

static ssize_t hello_repeats_set(uid_t user, long value) {
	repeats_map_t *pos = NULL;
	repeats_map_t *new_elem = NULL;
	int found = 0;
	ssize_t result = 0;

	mutex_lock(&hello_repeats_mutex);
	list_for_each_entry(pos, &hello_repeats_map, list) {
		if (pos->uid == user) {
			pos->repeats = value;
			found = 1;
			break;
		}
	}
	if (!found && value != HELLO_DEFAULT_REPEATS) {
		new_elem = kmalloc(sizeof(repeats_map_t), GFP_KERNEL);
		if (new_elem == NULL) {
			result = -ENOMEM;
		}
		else {
			INIT_LIST_HEAD(&new_elem->list);
			new_elem->uid = user;
			new_elem->repeats = value;
			list_add_tail(&new_elem->list, &hello_repeats_map);
		}
	}
	mutex_unlock(&hello_repeats_mutex);

	return result;
}

static long hello_repeats_get(uid_t user) {
	repeats_map_t *pos = NULL;
	long result = HELLO_DEFAULT_REPEATS;

	mutex_lock(&hello_repeats_mutex);
	list_for_each_entry(pos, &hello_repeats_map, list) {
		if (pos->uid == user) {
			result = pos->repeats;
			break;
		}
	}
	mutex_unlock(&hello_repeats_mutex);

	return result;
}

static void hello_repeats_cleanup(void) {
	repeats_map_t *pos = NULL;
	repeats_map_t *n = NULL;

	mutex_lock(&hello_repeats_mutex);
	list_for_each_entry_safe(pos, n, &hello_repeats_map, list) {
		list_del(&pos->list);
		kfree(pos);
	}
	mutex_unlock(&hello_repeats_mutex);
}


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
	uid_t uid = __kuid_val(task_uid(current));
	size_t file_len = hello_len * hello_repeats_get(uid);
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

static ssize_t hello_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp) {
	char buffer[HELLO_MAX_WRITE_BYTES] = {0};
	long repeats = 0;
	size_t it = 0;
	uid_t uid;
	ssize_t err_code;
	
	if (count > HELLO_MAX_WRITE_BYTES) {
		return -EINVAL;
	}

	if (copy_from_user(buffer, buff, count) != 0) {
		return -EFAULT;
	}

	for (it = 0; it < count; it++) {
		if (it == count - 1 && buffer[it] == '\n') {
			break;
		}
		else if (buffer[it] < '0' || '9' < buffer[it]) {
			return -EINVAL;
		}
		repeats = repeats * 10 + (buffer[it] - '0');
		if (HELLO_MAX_REPEATS < repeats) {
			return -EINVAL;
		}
	}

	uid = __kuid_val(task_uid(current)); // no need to read it before (uses rcu lock)
	err_code = hello_repeats_set(uid, repeats);

	return err_code != 0 ? err_code : count;
}

static long hello_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	uid_t uid;
	if (cmd != HELLO_IOCTL_SET_REPEATS)
		return -ENOTTY;
	if (arg > HELLO_MAX_REPEATS)
		return -EINVAL;

	uid = __kuid_val(task_uid(current)); // no need to read it before (uses rcu lock)
	return hello_repeats_set(uid, arg);
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
	.write = hello_write,
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
	hello_repeats_cleanup();
	device_destroy(&hello_class, hello_major + 1);
	device_destroy(&hello_class, hello_major);
	class_unregister(&hello_class);
	cdev_del(&hello_once_cdev);
	cdev_del(&hello_cdev);
	unregister_chrdev_region(hello_major, 2);
}

module_init(hello_init);
module_exit(hello_cleanup);
