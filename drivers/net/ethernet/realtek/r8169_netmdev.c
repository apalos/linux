#include <linux/module.h>
/* Will be needed for 4.13.x
#include <linux/sched/task.h>
*/
#include <linux/moduleparam.h>

/* #include <linux/mm.h>
#include <linux/vfio.h>
#include <linux/mdev.h>
#include <linux/net_mdev.h>
*/


/* FIXME Placeholder file, all mdev related functions should be moved in here once
 * rtl8169 private struct is available on a header file
 */
void r8169_register_netmdev(struct device *dev)
{
	int (*register_device)(struct device *d , struct netmdev_driver_ops *ops);

	register_device  = symbol_get(netmdev_register_device);
	if (!register_device)
		return;
	if (register_device(dev, &rtl8169_netmdev_driver_ops) < 0)
		dev_err(dev, "Could not register device\n");
	else
		dev_info(dev, "Successfully registered net_mdev device\n");
	symbol_put(netmdev_register_device);
}

void r8169_unregister_netmdev(struct device *dev)
{
	int (*unregister_device)(struct device*);

	unregister_device = symbol_get(netmdev_unregister_device);
	if (!unregister_device)
		return;
	if (unregister_device(dev) < 0)
		dev_err(dev, "Could not unregister device\n");
	else
		dev_info(dev, "Successfully unregistered net_mdev device\n");
	symbol_put(netmdev_unregister_device);
}
