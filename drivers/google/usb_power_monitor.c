// Copyright 2020 Google Inc. All Rights Reserved.

#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/power_supply.h>
#include <linux/thermal.h>

struct usb_power_monitor_data {
	struct device *dev;
	struct gpio_desc *bc_chg_det_desc;
	bool bc_chg_det_online;
	struct power_supply *typec_psy;
	bool typec_psy_online;
	struct notifier_block usb_power_nb;
	bool usb_power_nb_registered;
	int power_max;
};

static const struct platform_device_id usb_power_monitor_id[] = {
	{ "usb_power_monitor", 0 },
	{ },
};

static const struct of_device_id usb_power_monitor_match[] = {
	{ .compatible = "google,usb-power-monitor",
		.data = &usb_power_monitor_id[0] },
	{ },
};

static int usb_power_monitor_update_power_max(
		struct usb_power_monitor_data *data, int power_max)
{
	if (data->power_max < power_max) {
		data->power_max = power_max;
		dev_info(data->dev, "power_max: %d", power_max);
	}
	return 0;
}

// return max power in micro-watt
static int usb_power_monitor_get_power_max(struct usb_power_monitor_data *data)
{
	if ((data->bc_chg_det_desc && !data->bc_chg_det_online) ||
	    (data->typec_psy && !data->typec_psy_online)) {
		dev_info(data->dev,
			 "Waiting for power supplies online, report 7.5W.");
		return 7500000;
	}

	return data->power_max;
}

static int usb_power_monitor_notifier(struct notifier_block *nb,
				      unsigned long action, void *p)
{
	struct power_supply *psy = p;
	struct usb_power_monitor_data *data =
		container_of(nb, struct usb_power_monitor_data, usb_power_nb);
	union power_supply_propval val;
	int ret;

	if (action != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_DONE;

	if (data->typec_psy && psy == data->typec_psy) {
		dev_info(data->dev, "notifier called, psy: %s\n",
			 psy->desc->name);

		ret = power_supply_get_property(
			psy, POWER_SUPPLY_PROP_ONLINE, &val);
		if (ret) {
			dev_err(data->dev, "%s: Failed to get online status.\n",
				psy->desc->name);
			data->typec_psy_online = false;
		} else {
			data->typec_psy_online = val.intval;
		}

		if (!data->typec_psy_online)
			return NOTIFY_OK;

		ret = power_supply_get_property(
			psy, POWER_SUPPLY_PROP_CURRENT_MAX, &val);
		if (ret)
			dev_err(data->dev, "%s: Failed to get current max.\n",
				psy->desc->name);
		else
			usb_power_monitor_update_power_max(
				data, val.intval * 5);
	} else {
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static ssize_t usb_power_monitor_show_power_max(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct usb_power_monitor_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", usb_power_monitor_get_power_max(data));
}

static SENSOR_DEVICE_ATTR(power1_max, 0444,
		usb_power_monitor_show_power_max, NULL, 0);

static struct attribute *usb_power_monitor_attrs[] = {
	&sensor_dev_attr_power1_max.dev_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(usb_power_monitor);


static int usb_power_monitor_read_temp(void *data, int *temp)
{
	*temp = -usb_power_monitor_get_power_max(data);
	return 0;
}

static const struct thermal_zone_of_device_ops
usb_power_monitor_of_thermal_ops = {
	.get_temp = usb_power_monitor_read_temp,
};

static int usb_power_monitor_probe(struct platform_device *pdev)
{
	struct thermal_zone_device *tz;
	struct device *hwmon_dev;
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id =
		of_match_device(of_match_ptr(usb_power_monitor_match), dev);
	const struct platform_device_id *pdev_id;
	struct usb_power_monitor_data *data;
	struct power_supply *typec_psy;
	struct gpio_desc *bc_chg_det_desc;
	int ret;

	data = devm_kzalloc(dev, sizeof(struct usb_power_monitor_data),
			    GFP_KERNEL);

	data->dev = dev;

	bc_chg_det_desc = devm_gpiod_get(dev, "bc_chg_det", GPIOD_IN);
	if (IS_ERR_OR_NULL(bc_chg_det_desc))
		dev_err(dev, "Failed to get bc_chg_det.\n");
	else
		data->bc_chg_det_desc = bc_chg_det_desc;

	typec_psy = devm_power_supply_get_by_phandle(dev, "typec_power_supply");
	if (IS_ERR_OR_NULL(typec_psy))
		dev_err(dev, "Failed to get typec_power_supply.\n");
	else
		data->typec_psy = typec_psy;

	if (of_id)
		pdev->id_entry = of_id->data;

	pdev_id = platform_get_device_id(pdev);

	/* Default to 2.5W (USB2.0) */
	usb_power_monitor_update_power_max(data, 2500000);

	hwmon_dev = devm_hwmon_device_register_with_groups(
			dev, pdev_id->name, data, usb_power_monitor_groups);
	if (IS_ERR(hwmon_dev)) {
		dev_err(dev, "unable to register as hwmon device.\n");
		return PTR_ERR(hwmon_dev);
	}

	tz = devm_thermal_zone_of_sensor_register(
			dev, 0, data, &usb_power_monitor_of_thermal_ops);
	if (IS_ERR(tz))
		dev_dbg(dev, "Failed to register to thermal fw.\n");

	data->usb_power_nb.priority = 0;
	data->usb_power_nb.notifier_call = usb_power_monitor_notifier;
	ret = power_supply_reg_notifier(&data->usb_power_nb);
	if (ret)
		dev_err(dev, "Register power supply notifier failed.\n");
	else
		data->usb_power_nb_registered = true;

	if (data->bc_chg_det_desc) {
		if (gpiod_get_value(data->bc_chg_det_desc))
			usb_power_monitor_update_power_max(data, 7500000);
		data->bc_chg_det_online = true;
	}

	dev_info(dev, "USB power monitor type: %s successfully probed.\n",
		 pdev_id->name);

	return 0;
}

static int usb_power_monitor_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct usb_power_monitor_data *data = dev_get_drvdata(dev);

	if (data->usb_power_nb_registered)
		power_supply_unreg_notifier(&data->usb_power_nb);

	return 0;
}

static struct platform_driver usb_power_monitor_driver = {
	.driver = {
		.name = "usb-power-monitor",
		.of_match_table = of_match_ptr(usb_power_monitor_match),
	},
	.probe = usb_power_monitor_probe,
	.remove = usb_power_monitor_remove,
	.id_table = usb_power_monitor_id,
};

module_platform_driver(usb_power_monitor_driver);

MODULE_DESCRIPTION("Google USB Power Monitor Driver");
MODULE_AUTHOR("Jacky Liu <qsliu@google.com>");
MODULE_LICENSE("Proprietary");
MODULE_ALIAS("platform:usb-power-monitor");
