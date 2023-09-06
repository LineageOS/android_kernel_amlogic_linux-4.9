/*
 * pwm-fan.c - Hwmon driver for fans connected to PWM lines.
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *
 * Author: Kamil Debski <k.debski@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>

#define MAX_PWM 255

struct thermal_trips {
	int temp;
	int state;
};

struct pwm_fan_ctx {
	struct mutex lock;
	struct pwm_device *pwm;
	unsigned int pwm_value;
	unsigned int pwm_fan_state;
	unsigned int pwm_fan_max_state;
	unsigned int *pwm_fan_cooling_levels;
	unsigned int is_temp_trips_proprity;
	struct thermal_trips *thermal_trips;
	struct thermal_cooling_device *cdev;
};

static int  __set_pwm(struct pwm_fan_ctx *ctx, unsigned long pwm)
{
	struct pwm_args pargs;
	unsigned long duty;
	int ret = 0;

	pwm_get_args(ctx->pwm, &pargs);

	mutex_lock(&ctx->lock);
	if (ctx->pwm_value == pwm)
		goto exit_set_pwm_err;

	duty = DIV_ROUND_UP(pwm * (pargs.period - 1), MAX_PWM);
	ret = pwm_config(ctx->pwm, duty, pargs.period);
	if (ret)
		goto exit_set_pwm_err;

	if (pwm == 0)
		pwm_disable(ctx->pwm);

	if (ctx->pwm_value == 0) {
		ret = pwm_enable(ctx->pwm);
		if (ret)
			goto exit_set_pwm_err;
	}

	ctx->pwm_value = pwm;
exit_set_pwm_err:
	mutex_unlock(&ctx->lock);
	return ret;
}

static void pwm_fan_update_state(struct pwm_fan_ctx *ctx, unsigned long pwm)
{
	int i;

	for (i = 0; i < ctx->pwm_fan_max_state; ++i)
		if (pwm < ctx->pwm_fan_cooling_levels[i + 1])
			break;

	ctx->pwm_fan_state = i;
}

static ssize_t set_pwm(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	unsigned long pwm;
	int ret;

	if (kstrtoul(buf, 10, &pwm) || pwm > MAX_PWM)
		return -EINVAL;

	ret = __set_pwm(ctx, pwm);
	if (ret)
		return ret;

	pwm_fan_update_state(ctx, pwm);
	return count;
}

static ssize_t show_pwm(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ctx->pwm_value);
}


static SENSOR_DEVICE_ATTR(pwm1, S_IRUGO | S_IWUSR, show_pwm, set_pwm, 0);

static struct attribute *pwm_fan_attrs[] = {
	&sensor_dev_attr_pwm1.dev_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(pwm_fan);

static int pwm_fan_temp_to_state(struct pwm_fan_ctx *ctx, int temp)
{
	struct thermal_trips *trips;
	int i, state = 0;

	if (ctx->is_temp_trips_proprity != 1)
		return -EINVAL;

	trips = ctx->thermal_trips;

	for (i = 0; trips[i].state != INT_MAX; i++) {
		if (temp >= trips[i].temp)
			state = trips[i].state;
	}

	//ctx->pwm_fan_state = state;

	return state;
}

/* thermal cooling device callbacks */
static int pwm_fan_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct pwm_fan_ctx *ctx = cdev->devdata;

	if (!ctx)
		return -EINVAL;

	*state = ctx->pwm_fan_max_state;

	return 0;
}

static int pwm_fan_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct pwm_fan_ctx *ctx = cdev->devdata;

	if (!ctx)
		return -EINVAL;

	*state = ctx->pwm_fan_state;

	return 0;
}

static int
pwm_fan_set_cur_state(struct thermal_cooling_device *cdev, unsigned long state)
{
	struct pwm_fan_ctx *ctx = cdev->devdata;
	int ret;

	if (!ctx || (state > ctx->pwm_fan_max_state))
		return -EINVAL;

	if (state == ctx->pwm_fan_state)
		return 0;

	ret = __set_pwm(ctx, ctx->pwm_fan_cooling_levels[state]);
	if (ret) {
		dev_err(&cdev->device, "Cannot set pwm!\n");
		return ret;
	}

	ctx->pwm_fan_state = state;

	return ret;
}

static int
pwm_fan_power2state(struct thermal_cooling_device *cdev,
			       struct thermal_zone_device *tz, u32 power,
			       unsigned long *state)
{
	struct pwm_fan_ctx *ctx = cdev->devdata;
	int i, temp;

	temp = tz->temperature;
	i = pwm_fan_temp_to_state(ctx, temp);
	if (i >= 0)
		*state = i;
	else
		*state = 0;

	return 0;
}

static int pwm_fan_state2power(struct thermal_cooling_device *cdev,
			       struct thermal_zone_device *tz,
			       unsigned long state, u32 *power)
{
	*power = 0;

	return 0;
}

static int pwm_fan_get_requested_power(struct thermal_cooling_device *cdev,
				       struct thermal_zone_device *tz,
				       u32 *power)
{
	*power = 0;

	return 0;
}

static const struct thermal_cooling_device_ops pwm_fan_cooling_ops = {
	.get_max_state = pwm_fan_get_max_state,
	.get_cur_state = pwm_fan_get_cur_state,
	.set_cur_state = pwm_fan_set_cur_state,
	.power2state   = pwm_fan_power2state,
	.state2power   = pwm_fan_state2power,
	.get_requested_power = pwm_fan_get_requested_power,
};

static int pwm_fan_of_get_cooling_data(struct device *dev,
				       struct pwm_fan_ctx *ctx)
{
	struct device_node *np = dev->of_node;
	int num, i, ret;

	if (!of_find_property(np, "cooling-levels", NULL))
		return 0;

	ret = of_property_count_u32_elems(np, "cooling-levels");
	if (ret <= 0) {
		dev_err(dev, "Wrong data!\n");
		return ret ? : -EINVAL;
	}

	num = ret;
	ctx->pwm_fan_cooling_levels = devm_kzalloc(dev, num * sizeof(u32),
						   GFP_KERNEL);
	if (!ctx->pwm_fan_cooling_levels)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "cooling-levels",
					 ctx->pwm_fan_cooling_levels, num);
	if (ret) {
		dev_err(dev, "Property 'cooling-levels' cannot be read!\n");
		return ret;
	}

	for (i = 0; i < num; i++) {
		if (ctx->pwm_fan_cooling_levels[i] > MAX_PWM) {
			dev_err(dev, "PWM fan state[%d]:%d > %d\n", i,
				ctx->pwm_fan_cooling_levels[i], MAX_PWM);
			return -EINVAL;
		}
	}

	ctx->pwm_fan_max_state = num - 1;

	return 0;
}

static int pwm_fan_get_thermal_trips(struct device *dev, char *porp_name,
				     struct thermal_trips **trips)
{
	struct device_node *np = dev->of_node;
	struct thermal_trips *thermal_trips;
	const struct property *prop;
	int count, i;

	prop = of_find_property(np, porp_name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;
	count = of_property_count_u32_elems(np, porp_name);
	if (count < 0)
		return -EINVAL;
	if (count % 2)
		return -EINVAL;
	thermal_trips = devm_kzalloc(dev,
				     sizeof(*thermal_trips) * (count / 2 + 1),
				     GFP_KERNEL);
	if (!thermal_trips)
		return -ENOMEM;

	for (i = 0; i < count / 2; i++) {
		of_property_read_u32_index(np, porp_name, 2 * i,
					   &thermal_trips[i].temp);
		of_property_read_u32_index(np, porp_name, 2 * i + 1,
					   &thermal_trips[i].state);
	}
	thermal_trips[i].temp = 0;
	thermal_trips[i].state = INT_MAX;

	*trips = thermal_trips;

	return 0;
}

static int pwm_fan_probe(struct platform_device *pdev)
{
	struct thermal_cooling_device *cdev;
	struct pwm_fan_ctx *ctx;
	struct pwm_args pargs;
	struct device *hwmon;
	int duty_cycle;
	int ret;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mutex_init(&ctx->lock);

	ctx->pwm = devm_of_pwm_get(&pdev->dev, pdev->dev.of_node, NULL);
	if (IS_ERR(ctx->pwm)) {
		ret = PTR_ERR(ctx->pwm);

		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Could not get PWM: %d\n", ret);

		return ret;
	}

	platform_set_drvdata(pdev, ctx);

	/*
	 * FIXME: pwm_apply_args() should be removed when switching to the
	 * atomic PWM API.
	 */
	pwm_apply_args(ctx->pwm);

	/* Set duty cycle to maximum allowed */
	pwm_get_args(ctx->pwm, &pargs);

	duty_cycle = pargs.period - 1;
	ctx->pwm_value = MAX_PWM;

	ret = pwm_config(ctx->pwm, duty_cycle, pargs.period);
	if (ret) {
		dev_err(&pdev->dev, "Failed to configure PWM\n");
		return ret;
	}

	/* Enbale PWM output */
	ret = pwm_enable(ctx->pwm);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable PWM\n");
		return ret;
	}

	hwmon = devm_hwmon_device_register_with_groups(&pdev->dev, "pwmfan",
						       ctx, pwm_fan_groups);
	if (IS_ERR(hwmon)) {
		dev_err(&pdev->dev, "Failed to register hwmon device\n");
		pwm_disable(ctx->pwm);
		return PTR_ERR(hwmon);
	}

	ret = pwm_fan_of_get_cooling_data(&pdev->dev, ctx);
	if (ret)
		return ret;

	if (of_find_property(pdev->dev.of_node, "amlogic,temp-trips", NULL))
		if (pwm_fan_get_thermal_trips(&pdev->dev,
					"amlogic,temp-trips",
					&ctx->thermal_trips) == 0)
			ctx->is_temp_trips_proprity = 1;

	ctx->pwm_fan_state = ctx->pwm_fan_max_state;
	if (IS_ENABLED(CONFIG_THERMAL)) {
		cdev = thermal_of_cooling_device_register(pdev->dev.of_node,
							  "pwm-fan", ctx,
							  &pwm_fan_cooling_ops);
		if (IS_ERR(cdev)) {
			dev_err(&pdev->dev,
				"Failed to register pwm-fan as cooling device");
			pwm_disable(ctx->pwm);
			return PTR_ERR(cdev);
		}
		ctx->cdev = cdev;
		thermal_cdev_update(cdev);
	}

	return 0;
}

static int pwm_fan_remove(struct platform_device *pdev)
{
	struct pwm_fan_ctx *ctx = platform_get_drvdata(pdev);

	thermal_cooling_device_unregister(ctx->cdev);
	if (ctx->pwm_value)
		pwm_disable(ctx->pwm);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_fan_suspend(struct device *dev)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	struct pwm_args args;
	int ret;

	pwm_get_args(ctx->pwm, &args);

	if (ctx->pwm_value) {
		ret = pwm_config(ctx->pwm, 0, args.period);
		if (ret < 0)
			return ret;

		pwm_disable(ctx->pwm);
	}

	return 0;
}

static int pwm_fan_resume(struct device *dev)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	struct pwm_args pargs;
	unsigned long duty;
	int ret;

	if (ctx->pwm_value == 0)
		return 0;

	pwm_get_args(ctx->pwm, &pargs);
	duty = DIV_ROUND_UP(ctx->pwm_value * (pargs.period - 1), MAX_PWM);
	ret = pwm_config(ctx->pwm, duty, pargs.period);
	if (ret)
		return ret;
	return pwm_enable(ctx->pwm);
}
#endif

static SIMPLE_DEV_PM_OPS(pwm_fan_pm, pwm_fan_suspend, pwm_fan_resume);

static const struct of_device_id of_pwm_fan_match[] = {
	{ .compatible = "pwm-fan", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_fan_match);

static struct platform_driver pwm_fan_driver = {
	.probe		= pwm_fan_probe,
	.remove		= pwm_fan_remove,
	.driver	= {
		.name		= "pwm-fan",
		.pm		= &pwm_fan_pm,
		.of_match_table	= of_pwm_fan_match,
	},
};

module_platform_driver(pwm_fan_driver);

MODULE_AUTHOR("Kamil Debski <k.debski@samsung.com>");
MODULE_ALIAS("platform:pwm-fan");
MODULE_DESCRIPTION("PWM FAN driver");
MODULE_LICENSE("GPL");
