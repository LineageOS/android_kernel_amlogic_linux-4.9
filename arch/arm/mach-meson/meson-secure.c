/*
 * arch/arm/mach-meson/meson-secure.c
 *
 * Copyright (C) 2017 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <asm/cacheflush.h>
#include <asm/hardware/cache-l2x0.h>
#include <linux/dma-mapping.h>

#include <linux/sched.h>
#include <linux/amlogic/meson-secure.h>

#define MESON_SECURE_DEBUG 0
#if MESON_SECURE_DEBUG
#undef pr_fmt
#define pr_fmt(fmt) "meson-secure: " fmt
#define TZDBG(fmt, ...) pr_info(fmt, ##__VA_ARGS__)
#else
#define TZDBG(fmt, ...)
#endif

#define MESON_SECURE_FLAG_REG_OFFS            0x81F0
#define MESON_SECURE_FLAG_VALUE_DISABLED      0x0
#define MESON_SECURE_FLAG_VALUE_ENABLED       0x1
#define MESON_SECURE_FLAG_VALUE_INVALID       0xFFFFFFFF

static uint32_t secure_flag = MESON_SECURE_FLAG_VALUE_INVALID;
static void __iomem *secure_flag_base;

/* check secure status
 *
 * return value:
 *   true: secure enabled
 *  false: secure disabled
 */
bool meson_secure_enabled(void)
{
	bool ret = false;
	struct device_node *np;

	if (secure_flag == MESON_SECURE_FLAG_VALUE_INVALID) {
		np = of_find_compatible_node(NULL, NULL, "amlogic, iomap");
		if (!np) {
			TZDBG("find iomap node fail.");
			return false;
		}

		secure_flag_base = of_iomap(np, 0);
		if (!secure_flag_base) {
			TZDBG("of_iomap error.");
			return false;
		}

		secure_flag = readl_relaxed(secure_flag_base +
				MESON_SECURE_FLAG_REG_OFFS);
	}

	TZDBG("secure_flag: 0x%x\n", secure_flag);
	switch (secure_flag) {
	case MESON_SECURE_FLAG_VALUE_ENABLED:
		ret = true;
		break;
	case MESON_SECURE_FLAG_VALUE_DISABLED:
		ret = false;
		break;
	case MESON_SECURE_FLAG_VALUE_INVALID:
		ret = false;
		break;

	default:
		ret = false;
		break;
	}

	return ret;
}
EXPORT_SYMBOL(meson_secure_enabled);

uint32_t meson_secure_reg_read(uint32_t addr)
{
	uint32_t ret;
	uint32_t paddr;
	int offset;

	offset = 0;//IO_SECBUS_PHY_BASE - IO_SECBUS_BASE;
	paddr = addr + offset;
	ret = meson_smc2(paddr);
	TZDBG("end call smc2, read [0x%x]=%x\n", paddr, ret);

	return ret;
}

uint32_t meson_secure_reg_write(uint32_t addr, uint32_t val)
{
	uint32_t ret;
	uint32_t paddr;
	int offset;

	offset = 0;//IO_SECBUS_PHY_BASE - IO_SECBUS_BASE;
	paddr = addr + offset;
	ret = meson_smc3(paddr, val);
	TZDBG("end call smc3, write [0x%x 0x%x]=%x\n", paddr, val, ret);

	return ret;
}

uint32_t meson_secure_mem_base_start(void)
{
	return meson_smc1(TRUSTZONE_MON_MEM_BASE, 0);
}

uint32_t meson_secure_mem_total_size(void)
{
	return meson_smc1(TRUSTZONE_MON_MEM_TOTAL_SIZE, 0);
}

uint32_t meson_secure_mem_flash_start(void)
{
	return meson_smc1(TRUSTZONE_MON_MEM_FLASH, 0);
}

uint32_t meson_secure_mem_flash_size(void)
{
	return meson_smc1(TRUSTZONE_MON_MEM_FLASH_SIZE, 0);
}

int32_t meson_secure_mem_ge2d_access(uint32_t msec)
{
	int ret = -1;
	struct cpumask org_cpumask;

	cpumask_copy(&org_cpumask, &current->cpus_allowed);
	set_cpus_allowed_ptr(current, cpumask_of(0));
	ret = meson_smc_hal_api(TRUSTZONE_HAL_API_MEMCONFIG_GE2D, msec);
	set_cpus_allowed_ptr(current, &org_cpumask);

	return ret;
}
EXPORT_SYMBOL(meson_secure_jtag_disable);

uint32_t meson_secure_jtag_disable(void)
{
	return meson_smc1(TRUSTZONE_MON_JTAG_DISABLE, 0);
}

uint32_t meson_secure_jtag_apao(void)
{
	return meson_smc1(TRUSTZONE_MON_JTAG_APAO, 0);
}
EXPORT_SYMBOL(meson_secure_jtag_apao);

uint32_t meson_secure_jtag_apee(void)
{
	return meson_smc1(TRUSTZONE_MON_JTAG_APEE, 0);
}
EXPORT_SYMBOL(meson_secure_jtag_apee);

int meson_trustzone_efuse(void *arg)
{
	int ret;
	struct cpumask org_cpumask;

	if (!arg)
		return -1;

	cpumask_copy(&org_cpumask, &current->cpus_allowed);
	set_cpus_allowed_ptr(current, cpumask_of(0));

	ret = meson_smc_hal_api(TRUSTZONE_HAL_API_EFUSE, __pa(arg));

	set_cpus_allowed_ptr(current, &org_cpumask);

	return ret;
}
EXPORT_SYMBOL(meson_trustzone_efuse);
