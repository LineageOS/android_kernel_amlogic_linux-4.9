// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018, Richtek Technology Corporation
 *
 * Richtek RT1711H Type-C Chip Driver
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/usb/tcpm.h>
#include <linux/regmap.h>
#include "tcpci.h"

#define RT1711H_VID		0x29CF
#define RT1711H_PID		0x1711
#define RT1711H_DID_D		0x2173

#define RT1711H_RTCTRL8		0x9B

/* Autoidle timeout = (tout * 2 + 1) * 6.4ms */
#define RT1711H_RTCTRL8_SET(ck300, ship_off, auto_idle, tout) \
			    (((ck300) << 7) | ((ship_off) << 5) | \
			    ((auto_idle) << 3) | ((tout) & 0x07))

#define RT1711H_RTCTRL11	0x9E

/* I2C timeout = (tout + 1) * 12.5ms */
#define RT1711H_RTCTRL11_SET(en, tout) \
			     (((en) << 7) | ((tout) & 0x0F))

#define RT1711H_RTRXDZSEL	0x93
#define RT1711H_RTCTRL13	0xA0
#define RT1711H_RTCTRL14	0xA1
#define RT1711H_RTCTRL15	0xA2
#define RT1711H_RTCTRL16	0xA3
#define RT1711H_RTRXDZEN	0xAF

/* Port controller BIST TESTDATA mode support */
#define RT1711H_CTRL_BIST_EN	BIT(1)

/* 1: use gpios element to look up IRQ number. 0: use interrupts element. */
#define USE_GPIOS_INTR 1

struct rt1711h_chip {
	struct tcpci_data data;
	struct tcpci *tcpci;
	struct device *dev;
	u16 chip_id;
	u8 deadzone_enable;
	u8 deadzone_select;
	u8 en_bist;
#if USE_GPIOS_INTR
	int gpio_int_n_irq;
#endif
};

static int rt1711h_read16(struct rt1711h_chip *chip, unsigned int reg, u16 *val)
{
	return regmap_raw_read(chip->data.regmap, reg, val, sizeof(u16));
}

static int rt1711h_write16(struct rt1711h_chip *chip, unsigned int reg, u16 val)
{
	return regmap_raw_write(chip->data.regmap, reg, &val, sizeof(u16));
}

static int rt1711h_read8(struct rt1711h_chip *chip, unsigned int reg, u8 *val)
{
	return regmap_raw_read(chip->data.regmap, reg, val, sizeof(u8));
}

static int rt1711h_write8(struct rt1711h_chip *chip, unsigned int reg, u8 val)
{
	return regmap_raw_write(chip->data.regmap, reg, &val, sizeof(u8));
}

static const struct regmap_config rt1711h_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0xFF, /* 0x80 .. 0xFF are vendor defined */
};

static struct rt1711h_chip *tdata_to_rt1711h(struct tcpci_data *tdata)
{
	return container_of(tdata, struct rt1711h_chip, data);
}

static int rt1711h_init(struct tcpci *tcpci, struct tcpci_data *tdata)
{
	int ret;
	struct rt1711h_chip *chip = tdata_to_rt1711h(tdata);

	/* Clear BIST TESTDATA mode */
	chip->en_bist = 0;

	/* CK 300K from 320K, shipping off, auto_idle enable, tout = 32ms */
	ret = rt1711h_write8(chip, RT1711H_RTCTRL8,
			     RT1711H_RTCTRL8_SET(0, 1, 1, 2));
	if (ret < 0)
		return ret;

	/* I2C reset : (val + 1) * 12.5ms */
	ret = rt1711h_write8(chip, RT1711H_RTCTRL11,
			     RT1711H_RTCTRL11_SET(1, 0x0F));
	if (ret < 0)
		return ret;

	/* tTCPCfilter : (26.7 * val) us */
	ret = rt1711h_write8(chip, RT1711H_RTCTRL14, 0x0F);
	if (ret < 0)
		return ret;

	/*  tDRP : (51.2 + 6.4 * val) ms */
	ret = rt1711h_write8(chip, RT1711H_RTCTRL15, 0x04);
	if (ret < 0)
		return ret;

	/* dcSRC.DRP : 33% */
	return rt1711h_write16(chip, RT1711H_RTCTRL16, 330);
}

static enum typec_cc_status rt1711h_reg_to_cc(unsigned int cc, bool sink)
{
	switch (cc) {
	case 1:
		return sink ? TYPEC_CC_RP_DEF : TYPEC_CC_RD;
	case 2:
		return sink ? TYPEC_CC_RP_1_5 : TYPEC_CC_RA;
	case 3:
		return sink ? TYPEC_CC_RP_3_0 : TYPEC_CC_OPEN;
	default:
		return TYPEC_CC_OPEN;
	}
}

static int rt1711h_get_cc(struct tcpci *tcpci, struct tcpci_data *tdata,
			  enum typec_cc_status *cc1, enum typec_cc_status *cc2)
{
	struct rt1711h_chip *chip = tdata_to_rt1711h(tdata);
	u8 reg, cc1_state, cc2_state;
	bool cc1_sink, cc2_sink, tcpc_presenting_rd;
	enum typec_cc_status cc;
	u8 sel;
	int ret;

	/* Read CC1 & CC2 ROLE */
	ret = rt1711h_read8(chip, TCPC_ROLE_CTRL, &reg);
	if (ret < 0)
		return ret;

	cc1_sink = ((reg >> TCPC_ROLE_CTRL_CC1_SHIFT) &
	     TCPC_ROLE_CTRL_CC1_MASK) == TCPC_ROLE_CTRL_CC_RD;
	cc2_sink = ((reg >> TCPC_ROLE_CTRL_CC2_SHIFT) &
	     TCPC_ROLE_CTRL_CC2_MASK) == TCPC_ROLE_CTRL_CC_RD;
	/* Read CC STATUS */
	ret = rt1711h_read8(chip, TCPC_CC_STATUS, &reg);
	if (ret < 0)
		return ret;

	/* When auto toggling, BIT(4) is set on presenting Rd */
	tcpc_presenting_rd = !!(reg & TCPC_CC_STATUS_TERM);
	cc1_sink |= tcpc_presenting_rd;
	cc2_sink |= tcpc_presenting_rd;
	cc1_state = (reg >> TCPC_CC_STATUS_CC1_SHIFT) & TCPC_CC_STATUS_CC1_MASK;
	cc2_state = (reg >> TCPC_CC_STATUS_CC2_SHIFT) & TCPC_CC_STATUS_CC2_MASK;

	*cc1 = rt1711h_reg_to_cc(cc1_state, cc1_sink);
	*cc2 = rt1711h_reg_to_cc(cc2_state, cc2_sink);

	/* Set RT1711H deadzone */
	cc = (*cc1 == TYPEC_CC_OPEN) ? *cc2 : *cc1;

	if (cc == TYPEC_CC_RP_DEF) {
		if (chip->deadzone_enable != 0) {
			chip->deadzone_enable = 0;
			rt1711h_write8(chip, RT1711H_RTRXDZEN, 0);
		}
		if (chip->deadzone_select != 0x81) {
			chip->deadzone_select = 0x81;
			rt1711h_write8(chip, RT1711H_RTRXDZSEL, 0x81);
		}
	} else {
		sel = (chip->chip_id >= RT1711H_DID_D) ? 0x81 : 0x80;
		if (chip->deadzone_enable != 1) {
			chip->deadzone_enable = 1;
			rt1711h_write8(chip, RT1711H_RTRXDZEN, 1);
		}
		if (chip->deadzone_select != sel) {
			chip->deadzone_select = sel;
			rt1711h_write8(chip, RT1711H_RTRXDZSEL, sel);
		}
	}

	return 0;
}

static int rt1711h_set_polarity(struct tcpci *tcpci, struct tcpci_data *tdata,
				enum typec_cc_polarity polarity)
{
	struct rt1711h_chip *chip = tdata_to_rt1711h(tdata);
	u8 reg;

	/* Polarity */
	reg = (polarity == TYPEC_POLARITY_CC2) ?
	      TCPC_TCPC_CTRL_ORIENTATION : 0;
	/* HW BIST feature */
	reg |= chip->en_bist ? RT1711H_CTRL_BIST_EN : 0;

	return rt1711h_write8(chip, TCPC_TCPC_CTRL, reg);
}

static int rt1711h_set_bist(struct tcpci *tcpci, struct tcpci_data *tdata,
			    bool enable)
{
	struct rt1711h_chip *chip = tdata_to_rt1711h(tdata);
	int ret;
	u8 reg;

	ret = rt1711h_read8(chip, TCPC_TCPC_CTRL, &reg);
	if (ret)
		return ret;

	if (enable)
		reg |= RT1711H_CTRL_BIST_EN;
	else
		reg &= ~RT1711H_CTRL_BIST_EN;

	ret = rt1711h_write8(chip, TCPC_TCPC_CTRL, reg);
	if (!ret)
		chip->en_bist = enable;

	return ret;
}

static int rt1711h_set_vconn(struct tcpci *tcpci, struct tcpci_data *tdata,
			     bool enable)
{
	struct rt1711h_chip *chip = tdata_to_rt1711h(tdata);

	return rt1711h_write8(chip, RT1711H_RTCTRL8,
			      RT1711H_RTCTRL8_SET(0, 1, !enable, 2));
}

static int rt1711h_start_drp_toggling(struct tcpci *tcpci,
				      struct tcpci_data *tdata,
				      enum typec_cc_status cc)
{
	struct rt1711h_chip *chip = tdata_to_rt1711h(tdata);
	int ret;
	unsigned int reg = 0;

	switch (cc) {
	default:
	case TYPEC_CC_RP_DEF:
		reg |= (TCPC_ROLE_CTRL_RP_VAL_DEF <<
			TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	case TYPEC_CC_RP_1_5:
		reg |= (TCPC_ROLE_CTRL_RP_VAL_1_5 <<
			TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	case TYPEC_CC_RP_3_0:
		reg |= (TCPC_ROLE_CTRL_RP_VAL_3_0 <<
			TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	}

	if (cc == TYPEC_CC_RD)
		reg |= (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC1_SHIFT) |
			   (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC2_SHIFT);
	else
		reg |= (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC1_SHIFT) |
			   (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC2_SHIFT);

	ret = rt1711h_write8(chip, TCPC_ROLE_CTRL, reg);
	if (ret < 0)
		return ret;
	usleep_range(500, 1000);

	return 0;
}

static irqreturn_t rt1711h_irq(int irq, void *dev_id)
{
	int ret;
	u16 alert;
	u8 status;
	struct rt1711h_chip *chip = dev_id;

	if (!chip->tcpci)
		return IRQ_HANDLED;

	ret = rt1711h_read16(chip, TCPC_ALERT, &alert);
	if (ret < 0)
		goto out;

	if (alert & TCPC_ALERT_CC_STATUS) {
		ret = rt1711h_read8(chip, TCPC_CC_STATUS, &status);
		if (ret < 0)
			goto out;
		/* Clear cc change event triggered by starting toggling */
		if (status & TCPC_CC_STATUS_TOGGLING)
			rt1711h_write8(chip, TCPC_ALERT, TCPC_ALERT_CC_STATUS);
	}

out:
	return tcpci_irq(chip->tcpci);
}

static int rt1711h_init_alert(struct rt1711h_chip *chip,
			      struct i2c_client *client)
{
	int ret;
#if USE_GPIOS_INTR
	struct gpio_desc *desc;
	int id_irqnr;
#endif

	/* Disable chip interrupts before requesting irq */
	ret = rt1711h_write16(chip, TCPC_ALERT_MASK, 0);
	if (ret < 0)
		return ret;

#if USE_GPIOS_INTR
	desc = gpiod_get_index(chip->dev, NULL, 0, GPIOD_IN);
	if (IS_ERR(desc)) {
		pr_err("fail to get id-gpioirq\n");
		return -1;
	}
	id_irqnr = gpiod_to_irq(desc);

	ret = devm_request_threaded_irq(chip->dev, id_irqnr, NULL,
					rt1711h_irq,
					IRQF_ONESHOT | IRQF_TRIGGER_LOW,
					"rt1711h_interrupt_int_n", chip);
	if (ret) {
		pr_err("failed to request ret=%d, id_irqnr=%d\n",
			ret, id_irqnr);
		return ret;
	}
	chip->gpio_int_n_irq = id_irqnr;
	pr_info("<%s> ok\n", __func__);
	enable_irq_wake(id_irqnr);
	return 0;
#else
	ret = devm_request_threaded_irq(chip->dev, client->irq, NULL,
					rt1711h_irq,
					IRQF_ONESHOT | IRQF_TRIGGER_LOW,
					dev_name(chip->dev), chip);
	if (ret < 0)
		return ret;
	enable_irq_wake(client->irq);
	return 0;
#endif
}

static int rt1711h_sw_reset(struct rt1711h_chip *chip)
{
	int ret;

	ret = rt1711h_write8(chip, RT1711H_RTCTRL13, 0x01);
	if (ret < 0)
		return ret;

	usleep_range(1000, 2000);
	return 0;
}

static int rt1711h_check_revision(struct i2c_client *i2c)
{
	int ret;

	ret = i2c_smbus_read_word_data(i2c, TCPC_VENDOR_ID);
	if (ret < 0)
		return ret;
	if (ret != RT1711H_VID) {
		dev_err(&i2c->dev, "vid is not correct, 0x%04x\n", ret);
		return -ENODEV;
	}
	ret = i2c_smbus_read_word_data(i2c, TCPC_PRODUCT_ID);
	if (ret < 0)
		return ret;
	if (ret != RT1711H_PID) {
		dev_err(&i2c->dev, "pid is not correct, 0x%04x\n", ret);
		return -ENODEV;
	}
	return 0;
}

static int rt1711h_probe(struct i2c_client *client,
			 const struct i2c_device_id *i2c_id)
{
	int ret;
	struct rt1711h_chip *chip;
	u16 chip_id;

	ret = rt1711h_check_revision(client);
	if (ret < 0) {
		dev_err(&client->dev, "check vid/pid fail\n");
		return ret;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->data.regmap = devm_regmap_init_i2c(client,
						 &rt1711h_regmap_config);
	if (IS_ERR(chip->data.regmap))
		return PTR_ERR(chip->data.regmap);

	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);

	ret = rt1711h_sw_reset(chip);
	if (ret < 0)
		return ret;

	/* Get device id and initialize deadzone */
	ret = rt1711h_read16(chip, TCPC_BCD_DEV, &chip_id);
	if (ret < 0)
		return ret;
	chip->chip_id = chip_id;
	ret = rt1711h_read8(chip, RT1711H_RTRXDZEN, &chip->deadzone_enable);
	if (ret < 0)
		return ret;
	ret = rt1711h_read8(chip, RT1711H_RTRXDZSEL, &chip->deadzone_select);
	if (ret < 0)
		return ret;

	ret = rt1711h_init_alert(chip, client);
	if (ret < 0)
		return ret;

	chip->data.init = rt1711h_init;
	chip->data.get_cc = rt1711h_get_cc;
	chip->data.set_bist = rt1711h_set_bist;
	chip->data.set_polarity = rt1711h_set_polarity;
	chip->data.set_vconn = rt1711h_set_vconn;
	chip->data.start_drp_toggling = rt1711h_start_drp_toggling;
	chip->tcpci = tcpci_register_port(chip->dev, &chip->data);
	if (IS_ERR_OR_NULL(chip->tcpci))
		return PTR_ERR(chip->tcpci);

	return 0;
}

static int rt1711h_remove(struct i2c_client *client)
{
	struct rt1711h_chip *chip = i2c_get_clientdata(client);

	tcpci_unregister_port(chip->tcpci);
	return 0;
}

static const struct i2c_device_id rt1711h_id[] = {
	{ "rt1711h", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rt1711h_id);

#ifdef CONFIG_OF
static const struct of_device_id rt1711h_of_match[] = {
	{ .compatible = "richtek,rt1711h", },
	{},
};
MODULE_DEVICE_TABLE(of, rt1711h_of_match);
#endif

static struct i2c_driver rt1711h_i2c_driver = {
	.driver = {
		.name = "rt1711h",
		.of_match_table = of_match_ptr(rt1711h_of_match),
	},
	.probe = rt1711h_probe,
	.remove = rt1711h_remove,
	.id_table = rt1711h_id,
};
module_i2c_driver(rt1711h_i2c_driver);

MODULE_AUTHOR("ShuFan Lee <shufan_lee@richtek.com>");
MODULE_DESCRIPTION("RT1711H USB Type-C Port Controller Interface Driver");
MODULE_LICENSE("GPL");
