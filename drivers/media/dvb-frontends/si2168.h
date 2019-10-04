/*
 * Silicon Labs Si2168 DVB-T/T2/C demodulator driver
 *
 * Copyright (C) 2014 Antti Palosaari <crope@iki.fi>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#ifndef SI2168_H
#define SI2168_H

#include <linux/dvb/frontend.h>
/**
 * struct si2168_config - configuration parameters for si2168
 *
 * @fe:
 *	frontend returned by driver
 * @i2c_adapter:
 *	tuner I2C adapter returned by driver
 * @ts_mode:
 *	Transport Stream mode. Can be:
 *	- %SI2168_TS_PARALLEL
 *	- %SI2168_TS_SERIAL
 *	- %SI2168_TS_TRISTATE
 *	- %SI2168_TS_CLK_MANUAL
 * @ts_clock_inv:
 *	TS clock inverted
 * @ts_clock_gapped:
 *	TS clock gapped
 * @spectral_inversion:
 *	Inverted spectrum
 *
 * Note:
 *	The I2C address of this demod is 0x64.
 */
struct si2168_config {
	struct dvb_frontend **fe;
	struct i2c_adapter **i2c_adapter;

#define SI2168_TS_PARALLEL	0x06
#define SI2168_TS_SERIAL	0x03
#define SI2168_TS_TRISTATE	0x00
#define SI2168_TS_CLK_MANUAL	0x20
	u8 ts_mode;

	/* Flags */
	unsigned int ts_clock_inv:1;
	unsigned int ts_clock_gapped:1;
	unsigned int spectral_inversion:1;
};

#endif
