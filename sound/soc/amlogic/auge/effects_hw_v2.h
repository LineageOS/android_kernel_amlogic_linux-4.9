/*
 * sound/soc/amlogic/auge/effects_hw_v2.h
 *
 * Copyright (C) 2018 Amlogic, Inc. All rights reserved.
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
#ifndef __EFFECTS_HW_V2_H__
#define __EFFECTS_HW_V2_H__
#include <linux/types.h>
#include "ddr_mngr.h"

enum {
	VERSION1 = 0,
	VERSION2,
	VERSION3,
	VERSION4
};

void aed_init_ram_coeff(int version, int add, int len, unsigned int *params);
void aed_set_ram_coeff(int version, int add, int len, unsigned int *params);
void aed_get_ram_coeff(int add, int len, unsigned int *params);
void aed_set_multiband_drc_coeff(int band, unsigned int *params);
void aed_get_multiband_drc_coeff(int band, unsigned int *params);
void aed_set_fullband_drc_coeff(int group, unsigned int *params);
void aed_get_fullband_drc_coeff(int len, unsigned int *params);
void aed_dc_enable(bool enable);
void aed_nd_enable(bool enable);
void aed_eq_enable(int idx, bool enable);
void aed_set_fullband_drc_enable(bool enable);
void aed_set_multiband_drc_enable(bool enable);
void aed_set_volume(unsigned int master_volume,
		unsigned int Lch_vol, unsigned int Rch_vol);
void aed_set_lane_and_channels(int lane_mask, int ch_mask);
void aed_set_lane_and_channels_v3(int lane_mask, int ch_mask);
void aed_set_ctrl(bool enable, int sel,
		enum frddr_dest module, int offset);
void aed_set_format(int msb, enum ddr_types frddr_type,
		enum ddr_num source, int offset);
void aed_enable(bool enable);
void aed_set_mixer_params(void);
void aed_eq_taps(unsigned int eq1_taps);
void aed_set_multiband_drc_param(void);
void aed_set_fullband_drc_param(int tap);
void aed_module_reset(int offset);

#endif
