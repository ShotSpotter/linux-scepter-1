/*
 * fixed.h
 *
 * Copyright 2008 Wolfson Microelectronics PLC.
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * Copyright (c) 2009 Nokia Corporation
 * Roger Quadros <ext-roger.quadros@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 */

#ifndef __REGULATOR_WM8737_MVDD_H
#define __REGULATOR_WM8737_MVDD_H

struct regulator_init_data;

struct wm8737_mvdd_config {
	int avdd_mV;
	struct regulator_init_data *init_data;
};

#endif
