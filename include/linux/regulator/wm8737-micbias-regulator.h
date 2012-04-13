/*
 * wm8737-micbias-regulator.h
 *
 * Copyright 2012 ShotSpotter Inc.
 *
 * Author: Sarah Newman <snewman@shotspotter.com>
 *
 * Based off of fixed.h by Mark Brown
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 */

#ifndef __REGULATOR_WM8737_MICBIAS_H
#define __REGULATOR_WM8737_MICBIAS_H

struct regulator_init_data;

struct wm8737_micbias_config {
	int avdd_mV;
	int mvdd_mV;
	struct regulator_init_data *init_data;
};

#endif
