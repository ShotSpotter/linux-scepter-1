/*
 * wm8737-sync.h - header for synchronized wm8737's omap driver
 *
 * Author: Sarah Newman <snewman@shotspotter.com>
 *
 * Copyright (C) 2012 ShotSpotter Inc.
 *
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef _WM8737_SYNC_H
#define _WM8737_SYNC_H

#define MAX_WM8737_SLAVES 3

struct wm8737_audio_data {
	int mcbsp;
};

struct wm8737_sync_platform_data {
	int cntr_rst_gpio;
	int mclk_gate_gpio;
	int audio_mclk;
	wm8737_audio_data *master;
	wm8737_audio_data *slaves;
	int num_slaves;
};

#endif
