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

struct wm8737_omap_data {
	unsigned int wm8737_id;
	unsigned int mcbsp_id;
	unsigned int codec_dai_audio_fmt;
	unsigned int cpu_dai_audio_fmt;
};

struct wm8737_sync_platform_data {
	int sample_cnt_rst_gpio;
	int mclk_en_gpio;
	unsigned int mclk;
	struct wm8737_omap_data *master;
	struct wm8737_omap_data *slaves;
	unsigned int num_slaves;
};

#endif
