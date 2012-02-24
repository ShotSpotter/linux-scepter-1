/*
 * wm8737.h
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

#ifndef WM8737_H_
#define WM8737_H_

#define MAX_WM8737_ID 3
#define MAX_WM8737_CNT (MAX_WM8737_ID+1)

struct wm8737_platform_data {
	int id;
};


#endif /* WM8737_H_ */
