/*
 * Copyright (C) Thomas Fijen
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/decawave/gps_uwb.h"
 * @author Thomas Fijen
 * Use this file so that the UWB can be simulated as a primary GPS structure
 */

#ifndef GPS_UWB_H
#define GPS_UWB_H

#include "std.h"
#include "generated/airframe.h"
#include "subsystems/gps.h"


#ifndef PRIMARY_GPS
#define PRIMARY_GPS GPS_UWB
#endif

extern struct GpsState gps_uwb;

extern void gps_uwb_init(void);
extern void gps_uwb_event(void);
#define gps_uwb_periodic_check() gps_periodic_check(&gps_uwb)
 
extern void update_uwb(uint32_t now_ts, struct GpsState *gps_dw1000);
 

#endif

