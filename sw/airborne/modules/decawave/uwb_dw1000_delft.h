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
 * @file "modules/decawave/uwb_dw1000_delft.h"
 * @author Thomas Fijen
 * This is a driver to get ranging data from the Decawave DW1000 connected to Arduino. The format of the data recieved over the serial link is: [(byte) START_MARKER, (byte) anchorID, (float) range]. This module is based off of the module 'dw1000_arduino' created by Gautier Hattenberger <gautier.hattenberger@enac.fr>
 */

#ifndef UWB_DW1000_DELFT_H
#define UWB_DW1000_DELFT_H

 extern void uwb_dw1000_init(void);
 extern void uwb_dw1000_periodic(void);
 extern void uwb_dw1000_resetheading(void);
 extern void uwb_dw1000_report(void);
 extern void uwb_dw1000_event(void);

#endif

