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
 * @file "modules/decawave/uwb_localisation_and_comms.h"
 * @author Thomas Fijen
 * This function contains the code necessary to localise the UAV, using UWB, as well as the inter-UAV communication
 */

#ifndef UWB_LOCALISATION_AND_COMMS_H
#define UWB_LOCALISATION_AND_COMMS_H

#include "std.h"



/* Structure containing the positions of the other UAVs sent over the UWB*/
struct nodeState {
  uint8_t nodeAddress;
  float x;
  float y;
  /*bool state_updated[UWB_SERIAL_COMM_NODE_STATE_SIZE];*/
};

extern void local_and_comms_init(void);
extern void local_and_comms_periodic(void);
extern void local_and_comms_report(void);
extern void local_and_comms_event(void);

 extern void kalman_filter(float out[6], float X_old[6], float u[2], float x, float y);
 extern void mat_mult_6x6_6x1(float out[6], float in1[36], float in2[6]);
 extern void mat_mult_6x2_2x1(float out[6], float in1[12], float in2[2]);
 extern void mat_add_6x1(float out[6], float in1[6], float in2[6]);
 extern void mat_mult_2x6_6x1(float out[2], float in1[12], float in2[6]);
 extern void mat_subtract_2x1(float out[2], float in1[2], float in2[2]);
 
 extern void getRanges(float ranges[4]);
 extern void getPos_UWB(uint8_t index, float positions[2]);

#endif

