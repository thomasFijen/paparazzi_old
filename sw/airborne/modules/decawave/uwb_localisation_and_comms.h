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
 * This function contains the code necessary to localise the UAV, using UWB, as well 
 * as the inter-UAV communication
 */

#ifndef UWB_LOCALISATION_AND_COMMS_H
#define UWB_LOCALISATION_AND_COMMS_H

#include "std.h"
#include "modules/decawave/kalmanFilter.h"

/* Structure containing the positions of the other UAVs sent over the UWB*/
struct nodeState {
  uint8_t nodeAddress;
  float x;
  float y;
  float d;
  /*bool state_updated[UWB_SERIAL_COMM_NODE_STATE_SIZE];*/
};

extern void local_and_comms_init(void);
extern void local_and_comms_periodic(void);
extern void local_and_comms_report(void);
extern void local_and_comms_event(void);

 extern void commandSpeed(float u_command[2]);
 extern void getCommandSpeed(float u_command[2]);
 extern void getRanges(float ranges[4]);
 extern void getPos_UWB(uint8_t index, float positions[2]);
 extern void use_UWB_position(void);

#endif

