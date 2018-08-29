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
 * @file "modules/neural_network/neural_network.c"
 * @author Thomas Fijen
 * This fuction implements the NN control necessary for my persistent surveillance mission. 
 * The NN used is not fully connected and has two outputs, namely; the x and y velocities.
 */

#include "modules/neural_network/neural_network.h"
#include "std.h"
#include "state.h"
#include <stdio.h>
#include <stdlib.h>

#ifndef MS_LENGTH
#define MS_LENGTH 5.0f
#endif

#ifndef MS_BREDTH
#define MS_BREDTH 5.0f
#define 

#ifndef MS_GRID_RES
#define MS_GRID_RES 1.0f
#endif

#ifndef MS_TIME_STEP
#define MS_TIME_STEP 0.5f
#endif

#ifndef MS_GRID_NUM_CELLS
#define MS_GRID_NUM_CELLS 25
#endif

#ifndef MS_NUM_INPUTS
#define MS_NUM_INPUTS 18
#endif

/** Mission Space Parameter structure */
struct MS_Struct {
  uint8_t MS[MS_BREDTH/MS_GRID_RES][MS_LENGTH/MS_GRID_RES];
  

};

void inputs(uint8_t *inNodes){
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
}

void neural_network_init(void) {
  
}

void neural_network_periodic(void) {
  uint8_t inputNodes[MS_NUM_INPUTS]
}


