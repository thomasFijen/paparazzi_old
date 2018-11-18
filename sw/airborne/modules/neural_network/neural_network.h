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
 * @file "modules/neural_network/neural_network.h"
 * @author Thomas Fijen
 * This fuction implements the NN control necessary for my persistent surveillance mission.
 * The NN used is not fully connected and has two outputs, namely; the x and y velocities.
 */

#ifndef MS_NUM_OUTPUTS
#define MS_NUM_OUTPUTS 2
#endif

#ifndef NEURAL_NETWORK_H
#define NEURAL_NETWORK_H

#include "std.h"

float activationFunction(float x);
void ageMS(void);
void homing(float xPos, float yPos);
void avoid(void); 
void behaviourTree(void);
extern void runSurviellance(void); //Use this if calcNN is set as a periodic function. 
extern bool printMS(void);
extern void printNode(void);
extern bool outArea(void);
extern bool testDistance(void);
 extern void calcInputs(void); 
 // extern bool calcNN(uint8_t wp_id); 
 extern void calcNN(void); 
 extern void neural_network_init(void);
// extern void neural_network_periodic(void);

#endif

