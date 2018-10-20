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
 * @file "modules/decawave/kalmanFilter.h"
 * @author Thomas Fijen
 * This code implements a Kalman filter on the UWB position of the Parrot Bebop 2. WARNING: The discretised model of the UAV and the kalman gain K were 
 * calculated offline in MATLAB and hardcoded into this file.
 */

#ifndef KALMANFILTER_H
#define KALMANFILTER_H
 
 extern void kalman_filter(float out[6], float X_old[6], float u[2], float x, float y);
 extern void mat_mult_6x6_6x1(float out[6], float in1[36], float in2[6]);
 extern void mat_mult_6x2_2x1(float out[6], float in1[12], float in2[2]);
 extern void mat_add_6x1(float out[6], float in1[6], float in2[6]);
 extern void mat_mult_2x6_6x1(float out[2], float in1[12], float in2[6]);
 extern void mat_subtract_2x1(float out[2], float in1[2], float in2[2]);

#endif

