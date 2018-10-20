/*
 * Copyright (C) 2017 Thomas Fijen
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
 * @file "modules/decawave/kalmanFilter.c"
 * @author Thomas Fijen
 * This code implements a Kalman filter on the UWB position of the Parrot Bebop 2. WARNING: The discretised model of the UAV and the kalman gain K were 
 * calculated offline in MATLAB and hardcoded into this file.
 */


//----------------------------------------------------
/* Added a Kalman Filter to the position. Not sure if this will affect the other filters in the INS...
*  Inputs: Previously Calculated X, inputs u, UWB pos x, UWB pos y
    u = [vel X; vel Y]; X = [accel x, vel x, accel y, vel y, x, y]
*/

#include "kalmanFilter.h"

void kalman_filter(float out[6], float X_old[6], float u[2], float x, float y)
{
  //--Discretised model
  float phi[36] = {0.8942,-2.4404,0,0,0,0,0.0018,0.9976,0,0,0,0,0,0,0.8942,-2.4404,0,0,0,0,0.0018,0.9976,0,0,0,0.002,0,0,1,0,0,0,0,0.002,0,1};
  float gamma[12] = {2.4404,0,0.0024,0,0,2.4404,0,0.0024,0,0,0,0};
  float Cd[12] = {0,0,0,0,1,0,0,0,0,0,0,1};

  float K[12] = {-0.008*1.0e-4,0,0,0,0.001*1.0e-4,-0.0049*1.0e-4,0,0.0001*1.0e-4,0.7365*1.0e-4,-0.009*1.0e-4,-0.009*1.0e-4,0.9060*1.0e-4};
  float X_hat[6] = {0,0,0,0,0,0};

  //--Temporary storage variables
  float temp_6x1[6];
  float temp2_6x1[6];
  float temp_2x1[2];
  float temp2_2x1[2] = {x,y};

  //Determine X_hat from the model 
  mat_mult_6x6_6x1(temp_6x1, phi, X_old);
  mat_mult_6x2_2x1(temp2_6x1, gamma, u);
  mat_add_6x1(X_hat,temp_6x1,temp2_6x1);
    
  //Update the estimate with the Kalman gain
  mat_mult_2x6_6x1(temp_2x1,Cd,X_hat);
  mat_subtract_2x1(temp_2x1,temp2_2x1,temp_2x1);
  mat_mult_6x2_2x1(temp_6x1,K,temp_2x1);
  mat_add_6x1(out,X_hat,temp_6x1);
}

//--Matrix functions for the Kalman filter
void mat_mult_6x6_6x1(float out[6], float in1[36], float in2[6])
{
  out[0] = in1[0]*in2[0] + in1[1]*in2[1] + in1[2]*in2[2] + in1[3]*in2[3] + in1[4]*in2[4] + in1[5]*in2[5];
  out[1] = in1[6]*in2[0] + in1[7]*in2[1] + in1[8]*in2[2] + in1[9]*in2[3] + in1[10]*in2[4] + in1[11]*in2[5];
  out[2] = in1[12]*in2[0] +in1[13]*in2[1] + in1[14]*in2[2] + in1[15]*in2[3] + in1[16]*in2[4] + in1[17]*in2[5];
  out[3] = in1[18]*in2[0] +in1[19]*in2[1] + in1[20]*in2[2] + in1[21]*in2[3] + in1[22]*in2[4] + in1[23]*in2[5];
  out[4] = in1[24]*in2[0] +in1[25]*in2[1] + in1[26]*in2[2] + in1[27]*in2[3] + in1[28]*in2[4] + in1[29]*in2[5];
  out[5] = in1[30]*in2[0] +in1[31]*in2[1] + in1[32]*in2[2] + in1[33]*in2[3] + in1[34]*in2[4] + in1[35]*in2[5];
}
void mat_mult_6x2_2x1(float out[6], float in1[12], float in2[2])
{
  out[0] = in1[0]*in2[0] + in1[1]*in2[1];
  out[1] = in1[2]*in2[0] + in1[3]*in2[1];
  out[2] = in1[4]*in2[0] + in1[5]*in2[1];
  out[3] = in1[6]*in2[0] + in1[7]*in2[1];
  out[4] = in1[8]*in2[0] + in1[9]*in2[1];
  out[5] = in1[10]*in2[0] + in1[11]*in2[1];
}
void mat_add_6x1(float out[6], float in1[6], float in2[6])
{
  out[0] = in1[0]+in2[0];
  out[1] = in1[1]+in2[1];
  out[2] = in1[2]+in2[2];
  out[3] = in1[3]+in2[3];
  out[4] = in1[4]+in2[4];
  out[5] = in1[5]+in2[5];
}
void mat_mult_2x6_6x1(float out[2], float in1[12], float in2[6])
{
  out[0] = in1[0]*in2[0] +in1[1]*in2[1] + in1[2]*in2[2] + in1[3]*in2[3] + in1[4]*in2[4] + in1[5]*in2[5];
  out[1] = in1[6]*in2[0] +in1[7]*in2[1] + in1[8]*in2[2] + in1[9]*in2[3] + in1[10]*in2[4] + in1[11]*in2[5];
}
void mat_subtract_2x1(float out[2], float in1[2], float in2[2])
{
  out[0] = in1[0]-in2[0];
  out[1] = in1[1]-in2[1];
}