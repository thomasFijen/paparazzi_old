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
 * @file "modules/decawave/multilateration_nlls.c"
 * @author Thomas Fijen
 * This code implements a multilateration algorithm using a nonlinear least Squares approach for 4 anchors. Note for this module to
 * work, the A matrix must be non-singular. To achieve this, the anchors should not all lie on the same z plane. 
 */
 
#ifndef MULTILATERATION_NLLS_H
#define MULTILATERATION_NLLS_H

#include "std.h"
#include "math/pprz_geodetic_float.h"

/** Anchor structure */
struct Anchor {
  float distance;       ///< last measured distance
  float time;           ///< time of the last received data
  struct EnuCoor_f pos; ///< position of the anchor
  uint16_t id;          ///< anchor ID
};

extern int nonLinLS_compute(struct Anchor *anchors, struct EnuCoor_f *pos, struct EnuCoor_f *oldPos);
void transpose_3x4(float out[12], float mat_in[12]);
void inverse_3x3(float out[9], float matIn[9]);
void mat_adjoint_3d(float out[9], float matIn[9]);
float det_mat_3d(float matIn[9]);
void mat_multi_3x4_4x3(float mat_out[9], float mat_inA[12], float mat_inB[12]);
void mat_multi_3x3_3x4(float mat_out[12], float mat_inA[9], float mat_inB[12]);
void mat_vec_multi_4d(float out[3], float matA[12], float vecB[4]);
void vec_subtract_3d(float out[3], float vecA[3], float vecB[3]);
void jacobian(float out[12], float X[3], struct Anchor *anchors);
void costFunction(float F[4], float X[3], struct Anchor *anchors);

#endif
