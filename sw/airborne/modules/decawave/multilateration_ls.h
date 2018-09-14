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
 * @file "modules/decawave/multilateration.c"
 * @author Thomas Fijen
 * This code implements a multilateration algorithm using a least Squares approach. For more information see: Norrdine, A., 2012, November.  
 * An algebraic solution to the multilateration problem. In Proceedings of the 15th International Conference on Indoor Positioning and
 * Indoor Navigation, Sydney, Australia (Vol. 1315).
 */
 
#ifndef MULTILATERATION_LS_H
#define MULTILATERATION_LS_H

#include "std.h"
#include "math/pprz_geodetic_float.h"

/** Anchor structure */
struct Anchor {
  float distance;       ///< last measured distance
  float time;           ///< time of the last received data
  struct EnuCoor_f pos; ///< position of the anchor
  uint16_t id;          ///< anchor ID
};


/*extern int multilateration_init(struct Anchor *anchors);*/
extern int multilateration_compute(struct Anchor *anchors, struct EnuCoor_f *pos);

extern void mat_transpose_4d(float invOut[16], float mat_in[16]);
extern void mat_multiplication_4d(float mat_out[16], float mat_inA[16], float mat_inB[16]);
extern void mat_multiplication_4d_vect(float mat_out[4], float mat_inA[16], float mat_inB[4]);

#endif
