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
 * This code implements a multilateration algorithm using a least Squares approach for a minimum of 4 anchors. Note for this module to
 * work, the A matrix must be non-singular. To achieve this, the anchors should not all lie on the same z plane. 
 * For more information see: Norrdine, A., 2012, November. An algebraic solution to the multilateration problem. In Proceedings of the 15th
 * International Conference on Indoor Positioning and Indoor Navigation, Sydney, Australia (Vol. 1315).
 */
 
#include "multilateration_ls.h"
#include "math/pprz_algebra_float.h"
 
 // base original locations
//static float P[4][3];
static float circ[4];
static float L[16];
//bool init_failed;

 
 int multilateration_init(struct Anchor *anchors)
 {
	//init_failed = false;
	float A_matrix[16], A_trans[16], A_trans_A[16], temp[16];

	for (int i = 0; i < 4; i++) {
//		P[i][0] = anchors[i].pos.x;
//		P[i][1] = anchors[i].pos.y;
//		P[i][2] = anchors[i].pos.z;
		
		circ[i] = anchors[i].pos.x*anchors[i].pos.x + anchors[i].pos.y*anchors[i].pos.y + anchors[i].pos.z*anchors[i].pos.z;
		A_matrix[4*i] = 1;
		A_matrix[4*i+1] =-2*anchors[i].pos.x;
		A_matrix[4*i+2] =-2*anchors[i].pos.y;
		A_matrix[4*i+3] =-2*anchors[i].pos.z;
	}
	// finding L = inv(A^T * A)*A^T
	
	/*if (float_mat_det_4d(A_matrix) != 0)
	{*/
		mat_transpose_4d(A_trans, A_matrix);
		mat_multiplication_4d(A_trans_A,A_trans,A_matrix);

		float_mat_inv_4d(temp,A_trans_A);

		mat_multiplication_4d(L,temp,A_trans);
/*	}
	else
	{
		init_failed = true;
	}*/
	
  
	return 0;
 }

 /*
  * Computes the position based on least squares
  * x_hat = inv(A^T*A)*A^T*b = L*b
  */
 int multilateration_compute(struct Anchor *anchors, struct EnuCoor_f *pos)
 {
	float b[4], x_hat[4];

	// Determine the b vector
	for (int i = 0; i < 4; i++)
	{
		b[i] = anchors[i].distance*anchors[i].distance - circ[i];
	}

	// compute the least squares solution. This assumes same covariance
	mat_multiplication_4d_vect(x_hat,L,b);

	//Store the x, y and z positions
	pos->x = x_hat[1];
	pos->y = x_hat[2];
	pos->z = x_hat[3];

 	return 0;
 }
 
 /*
  * Transpose of a 4x4 matrix
  */
 void mat_transpose_4d(float invOut[16], float mat_in[16])
 {
	 for (int i=0; i < 4; i++)
	 {
		 invOut[4*i] = mat_in[i];
		 invOut[4*i+1] = mat_in[i+4];
		 invOut[4*i+2] = mat_in[i+8];
		 invOut[4*i+3] = mat_in[i+12];
	 }
	 
 }

 /*
  * Multiplication of two  4x4 matrices
  */
 void mat_multiplication_4d(float mat_out[16], float mat_inA[16], float mat_inB[16])
 {
	 for (int i = 0; i < 4; i++)
	 {
		 mat_out[i] = mat_inA[0]*mat_inB[i] + mat_inA[1]*mat_inB[i+4] + mat_inA[2]*mat_inB[i+8] + mat_inA[3]*mat_inB[i+12];
		 mat_out[i+4] = mat_inA[4]*mat_inB[i] + mat_inA[5]*mat_inB[i+4] + mat_inA[6]*mat_inB[i+8] + mat_inA[7]*mat_inB[i+12];
		 mat_out[i+8] = mat_inA[8]*mat_inB[i] + mat_inA[9]*mat_inB[i+4] + mat_inA[10]*mat_inB[i+8] + mat_inA[11]*mat_inB[i+12];
		 mat_out[i+12] = mat_inA[12]*mat_inB[i] + mat_inA[13]*mat_inB[i+4] + mat_inA[14]*mat_inB[i+8] + mat_inA[15]*mat_inB[i+12];
	 }
	 
 }

/*
 * Matrix multiplication of a 4x4 matrix with a vector
 */
void mat_multiplication_4d_vect(float mat_out[4], float mat_inA[16], float mat_inB[4])
{
	mat_out[0] = mat_inA[0]*mat_inB[0] + mat_inA[1]*mat_inB[1] + mat_inA[2]*mat_inB[2] + mat_inA[3]*mat_inB[3];
	mat_out[1] = mat_inA[4]*mat_inB[0] + mat_inA[5]*mat_inB[1] + mat_inA[6]*mat_inB[2] + mat_inA[7]*mat_inB[3];
	mat_out[2] = mat_inA[8]*mat_inB[0] + mat_inA[9]*mat_inB[1] + mat_inA[10]*mat_inB[2] + mat_inA[11]*mat_inB[3];
	mat_out[3] = mat_inA[12]*mat_inB[0] + mat_inA[13]*mat_inB[1] + mat_inA[14]*mat_inB[2] + mat_inA[15]*mat_inB[3];

	
}
 
