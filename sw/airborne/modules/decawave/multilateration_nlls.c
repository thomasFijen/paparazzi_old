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
 
 #define TOL 0.01
 #define MAXITT 20
 
 #include "multilateration_nlls.h"
 #include "math/pprz_algebra_float.h"


/* Filter parameters */
static uint8_t count_MAF = 0;
static float aveX[5] = {0.f,0.f,0.f,0.f,0.f};
static float aveY[5] = {0.f,0.f,0.f,0.f,0.f};
/*static float aveZ[5] = {0.f,0.f,0.f,0.f,0.f};*/
 
 /*
 *   This performs Gauss Newton optimisation to find the solution to the nonlinear least squares problem 
 */
 int nonLinLS_compute(struct Anchor *anchors, struct EnuCoor_f *pos, struct EnuCoor_f *oldPos) {
    uint8_t count = 0;    //Counts the number of iterations
    float err = 1;
    float x0[3] = {oldPos->x, oldPos->y, oldPos->z};
    float jacob[12];
    float temp[12];
    float temp2[9];
    float inv[9];
    float temp3[3];
    float x_hat[3];
    float F[4];

    while(count < MAXITT && err > TOL)   {
        //x - inv(jacobian * jacobian^T) * jacobian * F
        jacobian(jacob, x0, &anchors[0]);
        transpose_3x4(temp,jacob);
        mat_multi_3x4_4x3(temp2, jacob, temp);
        inverse_3x3(inv,temp2);
        mat_multi_3x3_3x4(temp,inv,jacob);
        costFunction(F,x0, &anchors[0]);
        mat_vec_multi_4d(temp3,temp,F);
        vec_subtract_3d(x_hat,x0,temp3);

        err = sqrtf(temp[0]*temp[0]+temp[1]*temp[1]+temp[2]*temp[2]);
        x0[0] = x_hat[0];
        x0[1] = x_hat[1];
        x0[2] = x_hat[2];
        count++;
    }

    //--Outlier rejection:
    float error = sqrtf((aveX[count_MAF-1]-x_hat[0])*(aveX[count_MAF-1]-x_hat[0])+(aveY[count_MAF-1]-x_hat[1])*(aveY[count_MAF-1]-x_hat[1]));
    if (error > 10)  {
      if(count_MAF == 0)
      {
        aveX[count_MAF] = aveX[4];
        aveY[count_MAF] = aveY[4];
        /*aveZ[count_MAF] = aveZ[4];*/
      }
      else  {
        aveX[count_MAF] = aveX[count_MAF-1];
        aveY[count_MAF] = aveY[count_MAF-1];
        /*aveZ[count_MAF] = aveZ[count_MAF-1];*/
      }
    }
    else  {
      aveX[count_MAF] = x_hat[0];
      aveY[count_MAF] = x_hat[1];
      /*aveZ[count_MAF] = dw->pos.z;*/
    }//--End outlier rejection
     
    //-- Moving average Filter
    float x=0;
    float y=0;
    /*float z=0;*/
      
    for(uint8_t i=0;i<5;i++)   {
      x=x+aveX[i];
      y=y+aveY[i];
      /*z=z+aveZ[i];*/
    }
    x=x/5.0;
    y=y/5.0;
    /*z=z/5;*/
      
    if(count_MAF == 4)  {
      count_MAF = 0;
    }
    else  {
      count_MAF++;
    }
    //-- End of the moving average filters
    
    //Store the x, y and z positions
    pos->x = x;
    pos->y = y;
    pos->z = x_hat[2];

    return 0;
}

 /*
 * Returns the transpose of a 3x4 matrix
 */
 void transpose_3x4(float out[12], float mat_in[12])
 {
    for (uint8_t i=0; i < 3; i++)    {
        out[i] = mat_in[i*4];
        out[i+3] = mat_in[i*4+1];
        out[i+6] = mat_in[i*4+2];
        out[i+9] = mat_in[i*4+3];
    }
 }

 /*
 *    Returns the inverse of a 3x3 matrix
 */
 void inverse_3x3(float out[9], float matIn[9])
 {
     float det;
     float adj[9];
     
     det = det_mat_3d(matIn);
     mat_adjoint_3d(adj,matIn);
    
    if (det == 0)  {
        out[0] = 0;
        out[1] = 0;
        out[2] = 0;
        out[3] = 0;
        out[4] = 0;
        out[5] = 0;
        out[6] = 0;
        out[7] = 0;
        out[8] = 0;
    }
    else   {
        for(uint8_t i=0;i<9;i++)  {
            out[i] = (1/det)*adj[i];
        }
    }
 }
 /*
 * Finds the adjoint of a 3x3 matrix
 */
 void mat_adjoint_3d(float out[9], float matIn[9]) {
    out[0] =   matIn[4]*matIn[8]-matIn[5]*matIn[7];
    out[1] = -(matIn[1]*matIn[8]-matIn[2]*matIn[7]);
    out[2] =   matIn[1]*matIn[5]-matIn[2]*matIn[4];
    out[3] = -(matIn[3]*matIn[8]-matIn[5]*matIn[6]);
    out[4] =   matIn[0]*matIn[8]-matIn[2]*matIn[6];
    out[5] = -(matIn[0]*matIn[5]-matIn[2]*matIn[3]);
    out[6] =   matIn[3]*matIn[7]-matIn[4]*matIn[6];
    out[7] = -(matIn[0]*matIn[7]-matIn[1]*matIn[6]);
    out[8] =   matIn[0]*matIn[4]-matIn[1]*matIn[3];
 }
 /*
 * Gives the determinant of a 3x3 matrix
 */
 float det_mat_3d(float matIn[9]) {
    float out;
    out = matIn[0]*(matIn[4]*matIn[8]-matIn[5]*matIn[7])-matIn[1]*(matIn[3]*matIn[8]-matIn[5]*matIn[6])+matIn[2]*(matIn[3]*matIn[7]-matIn[4]*matIn[6]);
    return out;
 }

 /*
 * Returns the multiplication of matA * matB, where matA is 3x4 and matB is 4x3
 */
 void mat_multi_3x4_4x3(float mat_out[9], float mat_inA[12], float mat_inB[12]) {
    for (uint8_t i = 0; i < 3; i++)    {
        mat_out[i] = mat_inA[0]*mat_inB[i] + mat_inA[1]*mat_inB[i+3] + mat_inA[2]*mat_inB[i+6] + mat_inA[3]*mat_inB[i+9];
        mat_out[i+3] = mat_inA[4]*mat_inB[i] + mat_inA[5]*mat_inB[i+3] + mat_inA[6]*mat_inB[i+6] + mat_inA[7]*mat_inB[i+9];
        mat_out[i+6] = mat_inA[8]*mat_inB[i] + mat_inA[9]*mat_inB[i+3] + mat_inA[10]*mat_inB[i+6] + mat_inA[11]*mat_inB[i+9];
    }
 }
  /*
 * Returns the multiplication of matA * matB, where matA is 3x3 and matB is 3x4
 */
 void mat_multi_3x3_3x4(float mat_out[12], float mat_inA[9], float mat_inB[12]) {
    for (uint8_t i = 0; i < 4; i++)     {
        mat_out[i] = mat_inA[0]*mat_inB[i] + mat_inA[1]*mat_inB[i+4] + mat_inA[2]*mat_inB[i+8];
        mat_out[i+4] = mat_inA[3]*mat_inB[i] + mat_inA[4]*mat_inB[i+4] + mat_inA[5]*mat_inB[i+8];
        mat_out[i+8] = mat_inA[6]*mat_inB[i] + mat_inA[7]*mat_inB[i+4] + mat_inA[8]*mat_inB[i+8];
    }
 }

 /*
 * Returns the multiplication of matA * vecB, where matA is 3x4 and matB is 4x1
 */
 void mat_vec_multi_4d(float out[3], float matA[12], float vecB[4]) {
    out[0] = matA[0]*vecB[0]+matA[1]*vecB[1]+matA[2]*vecB[2]+matA[3]*vecB[3];
    out[1] = matA[4]*vecB[0]+matA[5]*vecB[1]+matA[6]*vecB[2]+matA[7]*vecB[3];
    out[2] = matA[8]*vecB[0]+matA[9]*vecB[1]+matA[10]*vecB[2]+matA[11]*vecB[3];
 }

 /*
 * Subtracts two 3D vectors from one another
 */
 void vec_subtract_3d(float out[3], float vecA[3], float vecB[3]) {
    out[0] = vecA[0]-vecB[0];
    out[1] = vecA[1]-vecB[1];
    out[2] = vecA[2]-vecB[2];
 }

 /*
 *    Jacobian of the nonlinear least squares function (3x4 matrix)
 */
 void jacobian(float out[12], float X[3], struct Anchor *anchors) {
    for (int i = 0; i<4; i++)   {
        out[i] = -(X[0]-anchors[i].pos.x)/sqrtf((X[0]-anchors[i].pos.x)*(X[0]-anchors[i].pos.x)+(X[1]-anchors[i].pos.y)*(X[1]-anchors[i].pos.y)+(X[2]-anchors[i].pos.z)*(X[2]-anchors[i].pos.z));
        out[i+4] = -(X[1]-anchors[i].pos.y)/sqrtf((X[0]-anchors[i].pos.x)*(X[0]-anchors[i].pos.x)+(X[1]-anchors[i].pos.y)*(X[1]-anchors[i].pos.y)+(X[2]-anchors[i].pos.z)*(X[2]-anchors[i].pos.z));
        out[i+8] = -(X[2]-anchors[i].pos.z)/sqrtf((X[0]-anchors[i].pos.x)*(X[0]-anchors[i].pos.x)+(X[1]-anchors[i].pos.y)*(X[1]-anchors[i].pos.y)+(X[2]-anchors[i].pos.z)*(X[2]-anchors[i].pos.z));
    }
 }

 /*
 * Calculates the value of the cost function (of the nonlin least squares problem) for a given estimated position
 */
 void costFunction(float F[4], float X[3], struct Anchor *anchors) {
    for(int i = 0; i < 4; i++)    {
        F[i] = anchors[i].distance-sqrtf((X[0]-anchors[i].pos.x)*(X[0]-anchors[i].pos.x)+(X[1]-anchors[i].pos.y)*(X[1]-anchors[i].pos.y)+(X[2]-anchors[i].pos.z)*(X[2]-anchors[i].pos.z));
    }
 }
