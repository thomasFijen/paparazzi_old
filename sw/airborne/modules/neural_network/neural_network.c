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
#define MS_LENGTH 20.0f
#endif

#ifndef MS_BREDTH
#define MS_BREDTH 20.0f
#endif 

#ifndef MS_GRID_RES
#define MS_GRID_RES 1.0f
#endif

#ifndef MS_SWARM_SIZE
#define MS_SWARM_SIZE 3
#endif

#ifndef MS_TIME_STEP
#define MS_TIME_STEP 1.f
#endif

#ifndef MS_GRID_NUM_CELLS
#define MS_GRID_NUM_CELLS 400
#endif

#ifndef MS_NUM_INPUTS
#define MS_NUM_INPUTS 18
#endif

#ifndef MS_NUM_OUTPUTS
#define MS_NUM_OUTPUTS 2
#endif

#ifndef MS_NUM_NODES
#define MS_NUM_NODES 26
#endif

#ifndef MS_NUM_CONNECT
#define MS_NUM_CONNECT 51
#endif

/** Mission Space Parameter structure */
struct MS_Struct {
    uint8_t MS[MS_BREDTH/MS_GRID_RES][MS_LENGTH/MS_GRID_RES];
    float sensorRange = 10;

};

/** Structure containing NN parameters */
struct NN_struct {
    // At the moment the connections are input by hand here.
    float connectionsInit [2*MS_NUM_INPUTS]; // These are the weights of the origional connections

    // Connections added by the NEAT
    uint8_t connectFrom [MS_NUM_CONNECT-2*MS_NUM_INPUTS];   // These are the weights of the added connections
    uint8_t connectTo [MS_NUM_CONNECT-2*MS_NUM_INPUTS];     // These are the weights of the added connections
    float connectWeight [MS_NUM_CONNECT-2*MS_NUM_INPUTS];   // These are the weights of the added connections

    // Node Properties
    uint8_t outputIndex[MS_NUM_OUTPUTS];
    uint8_t node_ID [MS_NUM_NODES];
    float node_out [MS_NUM_NODES];
};

static struct MS_Struct msParams;
static struct NN_struct nnParams;

/** This function determines the inputs into the NN */
void calcInputs(){

    struct EnuCoor_f *pos = stateGetPositionEnu_f();

    /* Default range values */
    for (uint8_t i = 0; i < MS_NUM_INPUTS; i++) {
        nnParams.node_out[i] = 0.0;
    }

    uint8_t currentCell_x;
    uint8_t currentCell_y;
    if ((*pos).x >= 0 && (*pos).x <= MS_LENGTH && (*pos).y >= 0 && (*pos).y <= MS_BREDTH) {
        /* Get current cell index */
        currentCell_x = (uint8_t) (*pos).x/MS_GRID_RES;
        currentCell_y = (uint8_t) (*pos).y/MS_GRID_RES;
        printf("\nX: %i",currentCell_x);
        printf("\nX: %i",currentCell_y);

        //TOD: Add conditional check for distance to other drones

        /* Antennae Function values */  // TODO !!!!!!!!
        uint8_t numCells = msParams.sensorRange / MS_GRID_RES;

        nnParams.node_out[0] = MS_GRID_RES*(currentCell_y+1)-(*pos).y;
        for (uint8_t i = 1; i <= numCells; i++) {
            if(msParams.MS[currentCell_y+i][currentCell_x] != 0) {
                nnParams.node_out[0] = nnParams.node_out[0] + MS_GRID_RES;
                nnParams.node_out[8] = 100-msParams.MS[currentCell_y+i][currentCell_x];
                if (nnParams.node_out[0] >= msParams.sensorRange) {
                    nnParams.node_out[0] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
            }
        }

        nnParams.node_out[2] = MS_GRID_RES*(currentCell_x+1)-(*pos).x;
        for (uint8_t i = 1; i <= numCells; i++) {
            if(msParams.MS[currentCell_y][currentCell_x+i] != 0) {
                nnParams.node_out[2] = nnParams.node_out[2] + MS_GRID_RES;
                nnParams.node_out[10] = 100-msParams.MS[currentCell_y][currentCell_x+i];
                if (nnParams.node_out[2] >= msParams.sensorRange) {
                    nnParams.node_out[2] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
            }
        }

        nnParams.node_out[4] = (*pos).y - MS_GRID_RES*(currentCell_y);
        for (uint8_t i = 1; i <= numCells; i++) {
            if(msParams.MS[currentCell_y-i][currentCell_x] != 0) {
                nnParams.node_out[4] = nnParams.node_out[4] + MS_GRID_RES;
                nnParams.node_out[12] = 100-msParams.MS[currentCell_y-i][currentCell_x];
                if (nnParams.node_out[4] >= msParams.sensorRange) {
                    nnParams.node_out[4] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
            }
        }

        nnParams.node_out[6] = (*pos).x-MS_GRID_RES*(currentCell_x);
        for (uint8_t i = 1; i <= numCells; i++) {
            if(msParams.MS[currentCell_y][currentCell_x-i] != 0) {
                nnParams.node_out[6] = nnParams.node_out[6] + MS_GRID_RES;
                nnParams.node_out[14] = 100-msParams.MS[currentCell_y][currentCell_x-i];
                if (nnParams.node_out[6] >= msParams.sensorRange) {
                    nnParams.node_out[6] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
            }
        }

        /* Diagonal antennae */
        float stepSize = MS_GRID_RES/cosf(PI/4);
        printf("\n Step size: %f",stepSize);
        nnParams.node_out[1] = sqrtf(((*pos).x-MS_GRID_RES*(currentCell_x+1))*((*pos).x-MS_GRID_RES*(currentCell_x+1))+((*pos).y-MS_GRID_RES*(currentCell_y+1))*((*pos).y-MS_GRID_RES*(currentCell_y+1)));
        for(uint8_t i = 1; i <= numCells; i++) {
            if(msParams.MS[currentCell_y+i][currentCell_x+i] != 0) {
                nnParams.node_out[1] = nnParams.node_out[1] + stepSize;
                nnParams.node_out[9] = 100-msParams.MS[currentCell_y+i][currentCell_x+i];
                if (nnParams.node_out[1] >= msParams.sensorRange) {
                    nnParams.node_out[1] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
            }
        }

        nnParams.node_out[3] = sqrtf(((*pos).x-MS_GRID_RES*(currentCell_x+1))*((*pos).x-MS_GRID_RES*(currentCell_x+1))+((*pos).y-MS_GRID_RES*(currentCell_y))*((*pos).y-MS_GRID_RES*(currentCell_y)));
     /*   if(fmodf((*pos).x,MS_GRID_RES) == 0 && fmodf((*pos).y,MS_GRID_RES) == 0) {
            nnParams.node_out[3] = stepSize;
        }else {
            nnParams.node_out[3] = sqrtf(((*pos).x-MS_GRID_RES*(currentCell_x+1))*((*pos).x-MS_GRID_RES*(currentCell_x+1))+((*pos).y-MS_GRID_RES*(currentCell_y))*((*pos).y-MS_GRID_RES*(currentCell_y)));
        }*/
        nnParams.node_out[3] = sqrtf(((*pos).x-MS_GRID_RES*(currentCell_x+1))*((*pos).x-MS_GRID_RES*(currentCell_x+1))+((*pos).y-MS_GRID_RES*(currentCell_y))*((*pos).y-MS_GRID_RES*(currentCell_y)));
        for(uint8_t i = 1; i <= numCells; i++) {
            if(msParams.MS[currentCell_y-i][currentCell_x+i] != 0) {
                nnParams.node_out[3] = nnParams.node_out[3] + stepSize;
                nnParams.node_out[11] = 100-msParams.MS[currentCell_y-i][currentCell_x+i];
                if (nnParams.node_out[3] >= msParams.sensorRange) {
                    nnParams.node_out[3] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
            }
        }

        nnParams.node_out[5] = sqrtf(((*pos).x-MS_GRID_RES*(currentCell_x))*((*pos).x-MS_GRID_RES*(currentCell_x))+((*pos).y-MS_GRID_RES*(currentCell_y))*((*pos).y-MS_GRID_RES*(currentCell_y)));
        for(uint8_t i = 1; i <= numCells; i++) {
            if(msParams.MS[currentCell_y-i][currentCell_x-i] != 0) {
                nnParams.node_out[5] = nnParams.node_out[5] + stepSize;
                nnParams.node_out[13] = 100-msParams.MS[currentCell_y-i][currentCell_x-i];
                if (nnParams.node_out[5] >= msParams.sensorRange) {
                    nnParams.node_out[5] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
            }
        }

        nnParams.node_out[7] = sqrtf(((*pos).x-MS_GRID_RES*(currentCell_x))*((*pos).x-MS_GRID_RES*(currentCell_x))+((*pos).y-MS_GRID_RES*(currentCell_y+1))*((*pos).y-MS_GRID_RES*(currentCell_y+1)));
        for(uint8_t i = 1; i <= numCells; i++) {
            if(msParams.MS[currentCell_y+i][currentCell_x-i] != 0) {
                nnParams.node_out[7] = nnParams.node_out[7] + stepSize;
                nnParams.node_out[15] = 100-msParams.MS[currentCell_y+i][currentCell_x-i];
                if (nnParams.node_out[7] >= msParams.sensorRange) {
                    nnParams.node_out[7] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
            }
        }
        
        /* Correcting the range measurements to account for other UAVs */
        for(uint8_t i = 0; i < MS_SWARM_SIZE; i++){
            if (i != ID) {
                uint8_t agentCell_x = (uint8_t) msParams.uavs[i].x/MS_GRID_RES;
                uint8_t agentCell_y = (uint8_t) msParams.uavs[i].y/MS_GRID_RES;
                if(agentCell_x == currentCell_x){
                    float distance = sqrtf((msParams.uavs[i].x-(*pos).x)*(msParams.uavs[i].x-(*pos).x)+(msParams.uavs[i].y-(*pos).y)*(msParams.uavs[i].y-(*pos).y));
                }
                if (agentCell_y == currentCell_y){
                    float distance = sqrtf((msParams.uavs[i].x-(*pos).x)*(msParams.uavs[i].x-(*pos).x)+(msParams.uavs[i].y-(*pos).y)*(msParams.uavs[i].y-(*pos).y));
                }
            }

        }

        /* Boolean in area */
        nnParams.node_out[MS_NUM_INPUTS-2] = 1;
    }
    else{
        /* Boolean outside area */
        nnParams.node_out[MS_NUM_INPUTS-2] = 0;
    }

    /** Bias Node */
    nnParams.node_out[MS_NUM_INPUTS-1] = 1;

    /* Normalising the input values */
    nnParams.node_out[0] = nnParams.node_out[0]/msParams.sensorRange;
    nnParams.node_out[1] = nnParams.node_out[1]/msParams.sensorRange;
    nnParams.node_out[2] = nnParams.node_out[2]/msParams.sensorRange;
    nnParams.node_out[3] = nnParams.node_out[3]/msParams.sensorRange;
    nnParams.node_out[4] = nnParams.node_out[4]/msParams.sensorRange;
    nnParams.node_out[5] = nnParams.node_out[5]/msParams.sensorRange;
    nnParams.node_out[6] = nnParams.node_out[6]/msParams.sensorRange;
    nnParams.node_out[7] = nnParams.node_out[7]/msParams.sensorRange;
}

/** This function implements the activation function of the NN */
float activationFunction(float x) {
    float output = (expf(x)-expf(-x))/(expf(x)+expf(-x));

    return output;
}

/** This function calculates the outputs of the NN */
void calcNN(float outputs[MS_NUM_OUTPUTS]) {
    uint8_t recurrentNN = 1;
    // Reset the node_out to zero for the new calculation
    for (uint8_t nodeNum = MS_NUM_INPUTS; nodeNum < MS_NUM_NODES; nodeNum++){
        nnParams.node_out[nodeNum] = 0;
    }

    //Calculate the contributions of the initial connections
    float outNodeInput1;
    float outNodeInput2;

    for(uint8_t numOutputs = 0; numOutputs < MS_NUM_OUTPUTS; numOutputs++){
        for (uint8_t numIn = 0; numIn < MS_NUM_INPUTS; numIn++){
            nnParams.node_out[nnParams.outputIndex[numOutputs]] = nnParams.node_out[nnParams.outputIndex[numOutputs]] + nnParams.connectionsInit[numIn+MS_NUM_INPUTS*numOutputs]*nnParams.node_out[numIn];
        }
    }
    outNodeInput1 = nnParams.node_out[nnParams.outputIndex[0]];
    outNodeInput2 = nnParams.node_out[nnParams.outputIndex[1]];

    // Calculate the contributions of the added connections and Nodes
    for (uint8_t nodeNum = MS_NUM_INPUTS; nodeNum < MS_NUM_NODES; nodeNum++){
        for (uint8_t connectNum = 0; connectNum < (MS_NUM_CONNECT-2*MS_NUM_INPUTS); connectNum++ ) {
            if(nnParams.connectTo[connectNum] == nnParams.node_ID[nodeNum]) {
                float inValue;
                for(uint8_t i = 0; i < MS_NUM_NODES; i++){
                    if(nnParams.connectFrom[connectNum] == nnParams.node_ID[i]) {
                        inValue = nnParams.node_out[i];
                    }
                }
                nnParams.node_out[nodeNum] = nnParams.node_out[nodeNum] + nnParams.connectWeight[connectNum]*inValue;
            }
        }
        nnParams.node_out[nodeNum] = activationFunction(nnParams.node_out[nodeNum]);
    }

    //Case when there are recurrent connections:
    if(recurrentNN == 1) {
        float no_change_threshold=1e-3;
        uint8_t no_change_count = 0;
        uint8_t index_loop = 0;         //-- Tracks the number of times the loop is run
        float inVal;

        while((no_change_count < MS_NUM_NODES) && index_loop < 3*MS_NUM_CONNECT){
            no_change_count = MS_NUM_INPUTS;

            // Calculate the contributions of the added connections and Nodes
            for (uint8_t nodeNum = MS_NUM_INPUTS; nodeNum < MS_NUM_NODES; nodeNum++){
                inVal = 0;
                for (uint8_t connectNum = 0; connectNum < (MS_NUM_CONNECT-2*MS_NUM_INPUTS); connectNum++) {
                    if(nnParams.connectTo[connectNum] == nnParams.node_ID[nodeNum]) {
                        float inValueTemp;
                        for(uint8_t i = 0; i < MS_NUM_NODES; i++){
                            if(nnParams.connectFrom[connectNum] == nnParams.node_ID[i]) {
                                inValueTemp = nnParams.node_out[i];
                            }
                        }
                        inVal = inVal + nnParams.connectWeight[connectNum]*inValueTemp;
                    }
                }
                //Add contribution of the original connections to the output node. Hard coded to two outputs
                if (nnParams.node_ID[nodeNum] == 19) {
                    inVal = inVal + outNodeInput1;
                }
                if (nnParams.node_ID[nodeNum] == 20) {
                    inVal = inVal + outNodeInput2;
                }
                //Compare new node output with old output
                inVal = activationFunction(inVal);
                if ((inVal-nnParams.node_out[nodeNum])*(inVal-nnParams.node_out[nodeNum]) < no_change_threshold) {
                    no_change_count = no_change_count + 1;
                }
                nnParams.node_out[nodeNum] = inVal;
            }
            index_loop++;
        }
    }

    //Return the output values:
    for(uint8_t numOutputs = 0; numOutputs < MS_NUM_OUTPUTS; numOutputs++){
        outputs[numOutputs] = nnParams.node_out[nnParams.outputIndex[numOutputs]];
    }
}

void neural_network_init(void) {
    /** Initialize the Mission Space age grid */
    for (uint8_t i = 0; i < MS_LENGTH/MS_GRID_RES; i++){
        for (uint8_t j = 0; j < MS_BREDTH/MS_GRID_RES; j++) {
            if( j == 0 || j == MS_BREDTH/MS_GRID_RES-MS_GRID_RES) {
                msParams.MS[j][i] = 0;
            }
            else if(i == 0 || i == MS_LENGTH/MS_GRID_RES - MS_GRID_RES){
                msParams.MS[j][i] = 0;
            }
            else {
                msParams.MS[j][i] = 1;
            }
        }
    }

    /* Initialise the NN structure */
    int8_t assign[26] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,21,68,93,210,19,108,153,20};
    memcpy(nnParams.node_ID, assign, 26*sizeof(uint8_t));

    uint8_t assign2[2] = {22,25};
    memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    float connectionsInit[36] = {7.806380957,5.528580557,0.580185567,-0.892800512,-1.560863676,-5.299624156,-4.744623565,7.188228362,-1.393527294,-5.525020404,2.879480339,1.522226165,3.768690915,-8,-3.174616974,-4.270717264,4.399241565,-6.88788973,1.85223179,7.267868926,-1.837650023,-8,-1.754804325,5.439443797,6.399576228,1.87862188,4.723880129,-4.245715255,0.678777551,4.695496426,-2.394260424,-7.186473925,1.838699615,3.85452967,-4.274725797,-8};
    memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    float connect[15] = {-2.097217819,-2.63461102,1.054388957,-1.586494309,-0.856340471,-3.970069819,-2.066530808,-1.077418409,-1.693239987,-1.92564801,2.955273135,-8,-3.16237498,2.989823099,-1.877785137};
    memcpy(nnParams.connectWeight, connect, 15*sizeof(float));

    uint8_t connectTo[15] = {21,20,68,20,93,20,108,20,153,20,21,93,153,210,19};
    memcpy(nnParams.connectTo, connectTo, 15*sizeof(uint8_t));

    uint8_t connectFrom[15] = {7,21,1,68,18,93,93,108,10,153,3,7,68,7,210};
    memcpy(nnParams.connectFrom, connectFrom, 15*sizeof(uint8_t));
}

void neural_network_periodic(void) {
    
}


