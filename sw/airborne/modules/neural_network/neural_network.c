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

        //TODO: Case when the agent is inside the buffer zone/obstacle

        //TOD: Add conditional check for distance to other drones

        /* Antennae Function values */  // TODO !!!!!!!!
        uint8_t numCells = msParams.sensorRange / MS_GRID_RES;

        nnParams.node_out[0] = MS_GRID_RES*(currentCell_y+1)-(*pos).y;
        for (uint8_t i = 0; i <= numCells; i++) {
            if(msParams.MS[currentCell_x][currentCell_y+i] != 0) {
                nnParams.node_out[0] = nnParams.node_out[0] + MS_GRID_RES;
                nnParams.node_out[8] = msParams.MS[currentCell_x][currentCell_y+i];
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
        for (uint8_t i = 0; i <= numCells; i++) {
            if(msParams.MS[currentCell_x+i][currentCell_y] != 0) {
                nnParams.node_out[2] = nnParams.node_out[2] + MS_GRID_RES;
                nnParams.node_out[10] = msParams.MS[currentCell_x+i][currentCell_y];
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
        for (uint8_t i = 0; i <= numCells; i++) {
            if(msParams.MS[currentCell_x][currentCell_y-i] != 0) {
                nnParams.node_out[4] = nnParams.node_out[4] + MS_GRID_RES;
                nnParams.node_out[12] = msParams.MS[currentCell_x][currentCell_y-i];
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
        for (uint8_t i = 0; i <= numCells; i++) {
            if(msParams.MS[currentCell_x-i][currentCell_y] != 0) {
                nnParams.node_out[6] = nnParams.node_out[6] + MS_GRID_RES;
                nnParams.node_out[14] = msParams.MS[currentCell_x-i][currentCell_y];
                if (nnParams.node_out[6] >= msParams.sensorRange) {
                    nnParams.node_out[6] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
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
    // TODO
 }

/** This function implements the activation function of the NN */
float activationFunction(float x) {
    float output = (expf(x)-expf(-x))/(expf(x)+expf(-x));

    return output;
}

/** This function calculates the outputs of the NN */
void calcNN(float *outputs[MS_NUM_OUTPUTS]) {
    uint8_t recurrentNN = 0;
    // Reset the node_out to zero for the new calculation
    for (uint8_t nodeNum = 2*MS_NUM_INPUTS; nodeNum < MS_NUM_NODES; nodeNum++){
        nnParams.node_out[nodeNum] = 0;
    }


    //Caculate the contributions of the initial connections
    float outNodeInput1;
    float outNodeInput2;
    for(uint8_t numOutputs = 0; numOutputs < MS_NUM_OUTPUTS; numOuputs++){
        for (uint8_t numIn = 0; numIn < MS_NUM_INPUTS; numIn++){
            nnParams.node_out[nnParams.outputIndex[numOutputs]] = nnParams.node_out[nnParams.outputIndex[numOutputs]] + nnParams.connectionsInit[numIN+MS_NUM_INPUTS*numOutputs]*NN_struct.node_out[numIN+MS_NUM_INPUTS*numOutputs];
        }
    }
    outNodeInput1 = nnParams.node_out[nnParams.outputIndex[0]];
    outNodeInput2 = nnParams.node_out[nnParams.outputIndex[1]];

    // Calculate the contributions of the added connections and Nodes
    for (uint8_t nodeNum = 2*MS_NUM_INPUTS; nodeNum < MS_NUM_NODES; nodeNum++){
        for (uint8_t connectNum = 0; connectNum < (MS_NUM_CONNECT-2*MS_NUM_INPUTS); connectNum++ ) {
            if(nnParams.connectTo[connectNum] == nnParams.node_ID[nodeNum]) {
                nnParams.node_out[nodeNum] = nnParams.node_out[nodeNum] + nnParams.connectWeight[connectNum]*nnParams.node_out[nnParams.connectFrom[connectNum]];
            }
            nnParams.node_out[nodeNum] = activationFunction(activationFunction);
        }
    }


    //Case when there are recurrent connections:
    if(recurrentNN == 1) {

        if found == 0
        float no_change_threshold=1e-3;

        uint8_t no_change_count;
        uint8_t index_loop = 0;         //-- Tracks the number of times the loop is run
        float inVal;

        while((no_change_count < num_nodes) && index_loop < 3*MS_NUM_CONNECT){
        	inVal = 0;
        	no_change_count = 0;

            // Calculate the contributions of the added connections and Nodes
            for (uint8_t nodeNum = 2*MS_NUM_INPUTS; nodeNum < MS_NUM_NODES; nodeNum++){
                for (uint8_t connectNum = 0; connectNum < (MS_NUM_CONNECT-2*MS_NUM_INPUTS); connectNum++ ) {
                    if(nnParams.connectTo[connectNum] == nnParams.node_ID[nodeNum]) {
                        inVal = inVal + nnParams.connectWeight[connectNum]*nnParams.node_out[nnParams.connectFrom[connectNum]];
                    }
                }
                //Add contributution of the origional connections to the output node
                if (nnParams.node_ID[nodeNum] == outputIndex[0]) {
                    inVal = inVal + outNodeInput1;
                }
                if (nnParams.node_ID[nodeNum] == outputIndex[1]) {
                    inVal = inVal + outNodeInput2;
                }
                //Compare new node output with old output
                inVal = activationFunction(inVal);
                if ((inVal-nnParams.node_out[nodeNum])*(inVal-nnParams.node_out[nodeNum]) < no_change_threshold) {
                	no_change_count = no_change_count + 1;
                }
                nnParams.node_out[nodeNum] = inVal;
            }
        }
    }

    //Return the output values:
    for(uint8_t numOutputs = 0; numOutputs < MS_NUM_OUTPUTS; numOuputs++){
        outputs[numOutputs] = nnParams.node_out[nnParams.outputIndex[numOutputs]];
    }
}

void neural_network_init(void) {
    /** Initialise the Mission Space age grid */
    for (uint8_t i = 0; i < MS_LENGTH/MS_GRID_RES; i++){
        for (uint8_t j = 0; j < MS_BREDTH/MS_GRID_RES; j++) {
            if j == 0 || j == MS_BREDTH/MS_GRID_RES-MS_GRID_RES {
            	msParams[j][i] == 0;
            } 
            else if(i == 0 || i == MS_LENGTH/MS_GRID_RES - MS_GRID_RES){
            	msParams[j][i] == 0;
            }
            else {
                msParams[j][i] = 1;
            }
        }
    }
}

void neural_network_periodic(void) {
    
}


