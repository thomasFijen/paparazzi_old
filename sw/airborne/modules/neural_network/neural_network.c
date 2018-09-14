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


/** This function determines the inputs into the NN */
void calcInputs(uint8_t *inNodes){
    struct EnuCoor_f *pos = stateGetPositionEnu_f();
}

/** This function implements the activation function of the NN */
float activationFunction(float x) {
    float output = (expf(x)-expf(-x))/(expf(x)+expf(-x));

    return output;
}

/** This function calculates the outputs of the NN */
float calcNN() {
    uint8_t recurrentNN = 0;
    // TODO: reset the node_out to zero for the new calculation...
    
    //Caculate the contributions of the initial connections
    for(uint8_t numOutputs = 0; numOutputs < MS_NUM_OUTPUTS; numOuputs++){
        for (uint8_t numIn = 0; numIn < MS_NUM_INPUTS; numIn++){
            NN_struct.node_out[NN_struct.outputIndex[numOutputs]] = NN_struct.node_out[NN_struct.outputIndex[numOutputs]] + NN_struct.connectionsInit[numIN+MS_NUM_INPUTS*numOutputs]*NN_struct.node_out[numIN+MS_NUM_INPUTS*numOutputs];
        }
    }

    // Calculate the contributions of the added connections and Nodes
    for (uint8_t nodeNum = 2*MS_NUM_INPUTS; nodeNum < MS_NUM_NODES; nodeNum++){
        for (uint8_t connectNum = 0; connectNum < (MS_NUM_CONNECT-2*MS_NUM_INPUTS); connectNum++ ) {
            if(NN_struct.connectTo[connectNum] == NN_struct.node_ID[nodeNum]) {
                NN_struct.node_out[nodeNum] = NN_struct.node_out[nodeNum] + NN_struct.connectWeight[connectNum]*NN_struct.node_out[NN_struct.connectFrom[connectNum]];
            }
            NN_struct.node_out[nodeNum] = activationFunction(activationFunction);
        }
    }


    //TODO: This still has to be completed. I might have to store the node inputs and the outputs...
    // reset inputs
    // calc new inputs based of old outputs (calced above)
    // calc new outputs
    // compare old vs new outputs to determine convergence
    // store new outputs
    // repeat if needed...
    if(recurrentNN == 1) {

        if found == 0
        no_change_threshold=1e-3;

        uint8_t no_change_count = 0;
        uint8_t index_loop = 0;         //-- Tracks the number of times the loop is run

        while(){
            // Calculate the contributions of the added connections and Nodes
            for (uint8_t nodeNum = 2*MS_NUM_INPUTS; nodeNum < MS_NUM_NODES; nodeNum++){
                for (uint8_t connectNum = 0; connectNum < (MS_NUM_CONNECT-2*MS_NUM_INPUTS); connectNum++ ) {
                    if(NN_struct.connectTo[connectNum] == NN_struct.node_ID[nodeNum]) {
                        NN_struct.node_out[nodeNum] = NN_struct.node_out[nodeNum] + NN_struct.connectWeight[connectNum]*NN_struct.node_out[NN_struct.connectFrom[connectNum]];
                    }
                }
            }
        }
    }
}

void neural_network_init(void) {
  
}

void neural_network_periodic(void) {
    uint8_t inputNodes[MS_NUM_INPUTS]
}


