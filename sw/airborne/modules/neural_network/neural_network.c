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
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "subsystems/navigation/waypoints.h"
#include "modules/decawave/uwb_localisation_and_comms.h"

#ifndef MS_LENGTH
#define MS_LENGTH 7.0f
#endif

#ifndef MS_BREDTH
#define MS_BREDTH 7.0f
#endif 

#ifndef MS_GRID_RES
#define MS_GRID_RES 0.5f
#endif

#ifndef MS_SWARM_SIZE
#define MS_SWARM_SIZE 3
#endif

#ifndef MS_TIME_STEP
#define MS_TIME_STEP 0.5f
#endif

#ifndef MS_GRID_NUM_CELLS
#define MS_GRID_NUM_CELLS 128
#endif

#ifndef MS_NUM_INPUTS
#define MS_NUM_INPUTS 18
#endif

#ifndef MS_NUM_OUTPUTS
#define MS_NUM_OUTPUTS 2
#endif

#ifndef MS_NUM_NODES
#define MS_NUM_NODES 24
#endif

#ifndef MS_NUM_CONNECT
#define MS_NUM_CONNECT 49
#endif

#ifndef MS_MAX_VEL
#define MS_MAX_VEL 1.0f
#endif

#ifndef MS_SENSOR_RANGE
#define MS_SENSOR_RANGE 4.0f
#endif

#ifndef MS_CURRENT_ID
#define MS_CURRENT_ID MS_SWARM_SIZE-1
#endif

#define PI 3.14159265


static struct MS_Struct msParams;
static struct NN_struct nnParams;
/** Mission Space Parameter structure */
struct MS_Struct {
    // uint8_t MS[(uint8_t) (MS_BREDTH/MS_GRID_RES)][(uint8_t) (MS_LENGTH/MS_GRID_RES)];
    uint8_t MS[14][14];
    float sensorRange;
	struct EnuCoor_f uavs[MS_SWARM_SIZE];
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
void calcInputs(){
/* This is used for testing in Eclipse
    struct EnuCoor_f UAV;
    UAV.x = 10.5;
    UAV.y = 10.5;
    UAV.z = 10; 
    
    struct EnuCoor_f *pos = &(UAV);
    */

    /* Conversion between coordinate systems */
    float a = 0.827559;
    float b = 0.5613786;
    float c = -3.903735;
    float d = 1.0823;

    struct EnuCoor_f *pos = stateGetPositionEnu_f();
    float tempX=(*pos).x;
    float tempY=(*pos).y;
    (*pos).x = a*tempX+b*tempY+c;
    (*pos).y = -b*tempX+a*tempY+d;

    for(uint8_t i=0;i<MS_SWARM_SIZE;i++){
        float temp[2];
        getPos_UWB((i+2),temp);             //!!!!!!!!!!!!!!!! MAGIC NUMBER: BEWARE!!!!!!
        msParams.uavs[i].x = temp[0];
        msParams.uavs[i].y = temp[1];

        msParams.uavs[i].x = a*temp[0]+b*temp[1]+c;
        msParams.uavs[i].y = -b*temp[0]+a*temp[1]+d;
    }
    
    /* Default range values */
    for (uint8_t i = 0; i < MS_NUM_INPUTS; i++) {
        nnParams.node_out[i] = 0.0;
    }

    uint8_t currentCell_x;
    uint8_t currentCell_y;
    if ((*pos).x >= 0 && (*pos).x <= MS_LENGTH && (*pos).y >= 0 && (*pos).y <= MS_BREDTH) {
        /* Get current cell index */
        currentCell_x = (uint8_t) ((*pos).x/MS_GRID_RES);
        currentCell_y = (uint8_t) ((*pos).y/MS_GRID_RES);

        /* Antennae Function values */
        uint8_t numCells = msParams.sensorRange / MS_GRID_RES;

        nnParams.node_out[0] = MS_GRID_RES*(currentCell_y+1)-(*pos).y;
        nnParams.node_out[8] = (100-msParams.MS[currentCell_y][currentCell_x])/100.0;
        for (uint8_t i = 1; i <= numCells; i++) {
            if (((*pos).y+i*MS_GRID_RES) > MS_BREDTH) {
                break;
            }
            else if(msParams.MS[currentCell_y+i][currentCell_x] != 0) {
                nnParams.node_out[0] = nnParams.node_out[0] + MS_GRID_RES;
                nnParams.node_out[8] = (100-msParams.MS[currentCell_y+i][currentCell_x])/100.0;
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
        nnParams.node_out[10] = (100-msParams.MS[currentCell_y][currentCell_x])/100.0;
        for (uint8_t i = 1; i <= numCells; i++) {
            if (((*pos).x+i*MS_GRID_RES) > MS_LENGTH) {
                break;
            }
            else if(msParams.MS[currentCell_y][currentCell_x+i] != 0) {
                nnParams.node_out[2] = nnParams.node_out[2] + MS_GRID_RES;
                nnParams.node_out[10] = (100-msParams.MS[currentCell_y][currentCell_x+i])/100.0;
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
        nnParams.node_out[12] = (100-msParams.MS[currentCell_y][currentCell_x])/100.0;
        for (uint8_t i = 1; i <= numCells; i++) {
            if (((*pos).y-i*MS_GRID_RES) < 0) {
                break;
            }
            else if(msParams.MS[currentCell_y-i][currentCell_x] != 0) {
                nnParams.node_out[4] = nnParams.node_out[4] + MS_GRID_RES;
                nnParams.node_out[12] = (100-msParams.MS[currentCell_y-i][currentCell_x])/100.0;
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
        nnParams.node_out[14] = (100-msParams.MS[currentCell_y][currentCell_x])/100.0;
        for (uint8_t i = 1; i <= numCells; i++) {
            if (((*pos).x-i*MS_GRID_RES) < 0) {
                break;
            }
            else if(msParams.MS[currentCell_y][currentCell_x-i] != 0) {
                nnParams.node_out[6] = nnParams.node_out[6] + MS_GRID_RES;
                nnParams.node_out[14] = (100-msParams.MS[currentCell_y][currentCell_x-i])/100.0;
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

        nnParams.node_out[1] = sqrtf(((*pos).x-MS_GRID_RES*(currentCell_x+1))*((*pos).x-MS_GRID_RES*(currentCell_x+1))+((*pos).y-MS_GRID_RES*(currentCell_y+1))*((*pos).y-MS_GRID_RES*(currentCell_y+1)));
        nnParams.node_out[9] = (100-msParams.MS[currentCell_y][currentCell_x])/100.0;
        for(uint8_t i = 1; i <= numCells; i++) {
            if (((*pos).y+i*MS_GRID_RES) > MS_BREDTH || ((*pos).x+i*MS_GRID_RES) > MS_LENGTH) {
                break;
            }
            else if(msParams.MS[currentCell_y+i][currentCell_x+i] != 0) {
                nnParams.node_out[1] = nnParams.node_out[1] + stepSize;
                nnParams.node_out[9] = (100-msParams.MS[currentCell_y+i][currentCell_x+i])/100.0;
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
        // nnParams.node_out[3] = sqrtf(((*pos).x-MS_GRID_RES*(currentCell_x+1))*((*pos).x-MS_GRID_RES*(currentCell_x+1))+((*pos).y-MS_GRID_RES*(currentCell_y))*((*pos).y-MS_GRID_RES*(currentCell_y)));
        nnParams.node_out[11] = (100-msParams.MS[currentCell_y][currentCell_x])/100.0;
        for(uint8_t i = 1; i <= numCells; i++) {
            if (((*pos).y-i*MS_GRID_RES) < 0 || ((*pos).x+i*MS_GRID_RES) > MS_BREDTH) {
                break;
            }
            else if(msParams.MS[currentCell_y-i][currentCell_x+i] != 0) {
                nnParams.node_out[3] = nnParams.node_out[3] + stepSize;
                nnParams.node_out[11] = (100-msParams.MS[currentCell_y-i][currentCell_x+i])/100.0;
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
        nnParams.node_out[13] = (100-msParams.MS[currentCell_y][currentCell_x])/100.0;
        for(uint8_t i = 1; i <= numCells; i++) {
            if (((*pos).y-i*MS_GRID_RES) < 0 || ((*pos).x-i*MS_GRID_RES) < 0) {
                break;
            }
            else if(msParams.MS[currentCell_y-i][currentCell_x-i] != 0) {
                nnParams.node_out[5] = nnParams.node_out[5] + stepSize;
                nnParams.node_out[13] = (100-msParams.MS[currentCell_y-i][currentCell_x-i])/100.0;
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
        nnParams.node_out[15] = (100-msParams.MS[currentCell_y][currentCell_x])/100.0;
        for(uint8_t i = 1; i <= numCells; i++) {
            if (((*pos).y+i*MS_GRID_RES) > MS_BREDTH || ((*pos).x-i*MS_GRID_RES) < 0) {
                break;
            }
            else if(msParams.MS[currentCell_y+i][currentCell_x-i] != 0) {
                nnParams.node_out[7] = nnParams.node_out[7] + stepSize;
                nnParams.node_out[15] = (100-msParams.MS[currentCell_y+i][currentCell_x-i])/100.0;
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
            if (i != MS_CURRENT_ID) {
                if (msParams.uavs[i].x >= 0 && msParams.uavs[i].x <= MS_LENGTH && msParams.uavs[i].y >= 0 && msParams.uavs[i].y <= MS_BREDTH) {
                    uint8_t agentCell_x = (uint8_t) (msParams.uavs[i].x/MS_GRID_RES);
                    uint8_t agentCell_y = (uint8_t) (msParams.uavs[i].y/MS_GRID_RES);
                    if(agentCell_x == currentCell_x){
                        float distance = sqrtf((msParams.uavs[i].x-(*pos).x)*(msParams.uavs[i].x-(*pos).x)+(msParams.uavs[i].y-(*pos).y)*(msParams.uavs[i].y-(*pos).y));
                        if(agentCell_y >= currentCell_y && distance < nnParams.node_out[0]){
                            nnParams.node_out[0] = distance;
                        } else if (agentCell_y < currentCell_y && distance < nnParams.node_out[4]) {
                            nnParams.node_out[4] = distance;
                        }
                    }
                    else if (agentCell_y == currentCell_y){
                        float distance = sqrtf((msParams.uavs[i].x-(*pos).x)*(msParams.uavs[i].x-(*pos).x)+(msParams.uavs[i].y-(*pos).y)*(msParams.uavs[i].y-(*pos).y));
                        if(agentCell_x >= currentCell_x && distance < nnParams.node_out[2]){
                            nnParams.node_out[2] = distance;
                        } else if (agentCell_x < currentCell_x && distance < nnParams.node_out[6]) {
                            nnParams.node_out[6] = distance;
                        }
                    }
                    else if ((agentCell_y-currentCell_y) == (agentCell_x-currentCell_x) && agentCell_x >= currentCell_x ) {
                        float distance = sqrtf((msParams.uavs[i].x-(*pos).x)*(msParams.uavs[i].x-(*pos).x)+(msParams.uavs[i].y-(*pos).y)*(msParams.uavs[i].y-(*pos).y));
                        if(distance < nnParams.node_out[1]){
                            nnParams.node_out[1] = distance;
                        }
                    } else if ((currentCell_y-agentCell_y) == (agentCell_x-currentCell_x) && agentCell_x >= currentCell_x){
                        float distance = sqrtf((msParams.uavs[i].x-(*pos).x)*(msParams.uavs[i].x-(*pos).x)+(msParams.uavs[i].y-(*pos).y)*(msParams.uavs[i].y-(*pos).y));
                        if(distance < nnParams.node_out[3]){
                            nnParams.node_out[3] = distance;
                        }
                    } else if ((currentCell_y-agentCell_y) == (currentCell_x-agentCell_x) && agentCell_x < currentCell_x) {
                        float distance = sqrtf((msParams.uavs[i].x-(*pos).x)*(msParams.uavs[i].x-(*pos).x)+(msParams.uavs[i].y-(*pos).y)*(msParams.uavs[i].y-(*pos).y));
                        if(distance < nnParams.node_out[5]){
                            nnParams.node_out[5] = distance;
                        }
                    } else if ((agentCell_y-currentCell_y) == (currentCell_x-agentCell_x) && agentCell_x < currentCell_x) {
                        float distance = sqrtf((msParams.uavs[i].x-(*pos).x)*(msParams.uavs[i].x-(*pos).x)+(msParams.uavs[i].y-(*pos).y)*(msParams.uavs[i].y-(*pos).y));
                        if(distance < nnParams.node_out[7]){
                            nnParams.node_out[7] = distance;
                        }
                    }
                }
            }

        }

        /* Boolean in area */
        nnParams.node_out[MS_NUM_INPUTS-2] = 1;
    }
    else{
        for (uint8_t i = 0; i < MS_NUM_INPUTS; i++){
            nnParams.node_out[i] = 0;
        }

        if ((*pos).x < 0) { 
            if ((*pos).y < 0) {
                nnParams.node_out[1] = msParams.sensorRange;
                nnParams.node_out[9] = 1.0;
            }else if ((*pos).y >= 0 && (*pos).y <= MS_BREDTH) {
                nnParams.node_out[2] = msParams.sensorRange;
                nnParams.node_out[10] = 1.0;
            }else {
                nnParams.node_out[3] = msParams.sensorRange;
                nnParams.node_out[11] = 1.0;
            }
        }else if ((*pos).x >=0 && (*pos).x <= MS_LENGTH){ 
            if ((*pos).y < 0) { 
                nnParams.node_out[0] = msParams.sensorRange;
                nnParams.node_out[8] = 1.0;
            }else if ((*pos).y > MS_BREDTH) { 
                nnParams.node_out[4] = msParams.sensorRange;
                nnParams.node_out[12] = 1.0;
            }
        }else { 
            if ((*pos).y < 0) { 
                nnParams.node_out[7] = msParams.sensorRange;
                nnParams.node_out[15] = 1.0;
            }else if ((*pos).y >= 0 && (*pos).y <= MS_BREDTH) { 
                nnParams.node_out[6] = msParams.sensorRange;
                nnParams.node_out[14] = 1.0;
            }else {
                nnParams.node_out[5] = msParams.sensorRange;
                nnParams.node_out[13] = 1.0;
            }
        }

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
    // nnParams.node_out[8] = nnParams.node_out[8]/100.0f;
    // nnParams.node_out[9] = nnParams.node_out[9]/100.0f;
    // nnParams.node_out[10] = nnParams.node_out[10]/100.0f;
    // nnParams.node_out[11] = nnParams.node_out[11]/100.0f;
    // nnParams.node_out[12] = nnParams.node_out[12]/100.0f;
    // nnParams.node_out[13] = nnParams.node_out[13]/100.0f;
    // nnParams.node_out[14] = nnParams.node_out[14]/100.0f;
    // nnParams.node_out[15] = nnParams.node_out[15]/100.0f;
}

/** This function implements the activation function of the NN */
float activationFunction(float x) {
    float output = (expf(x)-expf(-x))/(expf(x)+expf(-x));

    return output;
}

/** This function calculates the outputs of the NN */
void calcNN() {
    uint8_t recurrentNN = 0;
    // Reset the node_out to zero for the new calculation
    for (uint8_t nodeNum = MS_NUM_INPUTS; nodeNum < MS_NUM_NODES; nodeNum++){
        nnParams.node_out[nodeNum] = 0;
    }

    /* Determine the inputs to the NN */
    calcInputs();

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
                float inValue = 0;
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
                        float inValueTemp = 0;
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
    float outputs[MS_NUM_OUTPUTS];
    for(uint8_t numOutputs = 0; numOutputs < MS_NUM_OUTPUTS; numOutputs++){
        outputs[numOutputs] = nnParams.node_out[nnParams.outputIndex[numOutputs]];
    }
    
    /* Converting the NN outputs to velocities */
    float theta;
    if (outputs[0] != 0) {
        theta = atanf(fabs(outputs[1]/outputs[0]));
    } else {
    	theta = PI/2;
    }
    
    outputs[0] = outputs[0]*MS_MAX_VEL*cosf(theta);
    outputs[1] = outputs[1]*MS_MAX_VEL*sinf(theta);

    // // Flying in NAV mode 
    // struct EnuCoor_f newWaypoint;
    // struct EnuCoor_f *pos = stateGetPositionEnu_f();

    /* Conversion between coordinate systems */
    // float a = 0.827559;
    // float b = 0.5613786;
    // float c = -3.903735;
    // float d = 1.0823;

    // newWaypoint.x = a*((*pos).x+outputs[0]*MS_TIME_STEP)+b*((*pos).y+outputs[1]*MS_TIME_STEP)+c; 
    // newWaypoint.y = -b*((*pos).x+outputs[0]*MS_TIME_STEP)+a*((*pos).y+outputs[1]*MS_TIME_STEP)+d;

    // newWaypoint.x = (*pos).x+outputs[0]*MS_TIME_STEP; 
    // newWaypoint.y = (*pos).y+outputs[1]*MS_TIME_STEP;
    // newWaypoint.z=(*pos).z;
    // waypoint_set_enu(wp_id,&newWaypoint);

    // THIS IS FOR GUIDED MODE
    
    // guidance_h_set_guided_vel(outputs[0],outputs[1]); 
    // guidance_h_set_guided_vel(outputs[1],outputs[0]); //This is for optitrack, x and y swapped around!!!
    guidance_h_set_guided_body_vel(outputs[1],outputs[0]);

    // return 0;
}

void ageMS(void){
    /* Conversion between coordinate systems */
    float a = 0.827559;
    float b = 0.5613786;
    float c = -3.903735;
    float d = 1.0823;

    for(uint8_t x = 0; x < MS_LENGTH/MS_GRID_RES; x ++) {
        for (uint8_t y = 0; y < MS_LENGTH/MS_GRID_RES; y ++) {
            if (msParams.MS[y][x] != 0){
                msParams.MS[y][x] = msParams.MS[y][x] -1;
                if(msParams.MS[y][x] < 1){
                    msParams.MS[y][x] = 1;
                }
            }
        }
    }

    float footprint = 1.0; //This is the size of the sensor footprint

    for (uint8_t agentNum = 0; agentNum < MS_SWARM_SIZE; agentNum++){
        uint8_t currentCell_x = 0;
        uint8_t currentCell_y = 0;
        float loopX = 0;    //stores the position of the current UAV being tested. Just for convenience 
        float loopY = 0;
        if(agentNum == MS_CURRENT_ID){
            struct EnuCoor_f *pos = stateGetPositionEnu_f();
            float tempX=(*pos).x;
            float tempY=(*pos).y;
            (*pos).x = a*tempX+b*tempY+c;
            (*pos).y = -b*tempX+a*tempY+d;
            if ((*pos).x >= 0 && (*pos).x <= MS_LENGTH && (*pos).y >= 0 && (*pos).y <= MS_BREDTH) {
                currentCell_x = (uint8_t) ((*pos).x/MS_GRID_RES);
                currentCell_y = (uint8_t) ((*pos).y/MS_GRID_RES);
                loopX = (*pos).x;
                loopY = (*pos).y;
            }
        }
        else{
            if (msParams.uavs[agentNum].x >= 0 && msParams.uavs[agentNum].x <= MS_LENGTH && msParams.uavs[agentNum].y >= 0 && msParams.uavs[agentNum].y <= MS_BREDTH){
                currentCell_x = (uint8_t) (msParams.uavs[agentNum].x/MS_GRID_RES);
                currentCell_y = (uint8_t) (msParams.uavs[agentNum].y/MS_GRID_RES);
                loopX = msParams.uavs[agentNum].x;
                loopY = msParams.uavs[agentNum].y;
            }
        }
        if(msParams.MS[currentCell_y][currentCell_x] != 0) {
            // msParams.MS[currentCell_y][currentCell_x] = 100;
            uint8_t lim[4];

            /* Finding cells in range */
            uint8_t tempLimit;

            tempLimit = currentCell_x + (uint8_t) (footprint/MS_GRID_RES);
            if (tempLimit < (uint8_t) (MS_LENGTH/MS_GRID_RES)){
                lim[0] = tempLimit;
            }else{
                lim[0] = (uint8_t) (MS_LENGTH/MS_GRID_RES);
            }

            tempLimit = (uint8_t) (footprint/MS_GRID_RES);
            if (currentCell_x > tempLimit){
                lim[1] = currentCell_x-tempLimit;
            } else {
                lim[1] = 0;
            }

            tempLimit = currentCell_y + (uint8_t) (footprint/MS_GRID_RES);
            if (tempLimit < (uint8_t) (MS_BREDTH/MS_GRID_RES)){
                lim[2] = tempLimit;
            }else{
                lim[2] = (uint8_t) (MS_BREDTH/MS_GRID_RES);
            }

            tempLimit = (uint8_t) (footprint/MS_GRID_RES);
            if (currentCell_y > tempLimit){
                lim[3] = currentCell_y-tempLimit;;
            } else {
                lim[3] = 0;
            }
            for (uint8_t i=lim[1];i<=lim[0];i++){
                for (uint8_t j=lim[3];j<=lim[2];j++){
                    if (msParams.MS[j][i] != 0){
                        float x = i*MS_GRID_RES+MS_GRID_RES/2;
                        float y = j*MS_GRID_RES+MS_GRID_RES/2;
                        float dist = (x-loopX)*(x-loopX)+(y-loopY)*(y-loopY);
                        if (dist <= (footprint-MS_GRID_RES/2)*(footprint-MS_GRID_RES/2)) {
                            msParams.MS[j][i] = 100;
                        }
                    }
                }
            }
        }
    }
}

void neural_network_init(void) {
    /** Initialize the Mission Space age grid */
    for (uint8_t x = 0; x < MS_LENGTH/MS_GRID_RES; x++){
        for (uint8_t y = 0; y < MS_BREDTH/MS_GRID_RES; y++) {
            if( y == 0 || y == MS_BREDTH/MS_GRID_RES-1) {
                msParams.MS[y][x] = 0;
            }
            else if(x == 0 || x == MS_LENGTH/MS_GRID_RES - 1){
                msParams.MS[y][x] = 0;
            }
            else {
                msParams.MS[y][x] = 1;
            }
        }
    }
    
    msParams.sensorRange = MS_SENSOR_RANGE;
    /* Starting positions of the Drones */
    msParams.uavs[0].x = 3.5;
    msParams.uavs[0].y = 3.5;
    // msParams.uavs[1].x = 0.5;
    // msParams.uavs[1].y = 0.5;
    // msParams.uavs[2].x = 11;
    // msParams.uavs[2].y = 11;


/* _____________________________________________ NN for final data___________________________________________________ */
    /* Initialise the NN structure: Test 1 
     * Number of nodes: 22
     * Number of connections: 43    */
    // uint8_t assign[22] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,71,78,20,19};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {20,21};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-5.0315840434,3.449652292,5.5406762516,1.8517735034,-5.7041667863,-5.6688523585,-7.3692722924,3.418286351,8,8,5.9171154969,2.8235996033,-4.090047386,-3.4751704528,4.1989777832,-7.1941945078,-5.2213520003,8,4.0775034873,8,-5.4611103937,-1.1725010766,-8,6.6417367784,5.3697933532,4.6328042655,-4.3059179946,1.1892661373,-8,-6.5313075415,-2.7324348095,-2.0054456291,3.1248600167,0.901027336,6.134547426,-6.3317171363};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[7] = {5.5419895544,-5.0294275128,-8,1.4738066378,4.6370676007,0.4914742255,3.1691563089};
    // memcpy(nnParams.connectWeight, connect, 7*sizeof(float));

    // uint8_t connectTo[7] = {71,20,78,19,19,78,78};
    // memcpy(nnParams.connectTo, connectTo, 7*sizeof(uint8_t));

    // uint8_t connectFrom[7] = {3,71,9,78,20,13,18};
    // memcpy(nnParams.connectFrom, connectFrom, 7*sizeof(uint8_t));

    /* Initialise the NN structure: Test 2 NN_8x8_ 
     * Number of nodes: 24
     * Number of connections: 49    */
    // uint8_t assign[24] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,44,50,122,31,20};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {18,23};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {5.6854643953,5.4166854978,4.4695071276,2.6414439614,-6.0697657132,-3.4684758431,-3.8174169837,1.6373546717,5.7163153429,7.5273761475,0.6753311335,-5.0734045648,-8,-2.5331819721,1.0243093731,-5.1757751233,-6.0799442963,2.9452778666,-0.4377577079,-1.1463674706,-5.0354736812,-2.3887429022,-5.2648440839,5.6613246938,-0.9821943877,-6.2930716702,7.2629530138,1.6605960658,-2.2650751773,-4.4887565881,-3.4381483009,5.1789661082,4.699172285,8,0.5657086286,0.7024527972};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[13] = {5.0660609452,-0.197533224,5.7683334888,-3.2777014166,0.5019688247,-7.3251721754,-2.1982723131,3.0707197916,1.5412919367,-5.0884736986,-1.7758098904,-0.1826980181,1.0892008551};
    // memcpy(nnParams.connectWeight, connect, 13*sizeof(float));

    // uint8_t connectTo[13] = {20,31,20,31,44,20,50,20,50,44,122,20,50};
    // memcpy(nnParams.connectTo, connectTo, 13*sizeof(uint8_t));

    // uint8_t connectFrom[13] = {19,16,31,19,5,44,15,50,12,16,7,122,18};
    // memcpy(nnParams.connectFrom, connectFrom, 13*sizeof(uint8_t));

    //     /* Initialise the NN structure: Test 3 NN_8x8_ 
    //  * Number of nodes: 25
    //  * Number of connections: 47    */
    // uint8_t assign[25] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,24,29,70,135,165,19,20};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {23,24};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-3.5771787937,2.5862283547,4.0515124771,-6.635475843,7.3717118688,-1.4801757642,8,-7.7098625418,-6.4594536169,1.2774363915,3.704167533,5.5491261621,2.7468325137,5.6230148665,-8,-8,5.5818797709,5.9086396211,6.3356366059,1.0858290666,5.2058411737,1.6450376407,-2.9185671948,3.8123904435,-8,-6.5511698561,1.0278836718,6.720104633,3.7632374912,0.6538708305,-5.8552966957,-4.944389386,-5.2613166771,-5.5054832605,-3.2582508076,2.5590442474};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[11] = {-4.3329538896,1.574588618,-8,1.9622978998,4.9393872165,-4.1872943142,3.1388753478,6.1272519968,-1.6759120247,8,-1.9092086485};
    // memcpy(nnParams.connectWeight, connect, 11*sizeof(float));

    // uint8_t connectTo[11] = {24,19,29,19,70,19,135,20,165,19,29,};
    // memcpy(nnParams.connectTo, connectTo, 11*sizeof(uint8_t));

    // uint8_t connectFrom[11] = {4,24,14,29,16,70,9,135,7,165,10};
    // memcpy(nnParams.connectFrom, connectFrom, 11*sizeof(uint8_t));

    //     /* Initialise the NN structure: Test 4 NN_8x8_ 
    //  * Number of nodes: 27
    //  * Number of connections: 56    */
    // uint8_t assign[27] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,114,118,161,214,246,20,37,51,19};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {23,26};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-2.8091017067,2.6741037075,-4.6015922549,4.6325198884,4.8532889623,-3.5552078536,-4.4989754347,0.3397705017,-2.9772177469,4.4209120523,5.655344226,8,8,-4.0615316286,-3.6736217289,-8,-7.9393996088,2.4260497856,1.6931853283,-6.5873521746,4.0607365365,8,-7.427368745,-4.9067494669,-4.4042494256,-0.0995998256,-0.867260592,6.3896835486,8,1.2564087087,-5.7691738934,-3.9033576622,-3.1834793722,8,7.7997984038,-7.7926582052};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[20] = {8,5.5355986776,1.2715111607,2.2143856422,-6.7359031381,-3.4954358239,0.3055464248,5.0153428298,-2.5186258593,-4.3595752604,-5.275183538,4.7457404928,2.6898826361,4.1137965015,-6.8298555124,-3.8618539315,-3.7971203302,-1.4644760523,-1.0048656372,1.2711435629};
    // memcpy(nnParams.connectWeight, connect, 20*sizeof(float));

    // uint8_t connectTo[20] = {37,19,51,19,114,19,118,51,161,19,37,37,37,214,19,51,118,246,20,114};
    // memcpy(nnParams.connectTo, connectTo, 20*sizeof(uint8_t));

    // uint8_t connectFrom[20] = {6,37,2,51,5,114,2,118,18,161,8,114,17,15,214,11,11,7,246,17};
    // memcpy(nnParams.connectFrom, connectFrom, 20*sizeof(uint8_t));

    //     /* Initialise the NN structure: Test 5 NN_8x8_ 
    //  * Number of nodes: 27
    //  * Number of connections: 56    */
    // uint8_t assign[27] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,114,118,161,214,246,20,37,51,19};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {23,26};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-4.9007546898,1.3203493008,-4.7294102738,5.9077160884,5.2098044669,-2.454654058,-2.8116636877,0.3397705017,-4.0843101396,2.2436135432,3.5180209761,8,8,-5.1873294006,-2.1671352281,-5.5391298667,-6.3046936191,2.4260497856,-0.0965505715,-4.3286341315,5.2207873511,8,-5.8783322983,-5.9376939158,-3.7945472878,0.2357583282,-2.9284548085,5.4889263066,8,-0.8893457009,-3.5628890297,-3.2130656905,-3.1834793722,8,8,-8};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[20] = {8,5.5355986776,1.5053266891,2.2143856422,-5.7509672187,-1.6828120047,0.3055464248,2.5946709556,-1.428470387,-4.3595752604,-5.275183538,4.7457404928,1.5793512169,5.2640655228,-7.278102212,-2.9003324636,-2.3981679267,-2.9833133097,-1.4580838216,-1.183635503};
    // memcpy(nnParams.connectWeight, connect, 20*sizeof(float));

    // uint8_t connectTo[20] = {37,19,51,19,114,19,118,51,161,19,37,37,37,214,19,51,118,246,20,114};
    // memcpy(nnParams.connectTo, connectTo, 20*sizeof(uint8_t));

    // uint8_t connectFrom[20] = {6,37,2,51,5,114,2,118,18,161,8,114,17,15,214,11,11,7,246,17};
    // memcpy(nnParams.connectFrom, connectFrom, 20*sizeof(uint8_t));

    //     /* Initialise the NN structure: Test 6 NN_8x8_ 
    //  * Number of nodes: 24
    //  * Number of connections: 45    */
    // uint8_t assign[24] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,108,113,160,67,20,19};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {22,23};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-1.6002902309,-3.4410946028,1.4873686121,-5.5058337967,2.9442749484,-6.2063280467,-3.7784612223,-3.8291693968,7.5747964394,7.0532964948,4.2077685225,4.1592620269,-1.5701459065,-4.2668080471,-7.5681762453,3.1903331443,-4.9818430406,5.461090725,5.6875571325,0.5179380847,-1.9372844968,-3.8119710748,-4.7216635336,3.116348769,4.3992047228,-4.460468614,2.0283618673,1.7454249358,-7.2842241459,-2.2587671424,-4.2816086157,1.8062317346,1.0471457817,8,-2.0423914606,2.6661748121};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[9] = {5.7378582669,8,7.8711286625,8,1.1787886016,8,7.2319860396,0.3575651405,-4.6502137885};
    // memcpy(nnParams.connectWeight, connect, 9*sizeof(float));

    // uint8_t connectTo[9] = {19,108,19,113,20,160,19,67,20};
    // memcpy(nnParams.connectTo, connectTo, 9*sizeof(uint8_t));

    // uint8_t connectFrom[9] = {20,10,108,1,113,10,160,3,67};
    // memcpy(nnParams.connectFrom, connectFrom, 9*sizeof(uint8_t));

    //     /* Initialise the NN structure: Test 7 NN_8x8_ 
    //  * Number of nodes: 24
    //  * Number of connections: 48    */
    // uint8_t assign[24] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,25,48,85,219,20,19};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {22,23};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-1.1186067022,0.5507659774,4.507280889,0.4934957474,-3.8453889939,-8,-4.7062905778,4.2686464496,5.9105052192,5.2140975628,4.1311263992,6.3919050551,-7.175682906,-4.9774931682,3.1236553632,-4.5644693321,-3.3905040406,0.9703797108,2.6860813215,5.2871042613,-7.0371281339,-4.0240811801,3.9559474262,-1.8825845452,-5.5075734683,-1.7716240464,6.3064070694,2.5402810459,-4.850019419,-6.5896053226,-7.1214508166,1.4140547416,8,5.4303179023,3.1381272216,-3.5353463794};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[12] = {5.587661064,3.3205797777,-5.4093864678,8,-5.6951423809,1.5396396373,5.5390372767,4.7258953598,-6.7213389061,6.2824110076,-1.343022696,-8};
    // memcpy(nnParams.connectWeight, connect, 12*sizeof(float));

    // uint8_t connectTo[12] = {19,25,19,48,19,48,85,20,25,85,219,19};
    // memcpy(nnParams.connectTo, connectTo, 12*sizeof(uint8_t));

    // uint8_t connectFrom[12] = {20,16,25,4,48,7,12,85,18,2,6,219};
    // memcpy(nnParams.connectFrom, connectFrom, 12*sizeof(uint8_t));

    //     /* Initialise the NN structure: Test 8 NN_8x8_ 
    //  * Number of nodes: 22
    //  * Number of connections: 43    */
    // uint8_t assign[22] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,83,144,20};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {18,21};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-5.4349343736,-3.6207976931,6.8927676306,1.8182265419,2.9883885638,4.5636238031,-3.1387557126,-5.2250457514,-4.1347151851,-1.6334522291,5.4538874428,-4.5575108899,3.9309187995,1.7804603668,-1.3112252934,-7.2770876989,1.3491739264,3.6828421128,2.2053244829,6.7690223405,-4.3625090277,1.9345081961,-2.5737360937,0.8564856695,1.1333104713,-5.887006162,3.4055044055,8,6.7538663529,-8,-6.2269104934,-2.8440847875,-2.3276527282,2.595779938,-1.9730130913,-3.6418128661};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[7] = {0.5539830592,-2.35953856,0.7977921886,1.4440716658,-5.5546152363,-6.0822300232,-0.0744162367};
    // memcpy(nnParams.connectWeight, connect, 7*sizeof(float));

    // uint8_t connectTo[7] = {20,83,20,144,20,144,83};
    // memcpy(nnParams.connectTo, connectTo, 7*sizeof(uint8_t));

    // uint8_t connectFrom[7] = {19,6,83,12,144,4,2};
    // memcpy(nnParams.connectFrom, connectFrom, 7*sizeof(uint8_t));

    //     /* Initialise the NN structure: Test 9 NN_8x8_ 
    //  * Number of nodes: 24
    //  * Number of connections: 49    */
    // uint8_t assign[24] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,44,50,122,31,20};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {18,23};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {8,5.0349749069,2.4179047858,2.3092087375,-4.2366732504,-8,-3.1475808835,0.6363295366,3.3587711251,7.7552720322,-1.7599310046,-3.5044354441,-2.381627982,-4.0228221508,3.0117098112,-4.5439825289,-5.6894914592,3.3465064234,1.1191829484,-0.0763807632,-4.7957764531,-0.0434992361,-6.8383969973,1.0992929922,2.5848622904,-5.9531535883,6.2512441367,5.4430558578,-4.2223208987,-0.9010613251,-4.2006203999,-0.0955931084,6.1058073872,6.1859822665,-0.9504762462,0.1623928027};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[13] = {7.7186534879,2.711642716,5.6659600135,-0.8071395545,2.3298583292,-6.6503443508,-8,0.6190197944,0.3159551201,-6.6213725113,-2.6788729314,0.2522953659,2.828313211};
    // memcpy(nnParams.connectWeight, connect, 13*sizeof(float));

    // uint8_t connectTo[13] = {20,31,20,31,44,20,50,20,50,44,122,20,50};
    // memcpy(nnParams.connectTo, connectTo, 13*sizeof(uint8_t));

    // uint8_t connectFrom[13] = {19,16,31,19,5,44,15,50,12,16,7,122,18};
    // memcpy(nnParams.connectFrom, connectFrom, 13*sizeof(uint8_t));

    //     /* Initialise the NN structure: Test 10 NN_8x8_ 
    //  * Number of nodes: 25
    //  * Number of connections: 53    */
    uint8_t assign[25] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,81,176,79,28,20,24,19};
    memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    uint8_t assign2[2] = {22,24};
    memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    float connectionsInit[36] = {0.9207661265,0.2916641679,6.3894245956,-5.876807089,0.7765806776,8,-6.3178346348,-2.7822010832,-3.3798155011,-3.714803467,6.0853213039,8,5.6761756275,-3.7085986736,-0.5377148594,-0.5036210314,-3.506721978,-3.6062296346,5.9613351566,2.1827223686,7.8944220409,1.490843575,-7.6499754755,3.1584498902,-5.713109873,-0.169567458,0.2634392814,-1.2549521905,8,-1.9202406583,-4.0894472533,-8,-0.3977362838,-1.1493322412,-2.5336723479,-0.9165736135};
    memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    float connect[17] = {-2.3753024406,7.0288431242,1.8071825873,2.1077574136,-6.6067638161,-1.8846781707,0.2463254068,8,-3.8956754525,2.9413329306,-4.4801971153,-4.3121287736,1.4353793518,-8,2.7579210541,8,0.8827557309};
    memcpy(nnParams.connectWeight, connect, 17*sizeof(float));

    uint8_t connectTo[17] = {24,19,24,24,81,24,20,176,20,176,24,79,19,81,28,20,24};
    memcpy(nnParams.connectTo, connectTo, 17*sizeof(uint8_t));

    uint8_t connectFrom[17] = {10,24,5,17,10,81,81,16,176,4,8,8,79,11,2,28,18};
    memcpy(nnParams.connectFrom, connectFrom, 17*sizeof(uint8_t));



    /* ________________ NN for larger test Areas ____________________ */

    /* Initialise the NN structure: Test 1
     * Number of nodes: 26
     * Number of connections: 51    */

    // uint8_t assign[26] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,21,68,93,210,19,108,153,20};
    // memcpy(nnParams.node_ID, assign, 26*sizeof(uint8_t));

    // uint8_t assign2[2] = {22,25};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {7.806380957,5.528580557,0.580185567,-0.892800512,-1.560863676,-5.299624156,-4.744623565,7.188228362,-1.393527294,-5.525020404,2.879480339,1.522226165,3.768690915,-8,-3.174616974,-4.270717264,4.399241565,-6.88788973,1.85223179,7.267868926,-1.837650023,-8,-1.754804325,5.439443797,6.399576228,1.87862188,4.723880129,-4.245715255,0.678777551,4.695496426,-2.394260424,-7.186473925,1.838699615,3.85452967,-4.274725797,-8};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[15] = {-2.097217819,-2.63461102,1.054388957,-1.586494309,-0.856340471,-3.970069819,-2.066530808,-1.077418409,-1.693239987,-1.92564801,2.955273135,-8,-3.16237498,2.989823099,-1.877785137};
    // memcpy(nnParams.connectWeight, connect, 15*sizeof(float));

    // uint8_t connectTo[15] = {21,20,68,20,93,20,108,20,153,20,21,93,153,210,19};
    // memcpy(nnParams.connectTo, connectTo, 15*sizeof(uint8_t));

    // uint8_t connectFrom[15] = {7,21,1,68,18,93,93,108,10,153,3,7,68,7,210};
    // memcpy(nnParams.connectFrom, connectFrom, 15*sizeof(uint8_t));

    /* Initialise the NN structure: Test 2 
     * Number of nodes: 24
     * Number of connections: 47    */
    // uint8_t assign[24] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,20,24,29,117,70,19};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {23,18};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-2.028513526,-0.999471241,5.81441121,-3.123068448,5.356148261,-5.667570271,5.345452378,-4.915354194,-4.901790473,1.785882518,4.429722207,6.163249646,3.552724382,1.182559498,-5.796064955,-8,6.869960564,2.08647117,5.737571116,5.050365447,4.909976767,-5.899512268,0.907940093,0.488276117,-5.807266878,-0.557835599,8,6.89630141,8,-5.235546742,0.727357913,-6.431080182,-3.14359851,3.210467835,-3.862963,-8};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[11] = {-3.142701103,-6.102707703,-4.103714529,6.7399511,4.713996883,-3.978940775,2.886957094,6.715348566,-4.858884879,2.532084194,-0.980546984};
    // memcpy(nnParams.connectWeight, connect, 11*sizeof(float));

    // uint8_t connectTo[11] = {24,19,29,19,70,19,117,19,117,70,70};
    // memcpy(nnParams.connectTo, connectTo, 11*sizeof(uint8_t));

    // uint8_t connectFrom[11] = {4,24,14,29,16,70,17,117,1,18,29};
    // memcpy(nnParams.connectFrom, connectFrom, 11*sizeof(uint8_t));

    /* Initialise the NN structure: Test 1 NN_strucute 2 
     * Number of nodes: 27
     * Number of connections: 58    */
    // uint8_t assign[27] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,83,121,178,236,35,172,20,44,19};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {26,24};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {4.429598022,2.620497746,4.649862413,-2.875709262,-4.998229997,-3.539988695,-0.584947316,-1.536974365,5.693284409,5.022521425,6.439298515,-4.29526476,-8,-6.723690097,-1.402932056,4.075383274,2.916538186,2.565958252,-2.53011018,-4.336795246,-4.249729181,-1.229806892,5.711791235,0.719153938,-0.325743999,8,5.740576572,1.567148228,-5.525930682,-5.45198859,-4.899131613,-4.7954142,7.421623344,5.934113306,-6.512051155,-5.398208762};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[22] = {8,7.88385423,-2.81772193,5.514490078,3.80777428,8,-5.350441006,8,-0.304940694,5.658624981,3.938816676,-4.501366503,1.030686962,-8,-4.981156759,4.66938234,-6.927775191,7.0838781,-6.588890477,0.35185311,-0.875405002,2.454297922};
    // memcpy(nnParams.connectWeight, connect, 22*sizeof(float));

    // uint8_t connectTo[22] = {35,20,44,19,44,44,44,83,20,83,35,121,20,83,44,172,20,178,19,236,19,236};
    // memcpy(nnParams.connectTo, connectTo, 22*sizeof(uint8_t));

    // uint8_t connectFrom[22] = {10,35,9,44,14,2,10,12,83,3,83,3,121,11,20,121,172,15,178,12,236,18};
    // memcpy(nnParams.connectFrom, connectFrom, 22*sizeof(uint8_t));

    /* Initialise the NN structure: Test 2 NN_strucute 2 
     * Number of nodes: 27
     * Number of connections: 58    */
    // uint8_t assign[27] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,83,121,178,236,35,172,20,44,19};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {26,24};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {3.722831694,2.468246222,6.347445077,-5.283541189,-3.092373869,-3.539988695,-2.161156838,-1.536974365,4.945351589,5.022521425,6.439298515,-6.432958895,-8,-8,0.493024976,3.220237551,4.663644829,4.602807884,-2.033753702,-6.404339432,-3.909585713,0.921756962,3.668670064,1.897664652,-0.325743999,6.323796722,6.155256458,3.039111206,-5.064440582,-5.45198859,-6.750229573,-4.765182052,5.672868388,8,-7.375948979,-4.482788061};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[22] = {8,7.88385423,-2.729948666,3.578418987,4.080843085,6.534973743,-6.990195882,8,0.566417367,5.658624981,4.166694522,-4.501366503,0.259520358,-8,-3.140425227,2.415482306,-8,7.0838781,-5.491710903,0.35185311,-2.93416561,0.308787725};
    // memcpy(nnParams.connectWeight, connect, 22*sizeof(float));

    // uint8_t connectTo[22] = {35,20,44,19,44,44,44,83,20,83,35,121,20,83,44,172,20,178,19,236,19,236};
    // memcpy(nnParams.connectTo, connectTo, 22*sizeof(uint8_t));

    // uint8_t connectFrom[22] = {10,35,9,44,14,2,10,12,83,3,83,3,121,11,20,121,172,15,178,12,236,18};

/* ______________________________________________________ NN for 8x8 Mission space __________________________________________ */
    /* Initialise the NN structure: Test 1 NN_8x8_ 
     * Number of nodes: 22
     * Number of connections: 45    */
    // uint8_t assign[22] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,57,19,33,20};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {19,21};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {6.68397798,-7.011096498,4.22477405,1.37334448,-4.972096588,3.369649268,-4.39394876,7.701226167,4.330178904,5.945787658,-4.620123518,-3.200256073,-7.832427002,2.269887211,-5.621190615,4.069043592,3.493257348,2.485031934,5.700566809,-2.551691791,-2.613813953,-8,-8,5.209014955,3.141516696,-1.489104496,-8,3.617203409,-5.811839292,-7.964850366,-4.866802385,2.578867209,6.644081919,4.133378381,8,4.822844973};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[9] = {8,-5.945602839,-1.607090468,6.906052242,-5.86818043,1.214671762,0.519872311,2.243342479,6.574267786};
    // memcpy(nnParams.connectWeight, connect, 9*sizeof(float));

    // uint8_t connectTo[9] = {33,20,57,19,57,33,33,33,57};
    // memcpy(nnParams.connectTo, connectTo, 9*sizeof(uint8_t));

    // uint8_t connectFrom[9] = {14,33,6,57,14,19,5,2,11};
    // memcpy(nnParams.connectFrom, connectFrom, 9*sizeof(uint8_t));

     /* Initialise the NN structure: Test 2 NN_8x8_ 
     * Number of nodes: 24
     * Number of connections: 44    */
    // uint8_t assign[24] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,25,28,116,20,37,19};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {21,23};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {5.283783269,-1.299470374,5.41218604,-7.234246276,-3.300530603,-0.932467532,-6.052529676,1.704697964,-6.358833382,8,8,-2.854298496,-8,-1.582748166,-1.914545222,8,4.355598982,2.627215527,5.530937664,-4.293201699,-7.912133623,-4.560515003,-2.432927559,6.428544479,3.236962558,7.434877323,-6.141023524,-2.004435827,-3.654535521,-7.174535472,-7.00926667,6.136937081,0.3445915,2.93038681,2.068986956,5.76611966};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[8] = {-6.640326374,0.122216172,3.519665279,-2.839629468,-1.236220431,3.307892439,-0.043001854,-8};
    // memcpy(nnParams.connectWeight, connect, 8*sizeof(float));

    // uint8_t connectTo[8] = {25,20,28,20,37,19,116,37};
    // memcpy(nnParams.connectTo, connectTo, 8*sizeof(uint8_t));

    // uint8_t connectFrom[8] = {14,25,18,28,13,37,13,116};
    // memcpy(nnParams.connectFrom, connectFrom, 8*sizeof(uint8_t));   

     /* Initialise the NN structure: Test 3 NN_8x8_ 
     * Number of nodes: 22
     * Number of connections: 43    */
    // uint8_t assign[22] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,25,48,20,19};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {20,21};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {0.758276153,6.613760206,7.641370586,2.05194775,-1.945134748,-2.536768723,-6.357403733,8,5.06708865,3.007440016,1.849046149,-1.269361137,-5.247653742,-7.54289905,4.577923116,-6.121182486,0.570614195,-4.308993171,3.584171836,-6.699301525,0.518360243,-6.034676632,-3.773337785,2.869210323,0.662955925,0.960822812,2.909667741,-1.237123547,-8,-6.464401628,-4.719040365,-1.776453679,5.255969655,2.298880804,-0.054299528,8};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[7] = {1.679820357,4.134198285,3.906736078,-1.660334104,0.823279933,-5.423408749,7.729128767};
    // memcpy(nnParams.connectWeight, connect, 7*sizeof(float));

    // uint8_t connectTo[7] = {19,25,19,48,19,48,20};
    // memcpy(nnParams.connectTo, connectTo, 7*sizeof(uint8_t));

    // uint8_t connectFrom[7] = {20,16,25,4,48,7,25};
    // memcpy(nnParams.connectFrom, connectFrom, 7*sizeof(uint8_t));

     /* Initialise the NN structure: Test 4 NN_8x8_ 
     * Number of nodes: 22
     * Number of connections: 43    */
    // uint8_t assign[22] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,31,50,20};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {18,21};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {3.279733848,-7.631855603,2.668146918,-3.235513543,8,-4.931743709,-6.581992596,-0.803942644,-5.387523815,-5.074530671,7.207404539,7.152550871,-0.680557557,3.671292441,1.539872788,-4.492544346,2.048400808,2.025372548,-3.033840625,-4.08761024,5.239833735,-3.497375993,0.88174322,3.973545839,-5.163879649,7.021978435,4.357408403,2.058292456,8,-1.831572606,-7.655172552,-4.292540899,-7.623346636,-5.881747338,0.359910324,-4.837460245};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[7] = {-8,-0.164135768,-3.353320967,-3.659393426,-8,-1.689902957,-1.479563598};
    // memcpy(nnParams.connectWeight, connect, 7*sizeof(float));

    // uint8_t connectTo[7] = {31,20,31,50,20,50,50};
    // memcpy(nnParams.connectTo, connectTo, 7*sizeof(uint8_t));

    // uint8_t connectFrom[7] = {1,31,5,18,50,15,11};
    // memcpy(nnParams.connectFrom, connectFrom, 7*sizeof(uint8_t));   

     /* Initialise the NN structure: Test 5 NN_8x8_ 
     * Number of nodes: 22
     * Number of connections: 50    */
    // uint8_t assign[22] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,21,39,150,19,20};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {21,22};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-5.124972732,-6.929673257,7.888463717,-3.437556599,8,0.307434254,-2.84016483,-1.580916869,5.792891629,-2.863319515,0.941115971,2.34254156,2.834293863,-0.980123869,-4.822482104,2.548310683,-2.641666434,6.07057365,4.475958957,3.537584088,2.133353433,4.497484317,-3.317510632,5.097411655,1.375996952,2.968176945,6.273232312,-2.605742252,8,0.91879182,1.354336035,-7.07569133,-4.859222196,-0.315621854,-5.434992854,-1.675551245};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[14] = {-1.283912699,0.205602776,-4.600894961,5.911469358,6.099048098,-5.406181204,1.311094481,3.94865491,-7.356475734,-0.860274048,4.766271655,2.057908763,-6.222860338,-2.838321019};
    // memcpy(nnParams.connectWeight, connect, 14*sizeof(float));

    // uint8_t connectTo[14] = {21,19,21,39,20,39,39,21,21,21,20,150,19,21};
    // memcpy(nnParams.connectTo, connectTo, 14*sizeof(uint8_t));

    // uint8_t connectFrom[14] = {14,21,6,11,39,13,14,10,13,1,19,9,150,16};
    // memcpy(nnParams.connectFrom, connectFrom, 14*sizeof(uint8_t));

/* ______________________________________________________ NN for 7x7 Mission space __________________________________________ */

     /* Initialise the NN structure: Test 1 NN_7x7_ 
     * Number of nodes: 27
     * Number of connections: 54    */
    // uint8_t assign[27] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,33,59,194,218,228,19,89,22,20};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {23,26};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-3.717117455,2.192775694,0.482336913,1.884638178,-4.028251312,-4.547274807,1.115480152,-8,5.314087188,3.103122405,5.832575527,-1.563995807,0.902591347,-8,-3.945750057,-7.784112785,5.309103927,4.147914241,-3.39928983,3.479813004,1.090538512,5.399106504,-7.341632111,-6.645377617,4.036422052,1.092206981,4.543172662,1.170054056,-8,-7.616365769,-2.54404365,-8,5.581609015,8,-0.537830795,6.628168546};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[18] = {-1.173190846,4.425084315,2.171182185,-3.803040364,4.682675774,-7.69496467,-7.59763961,-1.326935713,-6.808741027,-3.269899923,-4.823651445,-6.28116479,4.077206046,-2.809362887,-0.386280029,-5.354598277,-0.812486927,-1.665935606};
    // memcpy(nnParams.connectWeight, connect, 18*sizeof(float));

    // uint8_t connectTo[18] = {22,20,33,19,59,20,89,20,22,33,89,194,89,22,218,20,228,19};
    // memcpy(nnParams.connectTo, connectTo, 18*sizeof(uint8_t));

    // uint8_t connectFrom[18] = {10,22,10,33,13,59,6,89,19,8,5,5,194,17,13,218,12,228};
    // memcpy(nnParams.connectFrom, connectFrom, 18*sizeof(uint8_t));

}

bool testDistance(){
    /* Conversion between coordinate systems */
    // float a = 0.827559;
    // float b = 0.5613786;
    // float c = -3.903735;
    // float d = 1.0823;

    struct EnuCoor_f *pos = stateGetPositionEnu_f();
    // float tempX=(*pos).x;
    // float tempY=(*pos).y;
    // (*pos).x = a*tempX+b*tempY+c;
    // (*pos).y = -b*tempX+a*tempY+d;

    bool output = 0;

    for(uint8_t i=0;i<MS_SWARM_SIZE;i++){
        if(i != MS_CURRENT_ID){
            float dist = ((*pos).x-msParams.uavs[i].x)*((*pos).x-msParams.uavs[i].x)+((*pos).y-msParams.uavs[i].y)*((*pos).y-msParams.uavs[i].y);
            if (dist <= 1) {
                output = 1;
            }
        }
    }

    return output;
}

bool outArea(){
    bool output = 0;

    struct EnuCoor_f *pos = stateGetPositionEnu_f();
    if ((*pos).x < 0 || (*pos).y < 0 || (*pos).y > MS_BREDTH || (*pos).x > MS_LENGTH) {
        output = 1;
    }

    return output;
}

/*void neural_network_periodic(void) {
    
}*/
