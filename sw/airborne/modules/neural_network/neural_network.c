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
#define MS_NUM_CONNECT 44
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
    uint8_t MS[(uint8_t) (MS_BREDTH/MS_GRID_RES)][(uint8_t) (MS_LENGTH/MS_GRID_RES)];
    // uint8_t MS[20][20];
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
    // float a = 0.827559;
    // float b = 0.5613786;
    // float c = -3.903735;
    // float d = 1.0823;

    struct EnuCoor_f *pos = stateGetPositionEnu_f();
    // float tempX=(*pos).x;
    // float tempY=(*pos).y;
    // (*pos).x = a*tempX+b*tempY+c;
    // (*pos).y = -b*tempX+a*tempY+d;

    for(uint8_t i=0;i<MS_SWARM_SIZE;i++){
        float temp[2];
        getPos_UWB(i,temp);
        msParams.uavs[i].x = temp[0];
        msParams.uavs[i].y = temp[1];

        // msParams.uavs[i].x = a*temp[0]+b*temp[1]+c;
        // msParams.uavs[i].y = b*temp[0]-a*temp[1]+d;
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
        nnParams.node_out[3] = sqrtf(((*pos).x-MS_GRID_RES*(currentCell_x+1))*((*pos).x-MS_GRID_RES*(currentCell_x+1))+((*pos).y-MS_GRID_RES*(currentCell_y))*((*pos).y-MS_GRID_RES*(currentCell_y)));
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
    // guidance_h_set_guided_body_vel(outputs[0],outputs[1]);
    guidance_h_set_guided_vel(outputs[0],outputs[1]); 

    // return 0;
}

void ageMS(void){
    /* Conversion between coordinate systems */
    // float a = 0.827559;
    // float b = 0.5613786;
    // float c = -3.903735;
    // float d = 1.0823;

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
    for (uint8_t agentNum = 0; agentNum < MS_SWARM_SIZE; agentNum++){
        uint8_t currentCell_x;
        uint8_t currentCell_y;
        if(agentNum == MS_CURRENT_ID){
            struct EnuCoor_f *pos = stateGetPositionEnu_f();
            // float tempX=(*pos).x;
            // float tempY=(*pos).y;
            // (*pos).x = a*tempX+b*tempY+c;
            // (*pos).y = -b*tempX+a*tempY+d;

            currentCell_x = (uint8_t) ((*pos).x/MS_GRID_RES);
            currentCell_y = (uint8_t) ((*pos).y/MS_GRID_RES);
        }
        else{
            currentCell_x = (uint8_t) (msParams.uavs[agentNum].x/MS_GRID_RES);
            currentCell_y = (uint8_t) (msParams.uavs[agentNum].y/MS_GRID_RES);
        }
        if(msParams.MS[currentCell_y][currentCell_x] != 0) {
            msParams.MS[currentCell_y][currentCell_x] = 100;
        }
    }
}

void neural_network_init(void) {
    /** Initialize the Mission Space age grid */
    for (uint8_t x = 0; x < MS_LENGTH/MS_GRID_RES; x++){
        for (uint8_t y = 0; y < MS_BREDTH/MS_GRID_RES; y++) {
            if( y == 0 || y == MS_BREDTH/MS_GRID_RES-MS_GRID_RES) {
                msParams.MS[y][x] = 0;
            }
            else if(x == 0 || x == MS_LENGTH/MS_GRID_RES - MS_GRID_RES){
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
    msParams.uavs[1].x = 0.5;
    msParams.uavs[1].y = 0.5;
    // msParams.uavs[2].x = 11;
    // msParams.uavs[2].y = 11;

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
    uint8_t assign[24] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,25,28,116,20,37,19};
    memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    uint8_t assign2[2] = {21,23};
    memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    float connectionsInit[36] = {5.283783269,-1.299470374,5.41218604,-7.234246276,-3.300530603,-0.932467532,-6.052529676,1.704697964,-6.358833382,8,8,-2.854298496,-8,-1.582748166,-1.914545222,8,4.355598982,2.627215527,5.530937664,-4.293201699,-7.912133623,-4.560515003,-2.432927559,6.428544479,3.236962558,7.434877323,-6.141023524,-2.004435827,-3.654535521,-7.174535472,-7.00926667,6.136937081,0.3445915,2.93038681,2.068986956,5.76611966};
    memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    float connect[8] = {-6.640326374,0.122216172,3.519665279,-2.839629468,-1.236220431,3.307892439,-0.043001854,-8};
    memcpy(nnParams.connectWeight, connect, 8*sizeof(float));

    uint8_t connectTo[8] = {25,20,28,20,37,19,116,37};
    memcpy(nnParams.connectTo, connectTo, 8*sizeof(uint8_t));

    uint8_t connectFrom[8] = {14,25,18,28,13,37,13,116};
    memcpy(nnParams.connectFrom, connectFrom, 8*sizeof(uint8_t));   

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
