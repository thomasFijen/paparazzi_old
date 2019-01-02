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
#include "mcu_periph/sys_time_arch.h"
#include "subsystems/electrical.h"

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

#ifndef MS_MAX_VEL
#define MS_MAX_VEL 1.0f
#endif

#ifndef MS_SENSOR_RANGE
#define MS_SENSOR_RANGE 4.0f
#endif

#ifndef MS_CURRENT_ID
#define MS_CURRENT_ID MS_SWARM_SIZE-1
#endif

#ifndef MS_FOOTPRINT
#define MS_FOOTPRINT 0.75
#endif

#ifndef MS_DIST_THRESH
#define MS_DIST_THRESH 0.9
#endif

#ifndef MS_DIST_THRESH_2
#define MS_DIST_THRESH_2 1.2
#endif

#ifndef MS_DEPOT_X
#define MS_DEPOT_X 1.0
#endif

#ifndef MS_DEPOT_Y
#define MS_DEPOT_Y 1.0
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

#define PI 3.14159265


static struct MS_Struct msParams;
static struct NN_struct nnParams;

static uint8_t counterTemp = 0;

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

    bool surveillanceOn;    //Flag indicating that the MAV can perform the surveillance task
    float depotNotFree;         //Flag indicating that the depot is unused
    bool land;              // Flag showing that the MAV must land
    uint8_t avoid;             // Avoid flag. Indicates the MAV is performing an avoidance move
    uint32_t currentTime;

    float bt_State;
};

/** Mission Space Parameter structure */
struct MS_Struct {
    // uint8_t MS[(uint8_t) (MS_BREDTH/MS_GRID_RES)][(uint8_t) (MS_LENGTH/MS_GRID_RES)];
    // uint8_t MS[14][14];
    float MS[14][14];
    float sensorRange;
	struct EnuCoor_f uavs[MS_SWARM_SIZE];
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

    // struct EnuCoor_f *pos_CalcIn = stateGetPositionEnu_f();
    // float tempX=(*pos_CalcIn).x;
    // float tempY=(*pos_CalcIn).y;
    // (*pos_CalcIn).x = a*tempX+b*tempY+c;
    // (*pos_CalcIn).y = -b*tempX+a*tempY+d;

    struct EnuCoor_f *pos_CalcIn = stateGetPositionEnu_f();
    (*pos_CalcIn).x=msParams.uavs[MS_CURRENT_ID].x;
    (*pos_CalcIn).y=msParams.uavs[MS_CURRENT_ID].y;
    // printf("\nCalc In: %f,%f",(*pos_CalcIn).x,(*pos_CalcIn).y);

    // for(uint8_t i=0;i<MS_SWARM_SIZE;i++){
    //     float temp[3];
    //     getPos_UWB((i+2),temp);             // MAGIC NUMBER: Used to relate the index i with the ID from UWB
    //     nnParams.depotNotFree = temp[2];

    //     msParams.uavs[i].x = temp[0];
    //     msParams.uavs[i].y = temp[1];

    //     msParams.uavs[i].x = a*temp[0]+b*temp[1]+c;
    //     msParams.uavs[i].y = -b*temp[0]+a*temp[1]+d;
    // }
    
    /* Default range values */
    for (uint8_t i = 0; i < MS_NUM_INPUTS; i++) {
        nnParams.node_out[i] = 0.0;
    }

    uint8_t currentCell_x;
    uint8_t currentCell_y;
    if ((*pos_CalcIn).x >= 0 && (*pos_CalcIn).x <= MS_LENGTH && (*pos_CalcIn).y >= 0 && (*pos_CalcIn).y <= MS_BREDTH) {
        /* Get current cell index */
        currentCell_x = (uint8_t) ((*pos_CalcIn).x/MS_GRID_RES);
        currentCell_y = (uint8_t) ((*pos_CalcIn).y/MS_GRID_RES);

        /* Antennae Function values */
        uint8_t numCells = msParams.sensorRange / MS_GRID_RES;

        nnParams.node_out[0] = MS_GRID_RES*(currentCell_y+1)-(*pos_CalcIn).y;
        nnParams.node_out[8] = (100-msParams.MS[currentCell_y][currentCell_x])/100.0;
        for (uint8_t i = 1; i <= numCells; i++) {
            if (((*pos_CalcIn).y+i*MS_GRID_RES) > MS_BREDTH) {
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

        nnParams.node_out[2] = MS_GRID_RES*(currentCell_x+1)-(*pos_CalcIn).x;
        nnParams.node_out[10] = (100-msParams.MS[currentCell_y][currentCell_x])/100.0;
        for (uint8_t i = 1; i <= numCells; i++) {
            if (((*pos_CalcIn).x+i*MS_GRID_RES) > MS_LENGTH) {
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

        nnParams.node_out[4] = (*pos_CalcIn).y - MS_GRID_RES*(currentCell_y);
        nnParams.node_out[12] = (100-msParams.MS[currentCell_y][currentCell_x])/100.0;
        for (uint8_t i = 1; i <= numCells; i++) {
            if (((*pos_CalcIn).y-i*MS_GRID_RES) < 0) {
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

        nnParams.node_out[6] = (*pos_CalcIn).x-MS_GRID_RES*(currentCell_x);
        nnParams.node_out[14] = (100-msParams.MS[currentCell_y][currentCell_x])/100.0;
        for (uint8_t i = 1; i <= numCells; i++) {
            if (((*pos_CalcIn).x-i*MS_GRID_RES) < 0) {
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

        nnParams.node_out[1] = sqrtf(((*pos_CalcIn).x-MS_GRID_RES*(currentCell_x+1))*((*pos_CalcIn).x-MS_GRID_RES*(currentCell_x+1))+((*pos_CalcIn).y-MS_GRID_RES*(currentCell_y+1))*((*pos_CalcIn).y-MS_GRID_RES*(currentCell_y+1)));
        nnParams.node_out[9] = (100-msParams.MS[currentCell_y][currentCell_x])/100.0;
        for(uint8_t i = 1; i <= numCells; i++) {
            if (((*pos_CalcIn).y+i*MS_GRID_RES) > MS_BREDTH || ((*pos_CalcIn).x+i*MS_GRID_RES) > MS_LENGTH) {
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

        nnParams.node_out[3] = sqrtf(((*pos_CalcIn).x-MS_GRID_RES*(currentCell_x+1))*((*pos_CalcIn).x-MS_GRID_RES*(currentCell_x+1))+((*pos_CalcIn).y-MS_GRID_RES*(currentCell_y))*((*pos_CalcIn).y-MS_GRID_RES*(currentCell_y)));
        // nnParams.node_out[3] = sqrtf(((*pos).x-MS_GRID_RES*(currentCell_x+1))*((*pos).x-MS_GRID_RES*(currentCell_x+1))+((*pos).y-MS_GRID_RES*(currentCell_y))*((*pos).y-MS_GRID_RES*(currentCell_y)));
        nnParams.node_out[11] = (100-msParams.MS[currentCell_y][currentCell_x])/100.0;
        for(uint8_t i = 1; i <= numCells; i++) {
            if (((*pos_CalcIn).y-i*MS_GRID_RES) < 0 || ((*pos_CalcIn).x+i*MS_GRID_RES) > MS_BREDTH) {
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

        nnParams.node_out[5] = sqrtf(((*pos_CalcIn).x-MS_GRID_RES*(currentCell_x))*((*pos_CalcIn).x-MS_GRID_RES*(currentCell_x))+((*pos_CalcIn).y-MS_GRID_RES*(currentCell_y))*((*pos_CalcIn).y-MS_GRID_RES*(currentCell_y)));
        nnParams.node_out[13] = (100-msParams.MS[currentCell_y][currentCell_x])/100.0;
        for(uint8_t i = 1; i <= numCells; i++) {
            if (((*pos_CalcIn).y-i*MS_GRID_RES) < 0 || ((*pos_CalcIn).x-i*MS_GRID_RES) < 0) {
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

        nnParams.node_out[7] = sqrtf(((*pos_CalcIn).x-MS_GRID_RES*(currentCell_x))*((*pos_CalcIn).x-MS_GRID_RES*(currentCell_x))+((*pos_CalcIn).y-MS_GRID_RES*(currentCell_y+1))*((*pos_CalcIn).y-MS_GRID_RES*(currentCell_y+1)));
        nnParams.node_out[15] = (100-msParams.MS[currentCell_y][currentCell_x])/100.0;
        for(uint8_t i = 1; i <= numCells; i++) {
            if (((*pos_CalcIn).y+i*MS_GRID_RES) > MS_BREDTH || ((*pos_CalcIn).x-i*MS_GRID_RES) < 0) {
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
                        float distance = sqrtf((msParams.uavs[i].x-(*pos_CalcIn).x)*(msParams.uavs[i].x-(*pos_CalcIn).x)+(msParams.uavs[i].y-(*pos_CalcIn).y)*(msParams.uavs[i].y-(*pos_CalcIn).y));
                        if(agentCell_y >= currentCell_y && distance < nnParams.node_out[0]){
                            nnParams.node_out[0] = distance;
                            nnParams.node_out[8] = (100-msParams.MS[agentCell_y][agentCell_x])/100.0;
                        } else if (agentCell_y < currentCell_y && distance < nnParams.node_out[4]) {
                            nnParams.node_out[4] = distance;
                            nnParams.node_out[12] = (100-msParams.MS[agentCell_y][agentCell_x])/100.0;
                        }
                    }
                    else if (agentCell_y == currentCell_y){
                        float distance = sqrtf((msParams.uavs[i].x-(*pos_CalcIn).x)*(msParams.uavs[i].x-(*pos_CalcIn).x)+(msParams.uavs[i].y-(*pos_CalcIn).y)*(msParams.uavs[i].y-(*pos_CalcIn).y));
                        if(agentCell_x >= currentCell_x && distance < nnParams.node_out[2]){
                            nnParams.node_out[2] = distance;
                            nnParams.node_out[10] = (100-msParams.MS[agentCell_y][agentCell_x])/100.0;
                        } else if (agentCell_x < currentCell_x && distance < nnParams.node_out[6]) {
                            nnParams.node_out[6] = distance;
                            nnParams.node_out[14] = (100-msParams.MS[agentCell_y][agentCell_x])/100.0;
                        }
                    }
                    else if ((agentCell_y-currentCell_y) == (agentCell_x-currentCell_x) && agentCell_x >= currentCell_x ) {
                        float distance = sqrtf((msParams.uavs[i].x-(*pos_CalcIn).x)*(msParams.uavs[i].x-(*pos_CalcIn).x)+(msParams.uavs[i].y-(*pos_CalcIn).y)*(msParams.uavs[i].y-(*pos_CalcIn).y));
                        if(distance < nnParams.node_out[1]){
                            nnParams.node_out[1] = distance;
                            nnParams.node_out[9] = (100-msParams.MS[agentCell_y][agentCell_x])/100.0;
                        }
                    } else if ((currentCell_y-agentCell_y) == (agentCell_x-currentCell_x) && agentCell_x >= currentCell_x){
                        float distance = sqrtf((msParams.uavs[i].x-(*pos_CalcIn).x)*(msParams.uavs[i].x-(*pos_CalcIn).x)+(msParams.uavs[i].y-(*pos_CalcIn).y)*(msParams.uavs[i].y-(*pos_CalcIn).y));
                        if(distance < nnParams.node_out[3]){
                            nnParams.node_out[3] = distance;
                            nnParams.node_out[11] = (100-msParams.MS[agentCell_y][agentCell_x])/100.0;
                        }
                    } else if ((currentCell_y-agentCell_y) == (currentCell_x-agentCell_x) && agentCell_x < currentCell_x) {
                        float distance = sqrtf((msParams.uavs[i].x-(*pos_CalcIn).x)*(msParams.uavs[i].x-(*pos_CalcIn).x)+(msParams.uavs[i].y-(*pos_CalcIn).y)*(msParams.uavs[i].y-(*pos_CalcIn).y));
                        if(distance < nnParams.node_out[5]){
                            nnParams.node_out[5] = distance;
                            nnParams.node_out[13] = (100-msParams.MS[agentCell_y][agentCell_x])/100.0;
                        }
                    } else if ((agentCell_y-currentCell_y) == (currentCell_x-agentCell_x) && agentCell_x < currentCell_x) {
                        float distance = sqrtf((msParams.uavs[i].x-(*pos_CalcIn).x)*(msParams.uavs[i].x-(*pos_CalcIn).x)+(msParams.uavs[i].y-(*pos_CalcIn).y)*(msParams.uavs[i].y-(*pos_CalcIn).y));
                        if(distance < nnParams.node_out[7]){
                            nnParams.node_out[7] = distance;
                            nnParams.node_out[15] = (100-msParams.MS[agentCell_y][agentCell_x])/100.0;
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

        if ((*pos_CalcIn).x < 0) { 
            if ((*pos_CalcIn).y < 0) {
                nnParams.node_out[1] = msParams.sensorRange;
                nnParams.node_out[9] = 1.0;
            }else if ((*pos_CalcIn).y >= 0 && (*pos_CalcIn).y <= MS_BREDTH) {
                nnParams.node_out[2] = msParams.sensorRange;
                nnParams.node_out[10] = 1.0;
            }else {
                nnParams.node_out[3] = msParams.sensorRange;
                nnParams.node_out[11] = 1.0;
            }
        }else if ((*pos_CalcIn).x >=0 && (*pos_CalcIn).x <= MS_LENGTH){ 
            if ((*pos_CalcIn).y < 0) { 
                nnParams.node_out[0] = msParams.sensorRange;
                nnParams.node_out[8] = 1.0;
            }else if ((*pos_CalcIn).y > MS_BREDTH) { 
                nnParams.node_out[4] = msParams.sensorRange;
                nnParams.node_out[12] = 1.0;
            }
        }else { 
            if ((*pos_CalcIn).y < 0) { 
                nnParams.node_out[7] = msParams.sensorRange;
                nnParams.node_out[15] = 1.0;
            }else if ((*pos_CalcIn).y >= 0 && (*pos_CalcIn).y <= MS_BREDTH) { 
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
}

/** This function implements the activation function of the NN */
float activationFunction(float x) {
    float output = (expf(x)-expf(-x))/(expf(x)+expf(-x));

    return output;
}

/* This function is used to turn the calcNN on and off. Only used if calcNN is defined as a periodic function */
void runSurviellance(){
    if(nnParams.surveillanceOn == TRUE) {
        nnParams.surveillanceOn = FALSE;
    } else {
        nnParams.surveillanceOn = TRUE;
    }
}

/** This function calculates the outputs of the NN */
void calcNN() {
    if (nnParams.surveillanceOn == TRUE) {
        uint8_t recurrentNN = 0; // Boolean showing whether the NN is a recurrent network or not
        nnParams.currentTime = get_sys_time_msec();

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


        // THIS IS FOR GUIDED MODE
        
        // guidance_h_set_guided_vel(outputs[0],outputs[1]); 
        // guidance_h_set_guided_vel(outputs[1],outputs[0]); //This is for optitrack, x and y swapped around!!!
        guidance_h_set_guided_body_vel(outputs[1],outputs[0]);

        /* Update the commanded speed for the Kalman filter */
        float u_command[2] = {outputs[0],outputs[1]};
        commandSpeed(u_command);

        //nnParams.currentTime = get_sys_time_usec()-nnParams.currentTime;
        // printNode(); //This is for debugging 
        // return 0;
    }
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
                msParams.MS[y][x] = msParams.MS[y][x] - MS_TIME_STEP;
                if(msParams.MS[y][x] < 1){
                    msParams.MS[y][x] = 1;
                }
            }
        }
    }

    //float footprint = 1.0; //This is the size of the sensor footprint

    for (uint8_t agentNum = 0; agentNum < MS_SWARM_SIZE; agentNum++){
        uint8_t currentCell_x = 0;
        uint8_t currentCell_y = 0;
        float loopX = 0;    //stores the position of the current UAV being tested. Just for convenience 
        float loopY = 0;
        if(agentNum == MS_CURRENT_ID){
            struct EnuCoor_f *pos_Age = stateGetPositionEnu_f();
            float tempX=(*pos_Age).x;
            float tempY=(*pos_Age).y;
            (*pos_Age).x = a*tempX+b*tempY+c;
            (*pos_Age).y = -b*tempX+a*tempY+d;
            // printf("\nAGE: %f,%f",(*pos_Age).x,(*pos_Age).y);

            if ((*pos_Age).x >= 0 && (*pos_Age).x <= MS_LENGTH && (*pos_Age).y >= 0 && (*pos_Age).y <= MS_BREDTH) {
                currentCell_x = (uint8_t) ((*pos_Age).x/MS_GRID_RES);
                currentCell_y = (uint8_t) ((*pos_Age).y/MS_GRID_RES);
                loopX = (*pos_Age).x;
                loopY = (*pos_Age).y;
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

            tempLimit = currentCell_x + (uint8_t) (MS_FOOTPRINT/MS_GRID_RES);
            if (tempLimit < (uint8_t) (MS_LENGTH/MS_GRID_RES)){
                lim[0] = tempLimit;
            }else{
                lim[0] = (uint8_t) (MS_LENGTH/MS_GRID_RES);
            }

            tempLimit = (uint8_t) (MS_FOOTPRINT/MS_GRID_RES);
            if (currentCell_x > tempLimit){
                lim[1] = currentCell_x-tempLimit;
            } else {
                lim[1] = 0;
            }

            tempLimit = currentCell_y + (uint8_t) (MS_FOOTPRINT/MS_GRID_RES);
            if (tempLimit < (uint8_t) (MS_BREDTH/MS_GRID_RES)){
                lim[2] = tempLimit;
            }else{
                lim[2] = (uint8_t) (MS_BREDTH/MS_GRID_RES);
            }

            tempLimit = (uint8_t) (MS_FOOTPRINT/MS_GRID_RES);
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
                        if (dist <= (MS_FOOTPRINT-MS_GRID_RES/2)*(MS_FOOTPRINT-MS_GRID_RES/2)) {
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
    msParams.MS[2][2] = 0;
    msParams.MS[2][3] = 0;
    msParams.MS[3][3] = 0;
    msParams.MS[3][2] = 0;

    msParams.sensorRange = MS_SENSOR_RANGE;
    /* Starting positions of the Drones */
    msParams.uavs[0].x = 3.5;
    msParams.uavs[0].y = 3.5;

    nnParams.surveillanceOn = FALSE;
    nnParams.land = FALSE;
    nnParams.depotNotFree = 0;
    nnParams.avoid = FALSE;
    nnParams.bt_State = 6;


/* --------------------------------------------------------------------------------------------------------------------*/
/* _____________________________________________ NN final 7x7 tests___________________________________________________ */
/* --------------------------------------------------------------------------------------------------------------------*/   

    /* Initialise the NN structure: Test 1 NN_7x7_ 
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

        /* Initialise the NN structure: Test 2 NN_7x7_ 
     * Number of nodes: 24
     * Number of connections: 47    */
    // uint8_t assign[24] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,20,24,29,117,70,19};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {23,18};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-2.0285135259,-0.9994712407,5.8144112103,-3.1230684485,5.3561482615,-5.6675702714,5.3454523776,-4.9153541935,-4.9017904727,1.7858825179,4.4297222065,6.1632496465,3.5527243817,1.1825594977,-5.796064955,-8,6.8699605639,2.0864711698,5.7375711164,5.0503654469,4.9099767666,-5.8995122678,0.9079400932,0.4882761165,-5.8072668782,-0.5578355993,8,6.89630141,8,-5.2355467417,0.7273579125,-6.431080182,-3.1435985099,3.2104678354,-3.8629629996,-8};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[11] = {-3.1427011028,-6.1027077033,-4.1037145288,6.7399510997,4.7139968833,-3.9789407749,2.8869570942,6.7153485658,-4.858884879,2.5320841941,-0.9805469844};
    // memcpy(nnParams.connectWeight, connect, 11*sizeof(float));

    // uint8_t connectTo[11] = {24,19,29,19,70,19,117,19,117,70,70};
    // memcpy(nnParams.connectTo, connectTo, 11*sizeof(uint8_t));

    // uint8_t connectFrom[11] = {4,24,14,29,16,70,17,117,1,18,29};
    // memcpy(nnParams.connectFrom, connectFrom, 11*sizeof(uint8_t));

        /* Initialise the NN structure: Test 3 NN_7x7_ 
     * Number of nodes: 25
     * Number of connections: 47    */
    // uint8_t assign[25] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,24,29,70,135,165,19,20};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {23,24};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-3.5771787937,2.5862283547,4.0515124771,-6.635475843,7.3717118688,-1.4801757642,8,-7.7098625418,-6.4594536169,1.2774363915,3.704167533,5.5491261621,2.7468325137,5.6230148665,-8,-8,5.5818797709,5.9086396211,6.3356366059,1.0858290666,5.2058411737,1.6450376407,-2.9185671948,3.8123904435,-8,-6.5511698561,1.0278836718,6.720104633,3.7632374912,0.6538708305,-5.8552966957,-4.944389386,-5.2613166771,-5.5054832605,-3.2582508076,2.5590442474};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[11] = {-4.3329538896,1.574588618,-8,1.9622978998,4.9393872165,-4.1872943142,3.1388753478,6.1272519968,-1.6759120247,8,-1.9092086485};
    // memcpy(nnParams.connectWeight, connect, 11*sizeof(float));

    // uint8_t connectTo[11] = {24,19,29,19,70,19,135,20,165,19,29};
    // memcpy(nnParams.connectTo, connectTo, 11*sizeof(uint8_t));

    // uint8_t connectFrom[11] = {4,24,14,29,16,70,9,135,7,165,10};
    // memcpy(nnParams.connectFrom, connectFrom, 11*sizeof(uint8_t));

        /* Initialise the NN structure: Test 4 NN_7x7_ 
     * Number of nodes: 25
     * Number of connections: 48    */
    // uint8_t assign[25] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,24,29,70,135,165,19,20};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {23,24};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-4.8397594513,2.5862283547,1.7212637923,-6.0109132738,8,-3.2945417342,8,-7.7098625418,-5.7851620501,-0.5575685973,4.1154405164,3.7631667705,4.5200177268,3.1986918337,-7.2657020531,-8,5.5818797709,7.4999523845,6.3356366059,0.2334549177,5.2058411737,0.0793784294,-4.8623957393,4.8394898245,-8,-6.5305993532,1.0278836718,7.0878623049,2.4308743247,-0.3286492927,-3.9684044411,-6.3808469494,-3.0352065955,-5.9876657171,-5.7274456786,2.5590442474};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[12] = {-6.3908161283,1.574588618,-8,1.9622978998,4.5967902749,-2.3569519005,4.2378121485,5.5208258641,-1.173413522,8,-1.266473448,0.1896226346};
    // memcpy(nnParams.connectWeight, connect, 12*sizeof(float));

    // uint8_t connectTo[12] = {24,19,29,19,70,19,135,20,165,19,29,70};
    // memcpy(nnParams.connectTo, connectTo, 12*sizeof(uint8_t));

    // uint8_t connectFrom[12] = {4,24,14,29,16,70,9,135,7,165,10,5};
    // memcpy(nnParams.connectFrom, connectFrom, 12*sizeof(uint8_t));

        /* Initialise the NN structure: Test 5 NN_7x7_ 
     * Number of nodes: 26
     * Number of connections: 55    */
    uint8_t assign[26] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,83,121,178,35,172,20,44,19};
    memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    uint8_t assign2[2] = {25,23};
    memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    float connectionsInit[36] = {3.8360311297,2.49015626,2.3169669735,-5.0755323134,-5.0578197772,-3.5591546915,-5.7928456906,4.9633766254,3.8503515232,5.9693259022,6.760043058,-6.6504926017,-3.315801608,-2.9028842763,-1.1976721678,0.9257645116,-6.5627287018,7.2830931274,-0.182095891,8,-1.0011790023,-5.134037816,8,2.9646068681,3.8619043883,-0.6331326088,4.1374115418,4.83178036,-6.0994057187,-7.1519338478,-8,-5.837795278,-1.6632439276,3.4829779104,3.5911153692,-2.5911586785};
    memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    float connect[19] = {0.7836012949,5.6957320684,-6.20957576,-3.8830001835,4.9996944861,0.504374047,-7.7319598887,6.8601192968,-7.0229238536,6.6897944871,7.1981552966,-4.3816151683,6.1408971153,-5.8992214925,0.8324769484,-7.4224343841,3.0631446286,2.1324622758,-4.9620109689};
    memcpy(nnParams.connectWeight, connect, 19*sizeof(float));

    uint8_t connectTo[19] = {35,20,44,19,44,44,44,83,20,83,35,121,20,83,44,172,20,178,19};
    memcpy(nnParams.connectTo, connectTo, 19*sizeof(uint8_t));

    uint8_t connectFrom[19] = {10,35,9,44,14,2,10,12,83,3,83,3,121,11,20,121,172,15,178};
    memcpy(nnParams.connectFrom, connectFrom, 19*sizeof(uint8_t));

        /* Initialise the NN structure: Test 6 NN_7x7_ 
     * Number of nodes: 27
     * Number of connections: 56    */
    // uint8_t assign[27] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,114,118,161,214,246,20,37,51,19};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {26,23};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-2.8091017067,2.6741037075,-4.6015922549,4.6325198884,4.8532889623,-3.5552078536,-4.4989754347,0.3397705017,-2.9772177469,4.4209120523,5.655344226,8,8,-4.0615316286,-3.6736217289,-8,-7.9393996088,2.4260497856,1.6931853283,-6.5873521746,4.0607365365,8,-7.427368745,-4.9067494669,-4.4042494256,-0.0995998256,-0.867260592,6.3896835486,8,1.2564087087,-5.7691738934,-3.9033576622,-3.1834793722,8,7.7997984038,-7.7926582052};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[20] = {8,5.5355986776,1.2715111607,2.2143856422,-6.7359031381,-3.4954358239,0.3055464248,5.0153428298,-2.5186258593,-4.3595752604,-5.275183538,4.7457404928,2.6898826361,4.1137965015,-6.8298555124,-3.8618539315,-3.7971203302,-1.4644760523,-1.0048656372,1.2711435629};
    // memcpy(nnParams.connectWeight, connect, 20*sizeof(float));

    // uint8_t connectTo[20] = {37,19,51,19,114,19,118,51,161,19,37,37,37,214,19,51,118,246,20,114};
    // memcpy(nnParams.connectTo, connectTo, 20*sizeof(uint8_t));

    // uint8_t connectFrom[20] = {6,37,2,51,5,114,2,118,18,161,8,114,17,15,214,11,11,7,246,17};
    // memcpy(nnParams.connectFrom, connectFrom, 20*sizeof(uint8_t));

        /* Initialise the NN structure: Test 7 NN_7x7_ 
     * Number of nodes: 27
     * Number of connections: 56    */
    // uint8_t assign[27] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,114,118,161,214,246,20,37,51,19};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {26,23};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-4.9007546898,1.3203493008,-4.7294102738,5.9077160884,5.2098044669,-2.454654058,-2.8116636877,0.3397705017,-4.0843101396,2.2436135432,3.5180209761,8,8,-5.1873294006,-2.1671352281,-5.5391298667,-6.3046936191,2.4260497856,-0.0965505715,-4.3286341315,5.2207873511,8,-5.8783322983,-5.9376939158,-3.7945472878,0.2357583282,-2.9284548085,5.4889263066,8,-0.8893457009,-3.5628890297,-3.2130656905,-3.1834793722,8,8,-8};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[20] = {8,5.5355986776,1.5053266891,2.2143856422,-5.7509672187,-1.6828120047,0.3055464248,2.5946709556,-1.428470387,-4.3595752604,-5.275183538,4.7457404928,1.5793512169,5.2640655228,-7.278102212,-2.9003324636,-2.3981679267,-2.9833133097,-1.4580838216,-1.183635503};
    // memcpy(nnParams.connectWeight, connect, 20*sizeof(float));

    // uint8_t connectTo[20] = {37,19,51,19,114,19,118,51,161,19,37,37,37,214,19,51,118,246,20,114};
    // memcpy(nnParams.connectTo, connectTo, 20*sizeof(uint8_t));

    // uint8_t connectFrom[20] = {6,37,2,51,5,114,2,118,18,161,8,114,17,15,214,11,11,7,246,17};
    // memcpy(nnParams.connectFrom, connectFrom, 20*sizeof(uint8_t));

        /* Initialise the NN structure: Test 8 NN_7x7_ 
     * Number of nodes: 24
     * Number of connections: 47    */
    // uint8_t assign[24] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,44,50,215,230,20,19};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {23,22};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-7.0140865166,-4.173036963,6.514694123,3.7127030811,-6.1655958553,-6.8442127946,-7.4541453229,0.3878354737,-4.5012768711,-0.0363558619,0.8203281935,8,1.9498508335,2.3105861759,-8,-0.9611526955,3.0287159705,-1.7780780953,-8,5.5198257811,-5.6945136825,-1.4115689222,-8,-3.5531054855,-4.915495567,-4.2861496855,7.9542694971,6.6040819987,5.8306742854,0.7990457109,5.8312766267,-0.3224307613,-6.6766169302,2.3525474661,-1.4421873346,6.1020345056};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[11] = {-2.7383729742,1.9332477641,0.8022979804,3.8493582881,4.0924680547,6.6542659115,-1.4684498094,5.5556298391,7.2360637413,-2.7290669515,-8};
    // memcpy(nnParams.connectWeight, connect, 11*sizeof(float));

    // uint8_t connectTo[11] = {19,44,20,50,20,44,50,215,19,230,19};
    // memcpy(nnParams.connectTo, connectTo, 11*sizeof(uint8_t));

    // uint8_t connectFrom[11] = {20,6,44,1,50,16,9,13,215,6,230};
    // memcpy(nnParams.connectFrom, connectFrom, 11*sizeof(uint8_t));

        /* Initialise the NN structure: Test 9 NN_7x7_ 
     * Number of nodes: 24
     * Number of connections: 45    */
    // uint8_t assign[24] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,108,113,160,67,20,19};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {23,22};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-1.6002902309,-3.4410946028,1.4873686121,-5.5058337967,2.9442749484,-6.2063280467,-3.7784612223,-3.8291693968,7.5747964394,7.0532964948,4.2077685225,4.1592620269,-1.5701459065,-4.2668080471,-7.5681762453,3.1903331443,-4.9818430406,5.461090725,5.6875571325,0.5179380847,-1.9372844968,-3.8119710748,-4.7216635336,3.116348769,4.3992047228,-4.460468614,2.0283618673,1.7454249358,-7.2842241459,-2.2587671424,-4.2816086157,1.8062317346,1.0471457817,8,-2.0423914606,2.6661748121};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[9] = {5.7378582669,8,7.8711286625,8,1.1787886016,8,7.2319860396,0.3575651405,-4.6502137885};
    // memcpy(nnParams.connectWeight, connect, 9*sizeof(float));

    // uint8_t connectTo[9] = {19,108,19,113,20,160,19,67,20};
    // memcpy(nnParams.connectTo, connectTo, 9*sizeof(uint8_t));

    // uint8_t connectFrom[9] = {20,10,108,1,113,10,160,3,67};
    // memcpy(nnParams.connectFrom, connectFrom, 9*sizeof(uint8_t));

        /* Initialise the NN structure: Test 10 NN_7x7_ 
     * Number of nodes: 23
     * Number of connections: 48    */
    // uint8_t assign[23] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,57,170,19,33,20};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {20,22};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {7.3836474422,-3.5069528341,-0.7832205205,-1.1450741106,-5.6144762482,-2.5196196777,-2.5698599765,4.2695663727,7.3197525793,-0.0564086386,3.3811581531,5.4330654121,-7.2644876042,-2.8787968723,-6.8218877329,-0.867126294,1.8968222977,-2.2541633004,6.291569631,-5.3404908098,-4.3551757901,-5.8421893072,0.8337866553,-1.8524505188,6.387452794,-0.1865200884,-0.9333789167,-3.0003298916,-8,1.0268079961,-8,-5.5325711064,7.0314809044,8,-0.0462766299,5.0739360077};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[12] = {5.8369233411,1.7590741354,3.3831486184,5.5393572915,-5.9068806185,2.9396357961,8,-2.7617025767,-2.3987489985,5.3504712873,-0.0077952919,-0.1233519379};
    // memcpy(nnParams.connectWeight, connect, 12*sizeof(float));

    // uint8_t connectTo[12] = {33,20,57,19,57,33,33,170,20,33,170,57};
    // memcpy(nnParams.connectTo, connectTo, 12*sizeof(uint8_t));

    // uint8_t connectFrom[12] = {14,33,6,57,14,19,11,16,170,17,3,10};
    // memcpy(nnParams.connectFrom, connectFrom, 12*sizeof(uint8_t));

        /* Initialise the NN structure: Test 11 NN_7x7_ 
     * Number of nodes: 25
     * Number of connections: 49    */
    // uint8_t assign[25] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,61,136,47,30,32,19,20};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {23,24};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {5.4841635232,1.1948567584,7.8686038707,2.9811569229,-1.9586833103,-5.274438419,-2.7581781154,-7.4438691736,4.671796126,7.8260151932,8,4.1817335811,-8,-5.9526577959,-1.3430140181,1.0023512386,7.4951704921,-0.0670602138,2.6343901105,2.8357237077,3.7282187413,-2.5387424682,-8,-0.0691779334,7.9196978542,4.217681148,-3.7175159726,7.8249807531,-2.5948667501,-7.8695431798,-6.1373177593,3.6014486726,-2.804943162,5.6241391705,-1.1018673765,-6.6929181479};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[13] = {1.336256461,-3.5103624169,1.6413759765,1.4667912822,1.0824592743,-8,-0.9634201871,6.2552908877,-0.4100813549,0.8300155924,1.171672835,5.7651063676,0.9925489661};
    // memcpy(nnParams.connectWeight, connect, 13*sizeof(float));

    // uint8_t connectTo[13] = {20,61,19,61,136,19,47,19,61,30,20,32,19};
    // memcpy(nnParams.connectTo, connectTo, 13*sizeof(uint8_t));

    // uint8_t connectFrom[13] = {19,4,61,8,7,136,9,47,9,4,30,136,32};
    // memcpy(nnParams.connectFrom, connectFrom, 13*sizeof(uint8_t));

        /* Initialise the NN structure: Test 12 NN_7x7_ 
     * Number of nodes: 26
     * Number of connections: 53    */
    // uint8_t assign[26] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,23,50,161,29,30,79,19,20};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {24,25};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-7.7142112094,0.6010032573,8,6.0720710578,-1.6238965053,-3.6971859382,0.6175741648,-5.8397754608,8,4.9690848573,-5.3429339826,3.2015464284,-3.0321039125,-2.3943512101,-7.4184753503,-7.8839733588,-0.1067873815,3.5903270542,6.4923530941,5.6889030208,-0.0571213569,-8,-1.3309697094,-5.4680529348,-3.2435017818,-5.1299466103,3.8144922659,5.1293582626,-5.9273381155,-1.1935890724,-0.4223099404,-6.0844166182,-4.8278253657,8,7.07405856,2.1016355317};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[17] = {-1.9291493599,-7.0370129765,6.4973387801,-2.5441459341,-0.5264267505,7.0801501486,-0.2921021915,-6.449541702,-4.7231566115,2.4371021259,2.9941360259,-0.2738979958,7.2778775508,3.5577120487,2.8836810048,-7.6678148036,-3.68983113};
    // memcpy(nnParams.connectWeight, connect, 17*sizeof(float));

    // uint8_t connectTo[17] = {23,19,23,50,19,79,19,50,161,79,23,20,79,29,20,30,19};
    // memcpy(nnParams.connectTo, connectTo, 17*sizeof(uint8_t));

    // uint8_t connectFrom[17] = {15,23,2,10,50,7,79,15,7,161,5,79,50,9,29,3,30};
    // memcpy(nnParams.connectFrom, connectFrom, 17*sizeof(uint8_t));

    /* Initialise the NN structure: Test 13 NN_7x7_ 
     * Number of nodes: 26
     * Number of connections: 50    */
    // uint8_t assign[26] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,27,66,80,34,37,38,19,20};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {24,25};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-8,1.3194291634,7.2932405305,0.7392500837,3.5214890822,2.712081432,-4.0431491436,-5.936903512,-8,5.8913930889,3.4737541942,-5.5706957568,4.2637908751,1.021974265,-0.3826459326,-6.8131650323,2.7395854864,4.1957100707,3.8490037305,5.2739353544,0.2503079058,-2.5204974617,-4.7232706205,-6.5280266362,6.1538316364,-5.2640499102,8,6.46421875,-2.3164311962,-5.4996649445,-1.2031402819,-3.6369165938,-8,5.5010466411,3.406650999,2.843147167};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[14] = {-3.6822848889,1.1943712026,2.4861359128,-5.4794009528,-4.7802167066,7.4611521322,-4.6945197574,5.0253971839,6.2728334005,-2.256454901,4.5415747609,-4.4683730486,1,-4.0431491436};
    // memcpy(nnParams.connectWeight, connect, 14*sizeof(float));

    // uint8_t connectTo[14] = {27,19,66,20,66,80,19,80,34,19,37,20,38,19};
    // memcpy(nnParams.connectTo, connectTo, 14*sizeof(uint8_t));

    // uint8_t connectFrom[14] = {7,27,15,66,12,6,80,16,4,34,13,37,7,38};
    // memcpy(nnParams.connectFrom, connectFrom, 14*sizeof(uint8_t));

        /* Initialise the NN structure: Test 14 NN_7x7_ 
     * Number of nodes: 26
     * Number of connections: 58    */
    // uint8_t assign[26] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,27,66,80,116,143,19,83,20};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {23,25};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {5.0273775337,5.7914166765,5.2598119974,-3.0993684978,-4.5856340822,-8,-0.6271332543,3.8447362493,2.6470552172,-5.8447199068,-3.1877050979,2.4944552661,-8,-7.3227875957,-3.2916132425,-1.9470479925,6.3208912483,5.5405826795,-4.4304623781,4.5090043768,-0.4985962741,-5.521472517,-7.0748096624,-7.1309317936,6.1849692658,5.7073575662,-3.585105054,6.4078041157,-8,-3.2873645585,-4.3597298597,-6.0033167021,-2.4834504392,1.9363597465,-2.324951271,4.9104633928};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[22] = {-5.0093107696,7.4592641127,-7.8401144041,-3.3449968102,4.4340712484,-5.7679680712,-1.6989028553,2.1421245834,-6.7667016149,-0.0984432985,2.3406747409,5.3350020801,8,8,1.4827132779,3.6413861149,-6.5779088499,-6.3429004282,0.6026489621,-0.4605954459,3.4690434649,-0.5610988131};
    // memcpy(nnParams.connectWeight, connect, 22*sizeof(float));

    // uint8_t connectTo[22] = {27,19,66,20,66,80,19,83,20,83,80,116,20,27,143,83,143,66,83,80,116,66};
    // memcpy(nnParams.connectTo, connectTo, 22*sizeof(uint8_t));

    // uint8_t connectFrom[22] = {7,27,15,66,12,6,80,10,83,14,15,6,116,1,10,143,9,11,7,8,16,3};
    // memcpy(nnParams.connectFrom, connectFrom, 7*sizeof(uint8_t));

        /* Initialise the NN structure: Test 15 NN_7x7_ 
     * Number of nodes: 26
     * Number of connections: 53    */
    // uint8_t assign[26] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,119,124,32,35,36,101,19,20};
    // memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    // uint8_t assign2[2] = {24,25};
    // memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    // float connectionsInit[36] = {-7.4855969085,0.3220077529,3.5711185192,5.5171792867,5.2017613817,-5.3013835273,-2.4979423803,-6.4247390016,0.3768104543,3.5339205662,-2.749631625,-1.0337706708,0.4820898555,-5.6161123136,-1.7314501706,-0.2842216303,7.671862988,6.2935649358,7.448508646,3.9237350378,8,-4.4290644547,-6.0119465372,-8,6.5981752568,2.0203859346,8,-2.5242756445,2.6168419536,-5.7092276836,-1.1378759324,-5.8332742272,-1.0187045198,1.4438648014,1.8936270444,-1.0874298047};
    // memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    // float connect[17] = {8,5.2496064682,0.5633136022,4.6606205902,-6.9071925607,2.743173537,-7.9085204503,-0.6975917561,8,2.3630878284,7.2209044073,-2.0534633613,3.8790125081,6.575349591,0.8957383558,-0.8153785341,0.3384633017};
    // memcpy(nnParams.connectWeight, connect, 17*sizeof(float));

    // uint8_t connectTo[17] = {20,101,19,119,19,124,19,101,124,32,19,35,20,36,20,35,20};
    // memcpy(nnParams.connectTo, connectTo, 17*sizeof(uint8_t));

    // uint8_t connectFrom[17] = {19,14,101,15,119,16,124,124,7,4,32,9,35,15,36,4,119};
    // memcpy(nnParams.connectFrom, connectFrom, 17*sizeof(uint8_t));

}

bool printMS(){
    for (uint8_t i = 0; i < 14; i++) {
        printf("\n%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",msParams.MS[i][0],msParams.MS[i][1],msParams.MS[i][2],msParams.MS[i][3],msParams.MS[i][4],msParams.MS[i][5],msParams.MS[i][6],msParams.MS[i][7],msParams.MS[i][8],msParams.MS[i][9],msParams.MS[i][10],msParams.MS[i][11],msParams.MS[i][12],msParams.MS[i][13]);
    }
    
    return 0;
}

void printNode(){
    // x1,y1,x2,y2,node1,node2,...,node18,outNode1,outNode2.
        /* Conversion between coordinate systems */
    // float a = 0.827559;
    // float b = 0.5613786;
    // float c = -3.903735;
    // float d = 1.0823;

    struct EnuCoor_f *pos_print = stateGetPositionEnu_f();
    // float tempX=(*pos_print).x;
    // float tempY=(*pos_print).y;
    // (*pos_print).x = a*tempX+b*tempY+c;
    // (*pos_print).y = -b*tempX+a*tempY+d;
    // float printX = a*tempX+b*tempY+c;;
    // float printY = -b*tempX+a*tempY+d;

    /* Print the flag states */
    // printf("\n%f,%f,%f,%f,%f,%f,%i,%i",msParams.uavs[1].x,msParams.uavs[1].y,msParams.uavs[0].x,msParams.uavs[0].y,nnParams.depotNotFree,nnParams.bt_State,nnParams.land,nnParams.surveillanceOn);



    // printf("\n%f,%f,%f,%f,%f,%f",msParams.uavs[0].x,msParams.uavs[0].y,msParams.uavs[1].x,msParams.uavs[1].y,nnParams.depotNotFree,nnParams.bt_State);
    printf("\n%f,%f,%f,%f,%f,%f",msParams.uavs[0].x,msParams.uavs[0].y,msParams.uavs[1].x,msParams.uavs[1].y,msParams.uavs[2].x,msParams.uavs[2].y);

    // for (uint8_t i =0; i < 18;i++){
    //     printf(",%f",nnParams.node_out[i]);
    // }
    // printf(",%f,%f",nnParams.node_out[nnParams.outputIndex[0]],nnParams.node_out[nnParams.outputIndex[1]]);
}

/*void neural_network_periodic(void) {
    
}*/

/* Implements the Homing behaviour*/
void homing(float xPos, float yPos){
    float dist = sqrtf((msParams.uavs[MS_CURRENT_ID].x-xPos)*(msParams.uavs[MS_CURRENT_ID].x-xPos)+(msParams.uavs[MS_CURRENT_ID].y-yPos)*(msParams.uavs[MS_CURRENT_ID].y-yPos));
    float theta;
    float out[2];
    if (xPos > msParams.uavs[MS_CURRENT_ID].x) {
        if (yPos > msParams.uavs[MS_CURRENT_ID].y) {
            theta = atanf((xPos-msParams.uavs[MS_CURRENT_ID].x)/(yPos-msParams.uavs[MS_CURRENT_ID].y));
        }
        else {
            theta = PI/2+fabs(atanf((yPos-msParams.uavs[MS_CURRENT_ID].y)/(xPos-msParams.uavs[MS_CURRENT_ID].x)));
        }
        
    } else {
        if (yPos > msParams.uavs[MS_CURRENT_ID].y) {
            theta = 1.5*PI+fabs(atanf((yPos-msParams.uavs[MS_CURRENT_ID].y)/(xPos-msParams.uavs[MS_CURRENT_ID].x)));
        }
        else {
            theta = 1.5*PI-fabs(atanf((yPos-msParams.uavs[MS_CURRENT_ID].y)/(xPos-msParams.uavs[MS_CURRENT_ID].x))); 
        }
    } 

    if (dist > (MS_MAX_VEL*MS_TIME_STEP)) {
        out[0] = MS_MAX_VEL*sinf(theta);   // x vel
        out[1] = MS_MAX_VEL*cosf(theta);   // Y vel
    } else {
        out[0] = dist*sinf(theta)/MS_TIME_STEP;
        out[1] = dist*cosf(theta)/MS_TIME_STEP;
    }
    /* Set the comaanded speed */
    guidance_h_set_guided_body_vel(out[1],out[0]);

    /* Update the commanded speed for the Kalman filter */
    commandSpeed(out);   
    
}

/* Implements a basic avoidance stratergy. Right turn if UAVs become too close */
void avoid() {

    float angle = PI/4; //This is the angle for the avoidance turn
    float uk[2];
    float out [2];
    getCommandSpeed(uk);
    float theta;    //Flight direction

    if (nnParams.avoid == 1) {
        //Determine current flight angle
        if (uk[0] != 0) {
            theta = atanf(uk[1]/uk[0]);
        }  else {
            if (uk[1] >= 0) {
                theta = PI/2;
            } else {
                theta = -PI/2;
            }  
        }
        theta = theta - angle;
        //Implements the avoidance 'turn'
        if (uk[0] >= 0) {
            if (theta < - PI/2) {
                theta = PI+theta;
                out[0] = -MS_MAX_VEL*cosf(theta);
                out[1] = -MS_MAX_VEL*sinf(theta);
            } else {
                out[0] = MS_MAX_VEL*cosf(theta);
                out[1] = MS_MAX_VEL*sinf(theta);
            }
        } else {
            if (theta < -PI/2 ) {
                theta = PI+theta;
                out[0] = MS_MAX_VEL*cosf(theta);
                out[1] = MS_MAX_VEL*sinf(theta);
            } else {
                out[0] = -MS_MAX_VEL*cosf(theta);
                out[1] = -MS_MAX_VEL*sinf(theta);
            }
        }
        nnParams.avoid = 2;
        
    } else if (nnParams.avoid == 2) {
        //Maintain the avoidance motion
        out[0] = uk[0];
        out[1] = uk[1];
    }else {
        //Determine current flight angle
        if (uk[0] != 0) {
            theta = atanf(uk[1]/uk[0]);
        }  else {
            if (uk[1] >= 0) {
                theta = PI/2;
            } else {
                theta = -PI/2;
            }  
        }

        //Reverse the flight path
        if (uk[0] >= 0) {
            out[0] = MS_MAX_VEL*cosf(theta);
            out[1] = MS_MAX_VEL*sinf(theta);
        } else {
            out[0] = -MS_MAX_VEL*cosf(theta);
            out[1] = -MS_MAX_VEL*sinf(theta);
        }
        out[0] = -out[0];
        out[1] = -out[1];
        nnParams.avoid = 1;        
    }

    /* Set the comaanded speed */
    guidance_h_set_guided_body_vel(out[1],out[0]);

    /* Update the commanded speed for the Kalman filter */
    commandSpeed(out);   
}

/* Returns the status of the land flag to the flight plan */
bool landNow() {
    if (nnParams.land == TRUE){
        nnParams.land = FALSE;
        return TRUE;
    } else
    {
        return FALSE;
    }
    
}

void setTakeOffFlag_NN() {
    nnParams.land = FALSE;
}

void getDepotFlag(float data[2]){
    data[0] = MS_CURRENT_ID+2;      // MAGIC NUMBER: Used to relate the index i with the ID from UWB
    data[1] = nnParams.depotNotFree;
}

/* This implements the refuelling Behaviour tree */
void behaviourTree(){
    if (nnParams.surveillanceOn == TRUE) {
        /* Conversion between coordinate systems */
        float a = 0.827559;
        float b = 0.5613786;
        float c = -3.903735;
        float d = 1.0823;

        float tempFlag[MS_SWARM_SIZE];
        for(uint8_t i=0;i<MS_SWARM_SIZE;i++){
            float temp[3];
            getPos_UWB((i+2),temp);             // MAGIC NUMBER: Used to relate the index i with the ID from UWB
            tempFlag[i] = temp[2];
            msParams.uavs[i].x = temp[0];
            msParams.uavs[i].y = temp[1];

            msParams.uavs[i].x = a*temp[0]+b*temp[1]+c;
            msParams.uavs[i].y = -b*temp[0]+a*temp[1]+d;
        }
        //Assigning the Depot free flag
        nnParams.depotNotFree = 0;
        for(uint8_t i=0;i<MS_SWARM_SIZE;i++){
            if (i != MS_CURRENT_ID) {
                if (tempFlag[i] == 1) {
                    nnParams.depotNotFree = 1;
                }
            }
        }
        // if (nnParams.depotNotFree == 1 && nnParams.returnToDepot == 1) {
        //     nnParams.returnToDepot = 0;
        // }

        struct EnuCoor_f *pos_BT = stateGetPositionEnu_f();
        float tempX=(*pos_BT).x;
        float tempY=(*pos_BT).y;
        (*pos_BT).x = a*tempX+b*tempY+c;
        (*pos_BT).y = -b*tempX+a*tempY+d;
        msParams.uavs[MS_CURRENT_ID].x = (*pos_BT).x;
        msParams.uavs[MS_CURRENT_ID].y = (*pos_BT).y;
        // printf("\nBT: %f,%f,%f,%f",(*pos_BT).x,(*pos_BT).y,msParams.uavs[1].x,msParams.uavs[1].y);

        // nnParams.depotNotFree = 1;

        /* --- This was used to test the Homing and depot flag -- */
        // float posTestX[3] = {1.0,3.5,6.0};
        // float posTestY[3] = {5.0,5.0,1.0};

        // float distDepot = ((*pos_BT).x-posTestX[counterTemp])*((*pos_BT).x-posTestX[counterTemp])+((*pos_BT).y-posTestY[counterTemp])*((*pos_BT).y-posTestY[counterTemp]);
        // if(distDepot < 0.01){
        //     counterTemp = counterTemp +1;
        //     if (counterTemp == 3) {
        //         counterTemp = 0;
        //     }
        //     if(nnParams.depotNotFree == 1){
        //         nnParams.depotNotFree = 0;
        //     }else{
        //         nnParams.depotNotFree = 1;
        //     }
        // }

        /* The BT: Assumes that the charging depot is inside the MS */
        if ((*pos_BT).x <= MS_LENGTH && (*pos_BT).x >= 0 && (*pos_BT).y <= MS_BREDTH && (*pos_BT).y >= 0) {
        //     // uint8_t currentCell_x = (uint8_t) ((*pos_BT).x/MS_GRID_RES);
        //     // uint8_t currentCell_y = (uint8_t) ((*pos_BT).y/MS_GRID_RES);
            float dist = 5;

            /* Find distance to nearst other UAV */
            for(uint8_t i=0;i < MS_SWARM_SIZE;i++){
                if (i != MS_CURRENT_ID) {
                    float tempDist = ((*pos_BT).x-msParams.uavs[i].x)*((*pos_BT).x-msParams.uavs[i].x)+((*pos_BT).y-msParams.uavs[i].y)*((*pos_BT).y-msParams.uavs[i].y);
                    if (tempDist < dist) {
                        dist = tempDist;
                    }
                }
            }

            if ((electrical.vsupply < 11.4*10) && nnParams.depotNotFree == 0) {
                /* If Depot is free and fuel is low, go and charge */
                float distDepot = ((*pos_BT).x-MS_DEPOT_X)*((*pos_BT).x-MS_DEPOT_X)+((*pos_BT).y-MS_DEPOT_Y)*((*pos_BT).y-MS_DEPOT_Y);
                if (distDepot <= 0.01) {
                    nnParams.land = TRUE;
                    nnParams.bt_State = 3;
                }else {
                    homing(MS_DEPOT_X,MS_DEPOT_Y);
                    // homing(3.5,3.5);
                    nnParams.bt_State = 2;
                }
                nnParams.depotNotFree = 1;
                nnParams.avoid = 0;
            } else if ((electrical.vsupply < 10.5*10)) {
                /* If fuel very low, return to depot */
                float distDepot = ((*pos_BT).x-(MS_DEPOT_X+MS_CURRENT_ID))*((*pos_BT).x-(MS_DEPOT_X+MS_CURRENT_ID))+((*pos_BT).y-(MS_DEPOT_Y))*((*pos_BT).y-(MS_DEPOT_Y));
                if (distDepot <= 0.01) {
                    nnParams.land = TRUE;
                    nnParams.bt_State = 4;
                }else {
                    homing((MS_DEPOT_X+MS_CURRENT_ID),MS_DEPOT_Y);
                    // homing(3.5,3.5);
                    nnParams.bt_State = 2;
                }
                nnParams.avoid = 0;
            } else  if (dist < MS_DIST_THRESH*MS_DIST_THRESH) {
            // if (dist < MS_DIST_THRESH*MS_DIST_THRESH) {
                avoid();
                nnParams.depotNotFree = 0;
                nnParams.bt_State = 1;
            } else if (dist < MS_DIST_THRESH_2*MS_DIST_THRESH_2 && nnParams.avoid > 0) {   
                avoid();
                nnParams.depotNotFree = 0;
                nnParams.bt_State = 1;
            } else {
                /* Default case is surveillance */
                calcNN();
                nnParams.depotNotFree = 0;
                nnParams.avoid = 0;
                nnParams.bt_State = 5;
            }
        }
        else{
            //Return to the centre of the MS 
            homing(3.5,3.5);
            nnParams.avoid = FALSE;
        }

       printNode(); 
    }  
}