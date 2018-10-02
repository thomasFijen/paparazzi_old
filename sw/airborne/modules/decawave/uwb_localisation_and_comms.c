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
 * @file "modules/decawave/uwb_localisation_and_comms.c"
 * @author Thomas Fijen
 * This function contains the code necessary to localise the UAV, using UWB, as well as the inter-UAV communication
 */

#include "modules/decawave/uwb_localisation_and_comms.h"
//#include "modules/decawave/multilateration_ls.h"
#include "modules/decawave/multilateration_nlls.h"
//#include "modules/decawave/trilateration.h"

#include "std.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/abi.h"
#include "subsystems/gps.h"
#include "state.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Number of anchors
 *
 * using standard trilateration algorithm, only 3 anchors are required/supported
 * at the moment.
 * More advanced multilateration algorithms might allow more anchors in the future
 */


#ifndef DW1000_NB_ANCHORS
#define DW1000_NB_ANCHORS 3
#endif

/** default offset, applied to final result not to individual distances */
#ifndef DW1000_OFFSET
#define DW1000_OFFSET { 0.f, 0.f, 0.f }
#endif

/** default scale factor, applied to final result not to individual distances */
#ifndef DW1000_SCALE
#define DW1000_SCALE { 1.f, 1.f, 1.f }
#endif

/** default initial heading correction between anchors frame and global frame */
#ifndef DW1000_INITIAL_HEADING
#define DW1000_INITIAL_HEADING 0.f
#endif

/** default timeout (in ms) */
#ifndef DW1000_TIMEOUT
#define DW1000_TIMEOUT 500
#endif

#ifndef DW1000_SERIAL_COMM_DIST_NUM_NODES
#define DW1000_SERIAL_COMM_DIST_NUM_NODES 2
#endif

static bool _inProgress = false;
static uint8_t _varByte = 0;
static uint8_t count = 0;
static float aveX[5] = {0.f,0.f,0.f,0.f,0.f};
static float aveY[5] = {0.f,0.f,0.f,0.f,0.f};
/*static float aveZ[5] = {0.f,0.f,0.f,0.f,0.f};*/
// static float X_old_kal[6] = {0,0,0,0,0,0};
// static float X_new_kal[6] = {0,0,0,0,0,0};


/* -- Parameters used by Steven in his Code. Not sure if these two lines are needed -- */
#define UWB_SERIAL_PORT (&((UWB_DW1000_DEV).device))
struct link_device *external_device = UWB_SERIAL_PORT;

/* Message types */
#define DW_NB_DATA 6
#define START_MARKER 254
#define UWB_SERIAL_COMM_RANGE 3
#define UWB_SERIAL_COMM_X 0
#define UWB_SERIAL_COMM_Y 1
//#define UWB_SERIAL_COMM_NUM_NODES 6 // How many nodes actually are in the network
//#define UWB_SERIAL_COMM_DIST_NUM_NODES UWB_SERIAL_COMM_NUM_NODES-1-DW1000_NB_ANCHORS  // How many distant nodes are in the network, excluding anchors

// static struct nodeState states[UWB_SERIAL_COMM_DIST_NUM_NODES];
static struct nodeState states[DW1000_SERIAL_COMM_DIST_NUM_NODES];

/** DW1000 positionning system structure */
struct DW1000 {
  uint8_t buf[DW_NB_DATA];    ///< incoming data buffer
  uint8_t idx;                ///< buffer index
  //uint8_t ck;                 ///< checksum
  //uint8_t state;              ///< parser state
  float initial_heading;      ///< initial heading correction
  struct Anchor anchors[DW1000_NB_ANCHORS];   ///<anchors data
  struct EnuCoor_f pos;       ///< local pos in anchors frame
  struct EnuCoor_f raw_pos;   ///< unscaled local pos in anchors frame
  struct GpsState gps_dw1000; ///< "fake" gps structure
  struct LtpDef_i ltp_def;    ///< ltp reference
  bool updated;               ///< new anchor data available
};

static struct DW1000 dw1000;

/* Returns the stored positions of the other drones sent over the UWB */
void getPos_UWB(uint8_t index, float positions[2]){
  for(uint8_t i = 0; i < DW1000_SERIAL_COMM_DIST_NUM_NODES; i++){
    if(states[i].nodeAddress == index){
      positions[0] = states[index].x;
      positions[1] = states[index].y;     
    }
  }
}

//-- These are the functions needed to send data between the UAVs -------------------------------------------
/**
 * Function that encodes the high bytes of the serial data to be sent.
 * Start and end markers are reserved values 254 and 255. In order to be able to send these values,
 * the payload values 253, 254, and 255 are encoded as 2 bytes, respectively 253 0, 253 1, and 253 2.
 */
/*static void encodeHighBytes(uint8_t *send_data, uint8_t msg_size, uint8_t *data_send_buffer, uint8_t *data_total_send)
{
  uint8_t data_send_count = msg_size;
  *data_total_send = 0;
  for (uint8_t i = 0; i < data_send_count; i++) {
    if (send_data[i] >= UWB_SERIAL_COMM_SPECIAL_BYTE) {
      data_send_buffer[*data_total_send] = UWB_SERIAL_COMM_SPECIAL_BYTE;
      (*data_total_send)++;
      data_send_buffer[*data_total_send] = send_data[i] - UWB_SERIAL_COMM_SPECIAL_BYTE;
    } else {
      data_send_buffer[*data_total_send] = send_data[i];
    }
    (*data_total_send)++;
  }
}*/

static void sendFloat(uint8_t msg_type, float data)
{
/*  static uint8_t data_send_buffer[DW_NB_DATA];
  static uint8_t data_total_send = 0;*/

  // Make bytes of the float
  uint8_t floatbyte[4];
  memcpy(floatbyte, &data, 4);
  /*encodeHighBytes(floatbyte, 4, data_send_buffer, &data_total_send);*/

  UWB_SERIAL_PORT->put_byte(UWB_SERIAL_PORT->periph, 0, START_MARKER);
  UWB_SERIAL_PORT->put_byte(UWB_SERIAL_PORT->periph, 0, msg_type);

  for (uint8_t i = 0; i < 4; i++) {
    UWB_SERIAL_PORT->put_byte(UWB_SERIAL_PORT->periph, 0, floatbyte[i]);
  }
  /*for (uint8_t i = 0; i < data_total_send; i++) {
    UWB_SERIAL_PORT->put_byte(UWB_SERIAL_PORT->periph, 0, data_send_buffer[i]);
  }*/

  /*UWB_SERIAL_PORT->put_byte(UWB_SERIAL_PORT->periph, 0, UWB_SERIAL_COMM_END_MARKER);*/
}

//-------------------------------------------------------------------------------------------------------------------


/*void getRanges(float ranges[4]){
  ranges[0] = dw1000.anchors[0].distance;
  ranges[1] = dw1000.anchors[1].distance;
  ranges[2] = dw1000.anchors[2].distance;
  if(DW1000_NB_ANCHORS == 4)
  {
    ranges[3] = dw1000.anchors[3].distance;
  }
  else
  {
    ranges[3] = 0;
  }
}*/


/** Utility function to get float from buffer */
static inline float float_from_buf(uint8_t* b) {
  float f;
  memcpy((uint8_t*)(&f), b, sizeof(float));
  return f;
}

/** Utility function to get message type from buffer */
static inline uint8_t uint8_t_from_buf(uint8_t* b) {
  uint8_t f;
  memcpy((uint8_t*)(&f), b, sizeof(uint8_t));
  return f;
}

/** Utility function to get uint16_t from buffer */
static inline uint16_t uint16_from_buf(uint8_t* b) {
  uint16_t u16 = 0x0000;
  uint8_t temp;
  memcpy (&temp, b, sizeof(uint8_t));
  return u16+temp;
}

/** Utility function to fill anchor from buffer */
static void fill_anchor_Cust(struct DW1000 *dw) {
  uint16_t id = uint16_from_buf(dw->buf);
  uint8_t msgType = uint8_t_from_buf(dw->buf+1);
  
  if (msgType == UWB_SERIAL_COMM_RANGE)  {
    for (uint8_t i = 0; i < DW1000_NB_ANCHORS; i++) {
      if (dw->anchors[i].id == id) {
      /*float norm = sqrtf((dw->anchors[i].distance - float_from_buf(dw->buf+2))*(dw->anchors[i].distance - float_from_buf(dw->buf+2))); norm < 2*/
        float norm = ((dw->anchors[i].distance - float_from_buf(dw->buf+2))*(dw->anchors[i].distance - float_from_buf(dw->buf+2)));
        if (norm < 4 || dw->anchors[i].distance == 0) { //This is a check to reject outlier distance measurements
          dw->anchors[i].distance = float_from_buf(dw->buf+2);
          dw->anchors[i].time = get_sys_time_float();
          dw->updated = true;
        }
        else{
          dw->anchors[i].distance = dw->anchors[i].distance;
          dw->anchors[i].time = get_sys_time_float();
          dw->updated = true;
        }
        break;
      }
    }
  }
  else{
    for(uint8_t i=0; i< DW1000_SERIAL_COMM_DIST_NUM_NODES; i++){
      if (states[i].nodeAddress == id)
      {
        if (msgType == UWB_SERIAL_COMM_X) {
          states[i].x = float_from_buf(dw->buf+2);
        }
        else{
          states[i].y = float_from_buf(dw->buf+2);
        }
        break;
      }
    } 
  }
}

/**
 * Function for receiving serial data.
 * Only receives serial data that is between the start and end markers. Discards all other data.
 * Stores the received data in received_message, and after decodes the high bytes and copies the final
 * message to the corresponding message in _messages.
 */
/*static void getSerialData(uint8_t *bytes_received)
{
  static bool in_progress = false;
  static uint8_t var_byte;
  static uint8_t received_message[UWB_SERIAL_COMM_MAX_MESSAGE];

  while (external_device->char_available(external_device->periph)) {
    var_byte = UWB_SERIAL_PORT->get_byte(UWB_SERIAL_PORT->periph);

    if (var_byte == UWB_SERIAL_COMM_START_MARKER) {
      (*bytes_received) = 0;
      in_progress = true;
    }

    if (in_progress) {
      if ((*bytes_received) < UWB_SERIAL_COMM_MAX_MESSAGE - 1) {
        received_message[*bytes_received] = var_byte;
        (*bytes_received)++;
      } else {
        in_progress = false;
      }
    }

    if (var_byte == UWB_SERIAL_COMM_END_MARKER) {
      in_progress = false;
      decodeHighBytes(*bytes_received, received_message);
    }
  }
}*/

/** Data parsing function */
static void dw1000_arduino_parse(struct DW1000 *dw)
{   
  while (uart_char_available(&UWB_DW1000_DEV)){
    _varByte = uart_getch(&UWB_DW1000_DEV);
    
    if (_inProgress){
      dw->buf[dw->idx] = _varByte;
      dw->idx++;
    }
    if (dw->idx == DW_NB_DATA){
      _inProgress = false;
      fill_anchor_Cust(dw);
    }
    
    if (_varByte == START_MARKER){
      dw->idx = 0;
      _inProgress = true;
    }
  }  
}

static void send_gps_dw1000_small(struct DW1000 *dw)
{
  // rotate and convert to cm integer
  float x = dw->pos.x * cosf(dw->initial_heading) - dw->pos.y * sinf(dw->initial_heading);
  float y = dw->pos.x * sinf(dw->initial_heading) + dw->pos.y * cosf(dw->initial_heading);
  struct EnuCoor_i enu_pos;
  
  struct EnuCoor_f *pos2 = stateGetPositionEnu_f();
  float z = (*pos2).z; //-- This just needs to be checked, this might be the problem on takeoff. Rather read in the sonar value

  //--Outlier rejection:
  float error = sqrtf((aveX[count-1]-x)*(aveX[count-1]-x)+(aveY[count-1]-y)*(aveY[count-1]-y));
  if (error > 10)
  {
    if(count == 0)
    {
    aveX[count] = aveX[4];
    aveY[count] = aveY[4];
    /*aveZ[count] = aveZ[4];*/
  }
  else
  {
    aveX[count] = aveX[count-1];
    aveY[count] = aveY[count-1];
    /*aveZ[count] = aveZ[count-1];*/
  }
  }
  else
  {
    aveX[count] = x;
    aveY[count] = y;
    /*aveZ[count] = dw->pos.z;*/
  }//--End outlier rejection
  
  // -- Moving average Filter
  x=0;
  y=0;
  /*float z=0;*/
  
  for(uint8_t i=0;i<5;i++)
  {
  x=x+aveX[i];
  y=y+aveY[i];
/*  z=z+aveZ[i];*/
  }
  x=x/5;
  y=y/5;
/*  z=z/5;*/
  
  if(count == 4)
  {
    count = 0;
  }
  else
  {
    count++;
  }
  //-- End of the filters and outlier rejection --
  
  
  //-- Applying the Kalman filter:
  
 //struct EnuCoor_f *vel = stateGetSpeedEnu_f();
/*   float uk[2] = {(*vel).x,(*vel).y};
  kalman_filter(X_new_kal, X_old_kal, uk, x, y);
  
  enu_pos.x = (int32_t) (X_new_kal[4] * 100);
  enu_pos.y = (int32_t) (X_new_kal[5] * 100);
  
  X_old_kal[1] = X_new_kal[1];
  X_old_kal[2] = X_new_kal[2];
  X_old_kal[3] = X_new_kal[3];
  X_old_kal[4] = X_new_kal[4];
  X_old_kal[5] = X_new_kal[5];
  X_old_kal[0] = X_new_kal[0]; */
  
  // --- End of the Kalman filter ---
  
  enu_pos.x = (int32_t) (x * 100);
  enu_pos.y = (int32_t) (y * 100);
  
  enu_pos.z = (int32_t) (z * 100); // dw->pos.
  //struct EnuCoor_f *pos2 = stateGetPositionEnu_f();
  
  // Convert the ENU coordinates to ECEF
  ecef_of_enu_point_i(&(dw->gps_dw1000.ecef_pos), &(dw->ltp_def), &enu_pos);
  SetBit(dw->gps_dw1000.valid_fields, GPS_VALID_POS_ECEF_BIT);

  lla_of_ecef_i(&(dw->gps_dw1000.lla_pos), &(dw->gps_dw1000.ecef_pos));
  SetBit(dw->gps_dw1000.valid_fields, GPS_VALID_POS_LLA_BIT);

  dw->gps_dw1000.hmsl = dw->ltp_def.hmsl + enu_pos.z * 10;    
  SetBit(dw->gps_dw1000.valid_fields, GPS_VALID_HMSL_BIT);

  dw->gps_dw1000.num_sv = 7;
  dw->gps_dw1000.tow = get_sys_time_msec();
  dw->gps_dw1000.fix = GPS_FIX_3D; // set 3D fix to true

  // set gps msg time
  dw->gps_dw1000.last_msg_ticks = sys_time.nb_sec_rem;
  dw->gps_dw1000.last_msg_time = sys_time.nb_sec;
  dw->gps_dw1000.last_3dfix_ticks = sys_time.nb_sec_rem;
  dw->gps_dw1000.last_3dfix_time = sys_time.nb_sec;

  // publish new GPS data
  uint32_t now_ts = get_sys_time_usec();
  
  // -- Sending the position to the Auto pilot
  //update_uwb(now_ts, &(dw->gps_dw1000));
}

//----------------------------------------------------
/* Added a Kalman Filter to the position. Not sure if this will affect the other filters in the INS...
* Inputs: Previously Calculated X, inputs u, UWB pos x, UWB pos y
    u = [vel X; vel Y]; X = [accel x, vel x, accel y, vel y, x, y]
*/
void kalman_filter(float out[6], float X_old[6], float u[2], float x, float y)
{
  //--Discretised model, This is hard coded from the MATLAB file.
  float phi[36] = {0.8942,-2.4404,0,0,0,0,0.0018,0.9976,0,0,0,0,0,0,0.8942,-2.4404,0,0,0,0,0.0018,0.9976,0,0,0,0.002,0,0,1,0,0,0,0,0.002,0,1};
  float gamma[12] = {2.4404,0,0.0024,0,0,2.4404,0,0.0024,0,0,0,0};
  float Cd[12] = {0,0,0,0,1,0,0,0,0,0,0,1};
  
  float K[12] = {-0.008*1.0e-4,0,0,0,0.001*1.0e-4,-0.0049*1.0e-4,0,0.0001*1.0e-4,0.7365*1.0e-4,-0.009*1.0e-4,-0.009*1.0e-4,0.9060*1.0e-4};
  float X_hat[6] = {0,0,0,0,0,0};
  
  //--Temporary storage variables
  float temp_6x1[6];
  float temp2_6x1[6];
  float temp_2x1[2];
  float temp2_2x1[2] = {x,y};
  
  //Determine X_hat from the model 
    mat_mult_6x6_6x1(temp_6x1, phi, X_old);
    mat_mult_6x2_2x1(temp2_6x1, gamma, u);
    mat_add_6x1(X_hat,temp_6x1,temp2_6x1);
    
    //Update the estimate with the Kalman gain
    mat_mult_2x6_6x1(temp_2x1,Cd,X_hat);
    mat_subtract_2x1(temp_2x1,temp2_2x1,temp_2x1);
    mat_mult_6x2_2x1(temp_6x1,K,temp_2x1);
    mat_add_6x1(out,X_hat,temp_6x1);
}

//--Matrix functions for the Kalman filter
void mat_mult_6x6_6x1(float out[6], float in1[36], float in2[6])
{
  out[0] = in1[0]*in2[0] + in1[1]*in2[1] + in1[2]*in2[2] + in1[3]*in2[3] + in1[4]*in2[4] + in1[5]*in2[5];
  out[1] = in1[6]*in2[0] + in1[7]*in2[1] + in1[8]*in2[2] + in1[9]*in2[3] + in1[10]*in2[4] + in1[11]*in2[5];
  out[2] = in1[12]*in2[0] +in1[13]*in2[1] + in1[14]*in2[2] + in1[15]*in2[3] + in1[16]*in2[4] + in1[17]*in2[5];
  out[3] = in1[18]*in2[0] +in1[19]*in2[1] + in1[20]*in2[2] + in1[21]*in2[3] + in1[22]*in2[4] + in1[23]*in2[5];
  out[4] = in1[24]*in2[0] +in1[25]*in2[1] + in1[26]*in2[2] + in1[27]*in2[3] + in1[28]*in2[4] + in1[29]*in2[5];
  out[5] = in1[30]*in2[0] +in1[31]*in2[1] + in1[32]*in2[2] + in1[33]*in2[3] + in1[34]*in2[4] + in1[35]*in2[5];
}
void mat_mult_6x2_2x1(float out[6], float in1[12], float in2[2])
{
  out[0] = in1[0]*in2[0] + in1[1]*in2[1];
  out[1] = in1[2]*in2[0] + in1[3]*in2[1];
  out[2] = in1[4]*in2[0] + in1[5]*in2[1];
  out[3] = in1[6]*in2[0] + in1[7]*in2[1];
  out[4] = in1[8]*in2[0] + in1[9]*in2[1];
  out[5] = in1[10]*in2[0] + in1[11]*in2[1];
}
void mat_add_6x1(float out[6], float in1[6], float in2[6])
{
  out[0] = in1[0]+in2[0];
  out[1] = in1[1]+in2[1];
  out[2] = in1[2]+in2[2];
  out[3] = in1[3]+in2[3];
  out[4] = in1[4]+in2[4];
  out[5] = in1[5]+in2[5];
}
void mat_mult_2x6_6x1(float out[2], float in1[12], float in2[6])
{
  out[0] = in1[0]*in2[0] +in1[1]*in2[1] + in1[2]*in2[2] + in1[3]*in2[3] + in1[4]*in2[4] + in1[5]*in2[5];
  out[1] = in1[6]*in2[0] +in1[7]*in2[1] + in1[8]*in2[2] + in1[9]*in2[3] + in1[10]*in2[4] + in1[11]*in2[5];
}
void mat_subtract_2x1(float out[2], float in1[2], float in2[2])
{
  out[0] = in1[0]-in2[0];
  out[1] = in1[1]-in2[1];
}


//----------------------------------------------------


/// init arrays from airframe file
static const uint16_t ids[] = DW1000_ANCHORS_IDS;
static const float pos_x[] = DW1000_ANCHORS_POS_X;
static const float pos_y[] = DW1000_ANCHORS_POS_Y;
static const float pos_z[] = DW1000_ANCHORS_POS_Z;
static const float offset[] = DW1000_OFFSET;
static const float scale[] = DW1000_SCALE;
static const uint16_t tag_ids[] = DW1000_TAG_IDS;

static void scale_position(struct DW1000 *dw)
{
  dw->pos.x = scale[0] * (dw->raw_pos.x - offset[0]);
  dw->pos.y = scale[1] * (dw->raw_pos.y - offset[1]);
  dw->pos.z = scale[2] * (dw->raw_pos.z - offset[2]);
}

/** check timeout for each anchor
 * @return true if one has reach timeout
 */
static bool check_anchor_timeout(struct DW1000 *dw)
{
  const float now = get_sys_time_float();
  const float timeout = (float)DW1000_TIMEOUT / 1000.;
  for (uint8_t i = 0; i < DW1000_NB_ANCHORS; i++) {
    if (now - dw->anchors[i].time > timeout) {
      return true;
    }
  }
  return false;
}

void local_and_comms_init(void) {
     // init DW1000 structure
    dw1000.idx = 0;
  //  dw1000.ck = 0;
  //  dw1000.state = DW_WAIT_STX;
    dw1000.initial_heading = DW1000_INITIAL_HEADING;
    dw1000.pos.x = 5.f;
    dw1000.pos.y = 5.f;
    dw1000.pos.z = 0.f;
    dw1000.updated = false;
    for (uint8_t i = 0; i < DW1000_NB_ANCHORS; i++) {
    dw1000.anchors[i].distance = 0.f;
    dw1000.anchors[i].time = 0.f;
    dw1000.anchors[i].id = ids[i];
    dw1000.anchors[i].pos.x = pos_x[i];
    dw1000.anchors[i].pos.y = pos_y[i];
    dw1000.anchors[i].pos.z = pos_z[i];
    }

    // gps structure init
    dw1000.gps_dw1000.fix = GPS_FIX_NONE;
    dw1000.gps_dw1000.pdop = 0;
    dw1000.gps_dw1000.sacc = 0;
    dw1000.gps_dw1000.pacc = 0;
    dw1000.gps_dw1000.cacc = 0;
    dw1000.gps_dw1000.comp_id = GPS_DW1000_ID; //GPS_DW1000_ID

    struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
    llh_nav0.lat = NAV_LAT0;
    llh_nav0.lon = NAV_LON0;
    /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
    llh_nav0.alt = NAV_ALT0 + NAV_MSL0;
    ltp_def_from_lla_i(&dw1000.ltp_def, &llh_nav0); 

    /* Initialising the nodeState structure */ 
    // for(uint8_t i = DW1000_NB_ANCHORS; i < (DW1000_NB_ANCHORS+ DW1000_SERIAL_COMM_DIST_NUM_NODES);i++){
    for(uint8_t i = 0; i < (DW1000_SERIAL_COMM_DIST_NUM_NODES);i++){ 
      // states[i].nodeAddress = ids[i];
      states[i].nodeAddress = tag_ids[i];
    }

    // init trilateration algorithm !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//    trilateration_init(dw1000.anchors); //This is for trilateration
//    multilateration_init(dw1000.anchors); //This is fo multilateration
}

void local_and_comms_periodic(void) {
	struct EnuCoor_f *pos2 = stateGetPositionEnu_f();
  sendFloat(UWB_SERIAL_COMM_X, (*pos2).x);
  sendFloat(UWB_SERIAL_COMM_Y, (*pos2).y);
}

void local_and_comms_report(void) {
	struct EnuCoor_f *pos2 = stateGetPositionEnu_f();
    // struct EnuCoor_f *vel = stateGetSpeedEnu_f();
    // printf("%f,%f,%f,%f,%f,%f,%f,%f,%f \n",dw1000.anchors[0].distance,dw1000.anchors[1].distance,dw1000.anchors[2].distance,dw1000.anchors[3].distance,(*pos2).x,(*pos2).y,(*pos2).z,(*vel).x,(*vel).y); //for identification
    printf("%f,%f,%f,%f \n",(*pos2).x,(*pos2).y,states[1].x ,states[1].y ); //for identification
}

void local_and_comms_event(void) {
    dw1000_arduino_parse(&dw1000);
  
 
    if (dw1000.updated) {
    // if no timeout on anchors, run trilateration algorithm
      
//    int temp = trilateration_compute(dw1000.anchors, &dw1000.raw_pos); 		//This is for trilateration
//    int temp = multilateration_compute(dw1000.anchors, &dw1000.raw_pos);	//This is for LS multilateration
   /*int temp = nonLinLS_compute(dw1000.anchors, &dw1000.raw_pos, &dw1000.pos);			//This is for NLLS multilateration

    //int temp = 0;
    if (temp == 0) { //check_anchor_timeout(&dw1000) == false &&
        // apply scale and neutral corrections
        scale_position(&dw1000);
        // send fake GPS message for INS filters
        send_gps_dw1000_small(&dw1000);
      }
      if(temp == -1)
      {
        // printf("ERROR: trilateration failed \n");
      }*/
      dw1000.updated = false;
    }
}