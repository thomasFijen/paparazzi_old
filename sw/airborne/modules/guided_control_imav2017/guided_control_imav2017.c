/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/*
 * Steven_guided_ctrl.h
 *
 *  Created on: Sep 11, 2017
 *      Author: steven <stevenhelm@live.nl>
 */

#include <math.h>
// #include "modules/relativelocalizationfilter/relativelocalizationfilter.h"
#include "modules/decawave/uwb_localisation_and_comms.h"
#include "math/pprz_algebra_int.h"
#include "navigation.h"
#include "autopilot.h"
#include "../../firmwares/rotorcraft/guidance/guidance_h.h"
#include "../../firmwares/rotorcraft/guidance/guidance_v.h"
#include "generated/airframe.h"
#include "std.h"
#include "guided_control_imav2017.h"
#include <stdio.h>
#include <stdlib.h>

#define PI 3.14159265

static float counter = 0.0;

// // ABI messages
// #include "subsystems/abi.h"

// static abi_event uwb_ev;

// static float rec_velx = 0.0;
// static float rec_vely = 0.0;
// static float rec_range = 10.0;
// static float rec_height = 1.0;
// static float relxcom = 0.0;
// static float relycom = 1.5;

// static float oldxerr = 0.0;
// static float oldyerr = 0.0;

// float relvxerr = 0.0;
// float relvyerr = 0.0;

// float oldtime = 0.0;
// float newtime = 0.0;
// float dt = 0.0;

// static float pgainx = 0.2;
// static float pgainy = 0.2;
// static float dgain = 0.2;
// static float vgain = 0.5;

// static pthread_mutex_t ekf_mutex;

// static void keepBounded(float bound);
// static void uwb_cb(uint8_t sender_id __attribute__((unused)),
// 		uint8_t ac_id, float range, float trackedVx, float trackedVy, float trackedh);
// static void uwb_cb(uint8_t sender_id __attribute__((unused)),
// 		uint8_t ac_id, float range, float trackedVx, float trackedVy, float trackedh){
// 	rec_velx = trackedVx;
// 	rec_vely = trackedVy;
// 	rec_range = range;
// 	rec_height = trackedh;

// }

// void guided_control_imav2017_init(void){
// 	AbiBindMsgUWB(ABI_BROADCAST, &uwb_ev, uwb_cb); // Subscribe to the ABI RSSI messages
// 	oldtime = get_sys_time_usec()/pow(10,6);
// }

// void guided_control_imav2017_periodic(void){
	
// }

// First put both here.
bool hoverGuided(float cmd_height){
  float u_command[2] = {0,0};
  commandSpeed(u_command);

	bool temp = true;
	temp &= guidance_v_set_guided_z(-cmd_height);
	temp &= guidance_h_set_guided_vel(0.0,0.0);
	temp &= guidance_h_set_guided_heading(-0.576); //0.0 not reccommended if without a good heading estimate
	return !temp; // Returning FALSE means in the flight plan that the function executed successfully.
}

bool circle(){
  float velX = 0.0;
  float velY = 0.0;
          /* Fly in a square pattern */
        counter = counter + 1;
        if(counter <= 60){
            velX = 0.5;
            velY = 0;
        }else if (counter <= 120){
            velX = 0.0;
            velY = -0.5;
        }else if (counter <=180){
            velX = -0.5;
            velY = 0.0;
        }else if(counter <=240){
            velX = 0.0;
            velY = 0.5;
        }else{
            counter = 0.0;
        }

  /* Fly in a Circle pattern */
  // if(counter <= 360){
  //   velX = 0.5*cosf(counter*PI/180.0);
  //   velY = 0.5*sinf(counter*PI/180.0);
  //   counter = counter + 1;
  // } else{
  //   counter = 0.0;
  // }
 // struct EnuCoor_f *pos2 = stateGetPositionEnu_f();
 // printf("%f,%f,%f,%f \n",(*pos2).x,(*pos2).y,velX ,velY); //for identification

  // bool ret = guidance_h_set_guided_vel(velX,velY);
  bool ret = guidance_h_set_guided_body_vel(velY,velX);

  float u_command[2] = {velX,velY};
  commandSpeed(u_command);

  return ret;
}

// bool trackOther(float cmd_height){
// 	newtime = get_sys_time_usec()/pow(10,6);
// 	dt = newtime-oldtime;
// 	oldtime = newtime;

// 	bool temp = true;
// 	temp &= guidance_v_set_guided_z(-cmd_height);
// 	pthread_mutex_lock(&ekf_mutex);
// 	float relx = ekf[0].X[0];
// 	float rely = ekf[0].X[1];
// 	pthread_mutex_unlock(&ekf_mutex);

// 	float relxerr = relx-relxcom; //positive error means VX must increase
// 	float relyerr = rely-relycom; // positive error means VY must increase

// 	if(dt>0.0 && dt < 0.5 && oldxerr > 0.0 && oldyerr > 0.0){
// 		relvxerr = (relxerr-oldxerr)/dt;
// 		relvyerr = (relyerr-oldyerr)/dt;
// 		oldxerr = relxerr;
// 		oldyerr = relyerr;
// 	}

// 	float vxcommand = pgainx*relxerr+dgain*relvxerr+vgain*rec_velx;
// 	float vycommand = pgainy*relyerr+dgain*relvyerr+vgain*rec_vely;
// 	temp &= guidance_h_set_guided_vel(vxcommand,vycommand);
// 	return !temp; // Returning FALSE means in the flight plan that the function executed successfully.
// }

// bool trackRelPos(float cmd_height){
// 	newtime = get_sys_time_usec()/pow(10,6);
// 	dt = newtime-oldtime;
// 	oldtime = newtime;

// 	bool temp = true;
// 	temp &= guidance_v_set_guided_z(-cmd_height);
// 	pthread_mutex_lock(&ekf_mutex);
// 	float relx, rely;

// 	if(stateGetPositionEnu_f()->z > 1.0)
// 	{
// 		relx = ekf[0].X[0];
// 		rely = ekf[0].X[1];
// 	}
// 	else
// 	{
// 		relx = relxcom;
// 		rely = relycom;
// 	}
// 	pthread_mutex_unlock(&ekf_mutex);

// 	float relxerr = relx-relxcom; //positive error means VX must increase
// 	float relyerr = rely-relycom; // positive error means VY must increase

// 	/*
// 	if(relyerr>0){
// 		pgainy=0.2;
// 	}
// 	else{
// 		pgainy = 0.5;
// 	}*/

// 	float Vmag = sqrt(rec_velx*rec_velx+rec_vely*rec_vely);

// 	pgainx = 0.6;//+Vmag*0.5;
// 	pgainy = 0.2;//+Vmag*0.5;


// 	if(dt>0.0 && dt < 0.5 && abs(oldxerr) > 0.0 && abs(oldyerr) > 0.0){
// 		relvxerr = (relxerr-oldxerr)/dt;
// 		relvyerr = (relyerr-oldyerr)/dt;
// 		oldxerr = relxerr;
// 		oldyerr = relyerr;
// 	}

// 	float vxcommand = pgainx*relxerr+dgain*relvxerr;
// 	float vycommand = pgainy*relyerr+dgain*relvyerr;
// 	temp &= guidance_h_set_guided_vel(vxcommand,vycommand);
// 	//temp &= guidance_h_set_guided_heading(0.0); // not reccommended if without a good heading estimate
// 	return !temp; // Returning FALSE means in the flight plan that the function executed successfully.

// }

// bool trackVelocity(float cmd_height){
// 	keepBounded(2.0);
// 	bool temp = true;
// 	temp &= guidance_v_set_guided_z(-cmd_height);
// 	temp &= guidance_h_set_guided_vel(rec_velx,rec_vely);
// 	return !temp; // Returning FALSE means in the flight plan that the function executed successfully.
// }

// bool setForwardVelocity(float velx, float cmd_height){
// 	bool temp = true;
// 	temp &= guidance_v_set_guided_z(-cmd_height);
// 	temp &= guidance_h_set_guided_vel(velx,0);
// 	return !temp;
// }

// bool setForwardAndTrack(float velx, float cmd_height){
// 	bool temp = true;
// 	temp &= guidance_v_set_guided_z(-cmd_height);

// 	pthread_mutex_lock(&ekf_mutex);
// 	float relx = ekf[0].X[0];
// 	float rely = ekf[0].X[1];
// 	pthread_mutex_unlock(&ekf_mutex);

// 	float relxerr = relx-relxcom; //positive error means VX must increase
// 	float relyerr = rely-relycom; // positive error means VY must increase
// 	float vxcommand = pgainx*relxerr+velx;
// 	float vycommand = pgainy*relyerr;
// 	temp &= guidance_h_set_guided_vel(vxcommand,vycommand);
// 	return !temp; //Returning FALSE means in the flight plan that the function executed successfully.
// }

bool goLand(void){
  float u_command[2] = {0,0};
  commandSpeed(u_command);

	bool temp = true;
	temp &= guidance_v_set_guided_vz(0.1);
	temp &= guidance_v_set_guided_z(0.0);
	temp &= guidance_h_set_guided_vel(0.0,0.0);
	return !temp;
}

// static void keepBounded(float bound){
// 	if (abs(rec_velx)>bound){
// 		rec_velx = rec_velx / (abs(rec_velx)*bound);
// 	}
// 	if (abs(rec_vely)>bound){
// 		rec_vely = rec_vely / (abs(rec_vely)*bound);
// 	}
// }