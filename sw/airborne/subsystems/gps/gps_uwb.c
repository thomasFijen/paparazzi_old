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
 * @file "modules/decawave/gps_uwb.c"
 * @author Thomas Fijen
 * Use this file so that the UWB can be simulated as a primary GPS structure
 */

#include "generated/flight_plan.h"        // reference lla NAV_XXX0
#include "subsystems/gps/gps_uwb.h"
#include "subsystems/gps.h"
#include "subsystems/abi.h"

//struct LtpDef_i ltp_def;

struct GpsState gps_uwb;
static uint32_t now_ts;	//This just needs to be double checked to ensure it is defined correctly
static bool updateGPS;

void gps_uwb_init(void) 
{
	gps_uwb.fix = GPS_FIX_NONE;
	gps_uwb.pdop = 0;
	gps_uwb.sacc = 0;
	gps_uwb.pacc = 0;
	gps_uwb.cacc = 0;

	gps_uwb.comp_id = GPS_DW1000_ID;
	updateGPS = 0;

//	struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
//	llh_nav0.lat = NAV_LAT0;
//	llh_nav0.lon = NAV_LON0;
	/* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
//	llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

//	ltp_def_from_lla_i(&ltp_def, &llh_nav0);
}

void gps_uwb_event(void)
{
	if (updateGPS)
	{
		AbiSendMsgGPS(GPS_DW1000_ID, now_ts, &(gps_uwb));
		updateGPS = 0;
		//printf("TEST: gps_uwb_event time: %d \n",now_ts);
	}

}

void update_uwb(uint32_t ts, struct GpsState *gps_dw1000)
{	
	now_ts = ts;
	gps_uwb = *gps_dw1000;
	updateGPS = 1;
	//printf("TEST: update_UWB time: %d \n",ts);
}


