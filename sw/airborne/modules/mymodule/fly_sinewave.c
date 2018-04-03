/*
 * Copyright (C) Thomas Fijen
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/mymodule/fly_sinewave.c"
 * @author Thomas Fijen
 * This function is used to allow the UAV to follow a sinewave trajectory.
 */

#include "modules/mymodule/fly_sinewave.h"
#include "autopilot_guided.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"


void fly_sinewave_init(void)
{

}


bool fly_sinewave_run()
{
	return false
}

/*
uint8_t my_moveWaypointLeft(uint8_t waypoint, float distanceMeters)
{
	struct EnuCoor_i new_coor;
	new_coor->x = waypoint_get_x(waypoint) + distanceMeters;
	new_coor->y = waypoint_get_x(waypoint);

	waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
	return false
}
*/

