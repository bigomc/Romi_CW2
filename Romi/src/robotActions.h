/*
 * robotActions.h
 *
 *  Created on: 30 Apr 2019
 *      Author: zeive
 */

#ifndef ROBOTACTIONS_H_
#include <Arduino.h>
#include "utils.h"
#include "mapping.h"
#define ROBOTACTIONS_H_
#define MAX_X 25+2
#define MAX_Y 25+2


struct cellsInMap{
	float x_c;
	float y_c;
	byte value;
};

enum Heading_t {
	WEST_DIR = 1,
	NORTH_DIR = 2,
	EAST_DIR = 3,
	SOUTH_DIR = 4,
	UNKNOWN_DIR = 5
};

struct sensors{
	byte right;
	byte front;
	byte left;
};

sensors check_sensors(float x,float y, short heading, short map[MAX_X][MAX_Y]);
cellsInMap check_surrounds(float x, float y, short map[MAX_X][MAX_Y]);
Point_t check_nearest_available(float x, float y);

Heading_t radToHeading(float rad);

Point_t move(float x,float y, float rad, Mapper map);


#endif /* ROBOTACTIONS_H_ */
