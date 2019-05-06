/*
 * robotActions.h
 *
 *  Created on: 30 Apr 2019
 *      Author: zeive
 */

#ifndef ROBOTACTIONS_H_
#define ROBOTACTIONS_H_
#define MAX_X 25+2
#define MAX_Y 25+2


struct Point_tag {
    float x;
    float y;
    short heading;
} typedef Point_t;

struct cellsInMap{
	float x_c;
	float y_c;
	short value;
};

struct sensors{
	short right;
	short front;
	short left;
};

sensors check_sensors(float x,float y, short heading, short map[MAX_X][MAX_Y]);
cellsInMap check_surrounds(float x, float y, short map[MAX_X][MAX_Y]);
Point_t check_nearest_available(float x, float y);

Point_t move(float x,float y, short heading, short map[MAX_X][MAX_Y]);


#endif /* ROBOTACTIONS_H_ */
