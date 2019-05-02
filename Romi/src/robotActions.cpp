/*
 * robotActions.cpp
 *
 *  Created on: 30 Apr 2019
 *      Author: zeive
 */

#include <Arduino.h>
#include "robotActions.h"
#define MAX_X 25+2
#define MAX_Y 25+2
#define TOLERANCE 0.78



sensors check_sensors(float x,float y, short heading, short map[MAX_X][MAX_Y]){
	sensors sensorValues = {1,1,1};

	switch (heading){
	case 1: //left
		if(map[(int)x][(int)y-1] == -1)
			sensorValues.left = 0;
		if(map[(int)x-1][(int)y] == -1)
			sensorValues.front = 0;
		if(map[(int)x][(int)y+1] == -1)
			sensorValues.right = 0;
		break;
	case 2: // up
		if(map[(int)x-1][(int)y] == -1)
			sensorValues.left = 0;
		if(map[(int)x][(int)y+1] == -1)
			sensorValues.front = 0;
		if(map[(int)x+1][(int)y] == -1)
			sensorValues.right = 0;
		break;
	case 3: // right
		if(map[(int)x][(int)y+1] == -1)
			sensorValues.left = 0;
		if(map[(int)x+1][(int)y] == -1)
			sensorValues.front = 0;
		if(map[(int)x][(int)y-1] == -1)
			sensorValues.right = 0;
		break;
	case 4: // down
		if(map[(int)x+1][(int)y] == -1)
			sensorValues.left = 0;
		if(map[(int)x][(int)y-1] == -1)
			sensorValues.front = 0;
		if(map[(int)x-1][(int)y] == -1)
			sensorValues.right = 0;
		break;
	default:
		sensorValues = {1, 1, 1};
	}

	return sensorValues;
}


Point_t check_nearest_available(float x, float y, short map[MAX_X][MAX_Y]){
	short dist = 50;
	short low_dist = 50;
	float x_f = 0;
	float y_f = 0;

	for(int i = 0; i < MAX_X; i++){
		for(int j = 0; j< MAX_Y; j++){
			if(map[i][j] == 1){
				dist = sqrt((x-i)*(x-i) + (y-j)*(y-j));
			}
			if(dist<low_dist){
				low_dist = dist;
				x_f = float(i);
				y_f = float(j);
			}
		}
	}

	return {x_f, y_f, 2};
}


Heading_t radToHeading(float rad){
	if(rad < (0+TOLERANCE) && rad > (0-TOLERANCE))
		return EAST_DIR;
	if(rad < ((PI/2)+TOLERANCE) && rad > ((PI/2)-TOLERANCE))
		return NORTH_DIR;
	if(rad > (PI-TOLERANCE) && rad < (-PI+TOLERANCE))
		return WEST_DIR;
	if(rad < (-(PI/2)+TOLERANCE) && rad > (-(PI/2)-TOLERANCE))
		return SOUTH_DIR;

}


Point_t move(float x,float y, float rad, Mapper map){

	Heading_t heading = radToHeading(rad);

	//saving last position
	float i_x = x;
	float i_y = y;
	Heading_t i_h = heading;
//heading = where am i pointing at

//	Go to the next unexplored cell
//	Try to mantain the up or down heading
//	My previous was up or down?
//	Can I continue with my previous? not? head to right and move one step
//	Can I go up? Was explored?
//	Can I go down? Was explored?
//	Can I move to the right? not? move to the previous position -> that means
//	rotate 180 and move until a non-explored area, then right
//
//
//	Where am I
//	Chech suround> Whera can I go?
//	From where can I go? they are already explored?
//	YES: look for another NO:go there
//	If all are explored return previous cell, and repeat the process
//
//	0 available -> obstacle
//	1:explored - avaliable
//	2:not explored
//	3:visited

	cellsInMap cells[5];
	for(int i = 1; i < 5; i++){
		switch (i){
		case WEST_DIR:
			cells[i] = {x-1, y, map.readEeprom(x-1, y)};//left
			break;
		case NORTH_DIR:
			cells[i] = {x, y+1, map.readEeprom(x, y+1)};//up
			break;
		case EAST_DIR:
			cells[i] = {x+1, y, map.readEeprom(x+1, y)};//right
			break;
		case SOUTH_DIR:
			cells[i] = {x, y-1, map.readEeprom(x, y-1)};//down
			break;
		}
	}

	Point_t coords = {i_x,i_y,i_h};

	for(int i = 1; i<5; i++){
		if(cells[i].value==2){
			coords.heading = i;
			break;
		}
		else
			if(cells[i].value ==1){
				coords.x = cells[i].x_c;
				coords.y = cells[i].y_c;
				coords.heading = i;
				break;
			}
	}

	// if ((coords.x==i_x)&&(coords.y==i_y)&&(coords.heading==i_h)){
		// coords = check_nearest_available(coords.x, coords.y, map);
	// }


	return {coords.x,coords.y, coords.heading};
}
