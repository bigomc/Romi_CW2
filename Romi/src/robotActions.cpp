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
#define CELL 1



Point_t check_nearest_available(float x, float y, Mapper map){
	short dist = 50;
	short low_dist = 50;
	float x_f = 0;
	float y_f = 0;

	for(int i = 0; i < MAX_X; i++){
		for(int j = 0; j< MAX_Y; j++){
			if(map.readEeprom((x-CELL)*(MAP_X / MAP_RESOLUTION) +36, y*(MAP_Y / MAP_RESOLUTION)+36) == map.EXPLORED){
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
			cells[i] = {x-CELL, y, map.readEeprom((x-CELL)*(MAP_X / MAP_RESOLUTION) +36, y*(MAP_Y / MAP_RESOLUTION)+36)};//left
			break;
		case NORTH_DIR:
			cells[i] = {x, y+CELL, map.readEeprom(x*(MAP_X / MAP_RESOLUTION)+36, (y+CELL)*(MAP_Y / MAP_RESOLUTION)+36)};//up
			break;
		case EAST_DIR:
			cells[i] = {x+CELL, y, map.readEeprom((x+CELL)*(MAP_X / MAP_RESOLUTION)+36, y*(MAP_Y / MAP_RESOLUTION)+36)};//right
			break;
		case SOUTH_DIR:
			cells[i] = {x, y-CELL, map.readEeprom(x*(MAP_X / MAP_RESOLUTION)+36, (y-CELL)*(MAP_Y / MAP_RESOLUTION)+36)};//down
			break;
		}
	}

	int b = cells[2].value;
	int c = cells[3].value;
	int d = cells[4].value;
	int a = cells[1].value;


	Point_t coords = {i_x,i_y,i_h};

	short av = 0;
	short ne = 0;

	for (int i = 1; i < 5; i++)
	{
		if (cells[i].value == 4)
		{
			av = i;
			break;
		}
		if (cells[i].value == 5)
		{
			ne = i;
			//break;
		}
	}

	short j = 0;

	if (av != 0)
	{
		if (cells[2].value == 4)
		{
			j = 2;
		}
		else if (cells[4].value == 4)
		{
			j = 4;
		}
		else
		{
			j = av;
		}
		coords.x = cells[j].x_c;
		coords.y = cells[j].y_c;
		coords.heading = j;
	}
	else if (ne != 0)
	{
		if (cells[2].value == 5)
		{
			coords.heading = 2;
		}
		else if (cells[4].value == 5)
		{
			coords.heading = 4;
		}
		else
		{
			coords.heading = ne;
		}
	}

	if ((coords.x==i_x)&&(coords.y==i_y)&&(coords.heading==i_h)){
		coords = check_nearest_available(coords.x, coords.y, map);
	}


	return {coords.x,coords.y, coords.heading};
}
