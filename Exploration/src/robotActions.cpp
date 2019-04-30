/*
 * robotActions.cpp
 *
 *  Created on: 30 Apr 2019
 *      Author: zeive
 */


#include "robotActions.h"
#include <iostream>
#include <cmath>
#define MAX_X 25+2
#define MAX_Y 25+2



sensors check_sensors(float x,float y, short heading, short map[MAX_X][MAX_Y]){
	sensors sensorValues = {1,1,1};

	switch (heading){
	case 0: //left
		if(map[(int)x][(int)y-1] == -1)
			sensorValues.right = 0;
		if(map[(int)x-1][(int)y] == -1)
			sensorValues.front = 0;
		if(map[(int)x][(int)y+1] == -1)
			sensorValues.left = 0;
		break;
	case 1: // up
		if(map[(int)x-1][(int)y] == -1)
			sensorValues.right = 0;
		if(map[(int)x][(int)y+1] == -1)
			sensorValues.front = 0;
		if(map[(int)x+1][(int)y] == -1)
			sensorValues.left = 0;
		break;
	case 2: // right
		if(map[(int)x][(int)y+1] == -1)
			sensorValues.right = 0;
		if(map[(int)x+1][(int)y] == -1)
			sensorValues.front = 0;
		if(map[(int)x][(int)y-1] == -1)
			sensorValues.left = 0;
		break;
	case 3: // down
		if(map[(int)x+1][(int)y] == -1)
			sensorValues.right = 0;
		if(map[(int)x][(int)y-1] == -1)
			sensorValues.front = 0;
		if(map[(int)x-1][(int)y] == -1)
			sensorValues.left = 0;
		break;
	default:
		sensorValues = {1, 1, 1};
	}

	return sensorValues;
}

cellsInMap check_surrounds(float x, float y, short map[MAX_X][MAX_Y]){
	return {x, y, map[(int)x][(int)y]};
}

Point_t check_nearest_available(float x, float y, short map[MAX_X][MAX_Y]){
	short dist = 50;
	short low_dist = 50;
	float x_f = 0;
	float y_f = 0;

	for(int i = 0; i < MAX_X; i++){
		for(int j = 0; j< MAX_Y; j++){
			if(map[i][j] == 1){
				dist = sqrt((pow(x-i,2) + pow(y-j,2)));
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


Point_t move(float x,float y, short heading, short map[MAX_X][MAX_Y]){
	//saving last position
	float i_x = x;
	float i_y = y;
	short i_h = heading+1;
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
//	zero:not available -> obstacle
//	one:explored - avaliable
//	two:not explored
//	three:visited
	cellsInMap cells[4];
	for(int i = 0; i < 4; i++){
		switch (i){
		case 0:
			cells[i] = check_surrounds(x-1, y, map);//izquierda
			break;
		case 1:
			cells[i] = check_surrounds(x, y+1, map);//arriba
			break;
		case 2:
			cells[i] = check_surrounds(x+1, y, map);//derecha
			break;
		case 3:
			cells[i] = check_surrounds(x, y-1, map);//abajo
			break;
		}
	}

	Point_t coords = {i_x,i_y,i_h};

	for(int i = 0; i<4; i++){
		if(cells[i].value==2){
			coords.heading = i+1;
			break;
		}
		else
			if(cells[i].value ==1){
				coords.x = cells[i].x_c;
				coords.y = cells[i].y_c;
				coords.heading = i+1;
				break;
			}
	}

	if ((coords.x==i_x)&&(coords.y==i_y)&&(coords.heading==i_h)){
		coords = check_nearest_available(coords.x, coords.y, map);
	}


	return {coords.x,coords.y, coords.heading};
}

