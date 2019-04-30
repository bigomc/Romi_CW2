//============================================================================
// Name        : Exploration.cpp
// Author      : Ruiz
// Version     :
// Copyright   : This is my happy Code for the Exploration translated from Matlab by Esteban Ochoa!
// Description :Exploration file in C++, previous to Arduino Ansi-style
//============================================================================

#include <iostream>
#include "RobotActions.h"
using namespace std;

//DEFINE THE 2-D MAP ARRAY

#define MAX_X 25+2
#define MAX_Y 25+2

void printMap();
void mapping(int x, int y, short heading, sensors sensorValue);

//This array stores the coordinates of the map and the
//Objects in each coordinate

unsigned char map_array [MAX_X][MAX_Y];
short r_map_array [MAX_X][MAX_Y];


// Obtain Obstacle, Target and Robot Position
// Initialize the MAP with input values
// Obstacle=-1,Target = 0,Robot=1,Space=2

//Only to create the grid
//axis([1 MAX_X+1 1 MAX_Y+1])

short n=0; //number of obstacles
short heading = 0;

int position[2] = {2,2};



int main() {
//	map_coord coords = {0, 0, false};

/*
 * Writing the map
 */
		// Draw the initial values of grid of the robot
	sensors sensorValues;
	Point_t coords = {10, 10, 2};


	for (int i =0; i < MAX_X; i ++){
		for (int j =0; j < MAX_Y; j++){
			if(i == 0 || i == (MAX_X-1) || j == 0 || j == (MAX_Y-1)){
				r_map_array[i][j] = -1;
			}
			else{
				r_map_array[i][j] = 2;
			}
		}
	}


	while(!(coords.x<=0 || coords.y<=0)){

		sensorValues = check_sensors(coords.x, coords.y, coords.heading, r_map_array);
		mapping((int)coords.x, (int)coords.y, coords.heading, sensorValues);
		coords = move(coords.x,coords.y,coords.heading, r_map_array);
		cout << "X: ";
		cout << coords.x;
		cout << "Y: ";
		cout << coords.y << endl;
	}


	printMap();

	cout << "!!!Hello World Exploration!!!" << endl; // prints !!!Hello World!!!
	return 0;
}

void printMap(){
	short character = -5;
	//printing map
	for (int i = 0; i < MAX_X; i++) {
		for (int j = 0; j < MAX_Y; j++) {
			cout << r_map_array[i][j];
			cout << " ";
		}
		cout << endl;
	}
}


void mapping(int x, int y, short heading, sensors sensorValue){
	r_map_array[x][y] = 3;

	switch(heading){
	case 0://left
		if (r_map_array[x][y-1]!=3)
			r_map_array[x][y-1] = sensorValue.left;
		if (r_map_array[x-1][y]!=3)
			r_map_array[x-1][y] = sensorValue.front;
		if (r_map_array[x][y+1]!=3)
			r_map_array[x][y+1] = sensorValue.right;
		break;
	case 1://up
		if (r_map_array[x-1][y]!=3) r_map_array[x-1][y] = sensorValue.left;
		if (r_map_array[x][y+1]!=3) r_map_array[x][y+1] = sensorValue.front;
		if (r_map_array[x+1][y]!=3) r_map_array[x+1][y] = sensorValue.right;
		break;
	case 2: //right
		if (r_map_array[x][y+1]!=3) r_map_array[x][y+1] = sensorValue.left;
		if (r_map_array[x+1][y]!=3) r_map_array[x+1][y] = sensorValue.front;
		if (r_map_array[x][y-1]!=3) r_map_array[x][y-1] = sensorValue.right;
		break;
	case 3:
		if (r_map_array[x+1][y]!=3) r_map_array[x+1][y] = sensorValue.left;
		if (r_map_array[x][y-1]!=3) r_map_array[x][y-1] = sensorValue.front;
		if (r_map_array[x-1][y]!=3) r_map_array[x-1][y] = sensorValue.right;
		break;

	}
}

