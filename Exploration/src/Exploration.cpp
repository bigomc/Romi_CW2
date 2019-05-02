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
Point_t coords = { 10, 10, 2 };

int main() {
//	map_coord coords = {0, 0, false};

/*
 * Writing the map
 */
		// Draw the initial values of grid of the robot
	sensors sensorValues;


	for (int i =0; i < MAX_X; i ++){
		for (int j =0; j < MAX_Y; j++){
			if(i == 0 || i==1 || i == (MAX_X-1) || i == (MAX_X - 2) || j == 0 || j == 1 || j == (MAX_Y-1) || j == (MAX_Y - 2)){
				r_map_array[i][j] = -1;
			}
			else{
				r_map_array[i][j] = 2;
			}
		}
	}

	r_map_array[8][21] = -1;
	r_map_array[8][22] = -1;
	r_map_array[8][23] = -1;
	r_map_array[8][24] = -1;
	r_map_array[8][20] = -1;

	r_map_array[16][5] = -1;
	r_map_array[16][6] = -1;
	r_map_array[16][7] = -1;
	r_map_array[16][8] = -1;
	r_map_array[16][9] = -1;

	r_map_array[10][12] = -1;
	r_map_array[10][13] = -1;
	r_map_array[11][13] = -1;
	r_map_array[11][12] = -1;

	printMap();

	while(!(coords.x<=0 || coords.y<=0)){

		sensorValues = check_sensors(coords.x, coords.y, coords.heading, r_map_array);
		mapping((int)coords.x, (int)coords.y, coords.heading, sensorValues);
		coords = move(coords.x,coords.y,coords.heading, r_map_array);
//		cout << "X: ";
//		cout << coords.x;
//		cout << "Y: ";
//		cout << coords.y << endl;
		system("CLS");
		printMap();
	}


	printMap();

	cout << "!!!Hello World Exploration!!!" << endl; // prints !!!Hello World!!!
	return 0;
}

void printMap(){
	char character = 'x';
	//printing map
	for (int i = 0; i < MAX_X; i++) {
		for (int j = 0; j < MAX_Y; j++) {
			switch (r_map_array[i][j])
			{
			case -1:
				character = 'x';
				break;
			case 0:
				character = '#';
				break;
			case 1:
				character = 'o';
				break;
			case 2:
				character = '^';
				break;
			case 3:
				character = '.';
				break;
			default:
				break;
			}
			if ((i==coords.x)&&(j==coords.y))
			{
				character = '&';
			}
			cout << character;
			cout << " ";
		}
		cout << endl;
	}
}


void mapping(int x, int y, short heading, sensors sensorValue){
	r_map_array[x][y] = 3;

	switch(heading){
	case 1://left
		if (r_map_array[x-2][y-1]!=3) r_map_array[x-2][y-1] = sensorValue.left;
		if (r_map_array[x - 2][y] != 3) {
			r_map_array[x - 2][y] = sensorValue.front;
			if (sensorValue.front==1)
			{
				r_map_array[x - 1][y] = 1;
			}
		}
		if (r_map_array[x-2][y+1]!=3) r_map_array[x-2][y+1] = sensorValue.right;
		break;
	case 2://up
		if (r_map_array[x-1][y+2]!=3) r_map_array[x-1][y+2] = sensorValue.left;
		if (r_map_array[x][y + 2] != 3) {
			r_map_array[x][y + 2] = sensorValue.front;
			if (sensorValue.front==1)
			{
				r_map_array[x][y + 1] = 1;
			}
		}
		if (r_map_array[x+1][y+2]!=3) r_map_array[x+1][y+2] = sensorValue.right;
		break;
	case 3: //right
		if (r_map_array[x+2][y+1]!=3) r_map_array[x+2][y+1] = sensorValue.left;
		if (r_map_array[x + 2][y] != 3) {
			r_map_array[x + 2][y] = sensorValue.front;
			if (sensorValue.front==1)
			{
				r_map_array[x + 1][y] = 1;
			}
		}
		if (r_map_array[x+2][y-1]!=3) r_map_array[x+2][y-1] = sensorValue.right;
		break;
	case 4: //down
		if (r_map_array[x+1][y-2]!=3) r_map_array[x+1][y-2] = sensorValue.left;
		if (r_map_array[x][y - 2] != 3) {
			r_map_array[x][y - 2] = sensorValue.front;
			if (sensorValue.front==1)
			{
				r_map_array[x][y - 1] = 1;
			}
		}
		if (r_map_array[x-1][y-2]!=3) r_map_array[x-1][y-2] = sensorValue.right;
		break;

	}
}

