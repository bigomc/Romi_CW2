#ifndef _Mapping_h
#define _Mapping_h
#include <Arduino.h>
#include <EEPROM.h>


const byte MAP_RESOLUTION = 25;
const byte MAP_DEFAULT_FEATURE = '#';
const int MAP_X=1800;
const int MAP_Y=1800;
const int C_HALF_WIDTH = 36;
const int BORDER_SIZE = 72;
const int X_LENGHT = 19;
const int Y_LENGHT = 19;
const int X_ORIGIN = 900;
const int Y_ORIGIN = 900;

class Mapper
{
    public:
        void resetMap();
        void printMap();
        void updateMapFeature(byte feature, int y, int x);
        void updateMapFeature(byte feature, float y, float x);
        byte readEeprom (float x, float y);
        enum Feature{
            OBSTACLE, //0
            LINE, //1
			BORDER, //2
            RFID, //3
            VISITED, //4
            EXPLORED,//5
            UNKNOWN//6
        };
        int  indexToPose(int i, int map_size, int resolution);
        int  poseToIndex(int x, int map_size, int resolution);

    private:
        int X_size;
        int Y_size;
        const char *symbols = "OLBR.*#";
};

#endif
