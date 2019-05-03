#ifndef _Mapping_h
#define _Mapping_h
#include <Arduino.h>
#include <EEPROM.h>


const byte MAP_RESOLUTION = 25;
const byte MAP_DEFAULT_FEATURE = '#';
const int MAP_X=1800;
const int MAP_Y=1800;
const int C_HALF_WIDTH = 36;

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
			BORDER,
            RFID, //1
            LINE, //2
            VISITED, //3
            EXPLORED,//4
            UNKNOWN//5
        };
        int  indexToPose(int i, int map_size, int resolution);
        int  poseToIndex(int x, int map_size, int resolution);

    private:
        int X_size;
        int Y_size;
        const char *symbols = "ORL.*#";
};

#endif
