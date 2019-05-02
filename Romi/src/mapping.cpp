#include "mapping.h"



void Mapper::resetMap()
{

    for (int i=0;i<MAP_RESOLUTION;i++)
    {
        for (int j=0;j<MAP_RESOLUTION;j++)
        {
            int eeprom_address = (i*MAP_RESOLUTION)+j;

            if (eeprom_address > 1023)
            {
                Serial.println(F("Error: EEPROM Address greater than 1023"));
            }
            else
            {
                EEPROM.update(eeprom_address, UNKNOWN );
            }
        }
    }

}

void Mapper::printMap()
{

    Serial.println("Map");
    for (int i = MAP_RESOLUTION - 1; i >= 0; i--)
    {
        for(int j=0;j<MAP_RESOLUTION;j++)
        {
            int eeprom_address = (j*MAP_RESOLUTION)+i;
            byte value;
            value = EEPROM.read(eeprom_address);//, value);
            Serial.print( (char)symbols[value] );
            Serial.print(" ");
        }
        Serial.println("");
    }

}

int Mapper::poseToIndex(int x, int map_size, int resolution)
{
    return x / (map_size / resolution);
}

int Mapper::indexToPose(int i, int map_size, int resolution)
{
    return i* (map_size / resolution);
}


void Mapper::updateMapFeature(byte feature, float y, float x) {
    byte current = readEeprom (x, y);
    if(feature < current) {
        updateMapFeature( feature, (int)y, (int)x );
    }
}

void Mapper::updateMapFeature(byte feature, int y, int x)
{
    if (x > MAP_X || x < 0 || y > MAP_Y || y < 0)
    {
      return;
    }

    int x_index = poseToIndex(x, MAP_X, MAP_RESOLUTION);
    int y_index = poseToIndex(y, MAP_Y, MAP_RESOLUTION);

    int eeprom_address = (x_index * MAP_RESOLUTION) + y_index;

    if (eeprom_address > 1023)
    {
        Serial.println(F("Error: EEPROM Address greater than 1023"));
    }
    else
    {
        EEPROM.update(eeprom_address, feature);
    }
}

byte Mapper::readEeprom (float x, float y){
	
	if((x >= 1800) || (y >= 1800) || (x<0) || (y<0)
		return 255; //Determine if it is out of bounds or wall detection
	else{
		byte value;
		int x_index = poseToIndex(x, MAP_X, MAP_RESOLUTION);
		int y_index = poseToIndex(y, MAP_Y, MAP_RESOLUTION);
		int eeprom_address = (x_index * MAP_RESOLUTION) + y_index;
		value = EEPROM.read(eeprom_address);
		return value;
	}
}
