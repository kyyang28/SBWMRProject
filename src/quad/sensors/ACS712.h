#ifndef __ACS712_H
#define __ACS712_H

#include <stdint.h>

typedef struct motorCurrentMeterConfig_s {
	int16_t currentMeterScale;
	float leftMotorCurrentMeterOffset;
	float rightMotorCurrentMeterOffset;	
//	int16_t currentMeterOffset;
}motorCurrentMeterConfig_t;

//extern float unfilteredCurrentMeterValue1;
//extern float unfilteredCurrentMeterValue2;
extern float filteredLeftMotorCurrentMeterValue;
extern float filteredRightMotorCurrentMeterValue;
//extern int32_t meanFilteredLeftMotorCurrentMeterValue;
//extern int32_t meanFilteredRightMotorCurrentMeterValue;

void updateACS712LeftMotorCurrentSensor(int32_t lastUpdateAt);
void updateACS712RightMotorCurrentSensor(int32_t lastUpdateAt);

#endif	// __ACS712_H
