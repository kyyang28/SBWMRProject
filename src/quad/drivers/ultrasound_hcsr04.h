
#ifndef __ULTRASOUND_HCSR04_H
#define __ULTRASOUND_HCSR04_H

#include <stdint.h>
#include "IOTypes.h"

#define HCSR04_MAX_RANGE_CM							400			// 4m from HC-SR04 datasheet
#define HCSR04_DETECTION_CONE_DECIDEGREES			300			// recommended cone angle is within 30 degrees
#define HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES	450			// 45 degrees seems to work well in practice

typedef struct ultrasoundConfig_s {
	ioTag_t triggerTag;
	ioTag_t echoTag;
}ultrasoundConfig_t;

typedef struct ultrasoundRange_s {
	int16_t maxRangeCm;
	int16_t detectionConeDeciDegrees;			// detection angle from HC-SR04 spec
	int16_t detectionConeExtendedDeciDegrees;	// device spec is conservative, in practice, the detection cone is slightly larger
}ultrasoundRange_t;

void hcsr04_init(const ultrasoundConfig_t *ultrasoundConfig, ultrasoundRange_t *ultrasoundRange);
void hcsr04_start_sequence(void);
int32_t hcsr04_get_distance(void);

#endif	// __ULTRASOUND_HCSR04_H
