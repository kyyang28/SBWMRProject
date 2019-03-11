
#include <stdio.h>
#include "target.h"
#include "sensors.h"
#include "runtime_config.h"

#ifdef ULTRASOUND

#include "ultrasound.h"

int16_t ultrasoundMaxRangeCm;

void ultrasoundInit(const ultrasoundConfig_t *ultrasoundConfig)
{
	ultrasoundRange_t ultrasoundRange;
	
	hcsr04_init(ultrasoundConfig, &ultrasoundRange);
	
	sensorSet(SENSOR_ULTRASOUND);
	
	ultrasoundMaxRangeCm = ultrasoundRange.maxRangeCm;
}

void ultrasoundUpdate(timeUs_t currentTimeUs)
{
	UNUSED(currentTimeUs);
	hcsr04_start_sequence();
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
}

/* Get the last distance (in cm) measured by hcsr04 ultrasound sensor, ULTRASOUND_OUT_OF_RANGE is returned when greater than 4 meters */
int32_t ultrasoundRead(void)
{
	int32_t distance = hcsr04_get_distance();
	
	if (distance > HCSR04_MAX_RANGE_CM) {
		distance = ULTRASOUND_OUT_OF_RANGE;				// ULTRASOUND_OUT_OF_RANGE = -1
	}

//	return applyUltrasoundMedianFilter(distance);
	return distance;
}

#endif
