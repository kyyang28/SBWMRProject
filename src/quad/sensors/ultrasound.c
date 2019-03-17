
#include <stdio.h>
#include "maths.h"
#include "target.h"
#include "sensors.h"
#include "runtime_config.h"

#ifdef ULTRASOUND

#include "ultrasound.h"

#define NUMBER_OF_SAMPLES					5

int16_t ultrasoundMaxRangeCm;

void ultrasoundInit(const ultrasoundConfig_t *ultrasoundConfig)
{
	ultrasoundRange_t ultrasoundRange;
	
	hcsr04_init(ultrasoundConfig, &ultrasoundRange);
	
	sensorSet(SENSOR_ULTRASOUND);
	
	ultrasoundMaxRangeCm = ultrasoundRange.maxRangeCm;
}

void ultrasound1Update(timeUs_t currentTimeUs)
{
	UNUSED(currentTimeUs);
	hcsr04_ultrasound1_start_sequence();
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
}

void ultrasound2Update(timeUs_t currentTimeUs)
{
	UNUSED(currentTimeUs);
	hcsr04_ultrasound2_start_sequence();
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
}

void ultrasound3Update(timeUs_t currentTimeUs)
{
	UNUSED(currentTimeUs);
	hcsr04_ultrasound3_start_sequence();
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
}

void ultrasound4Update(timeUs_t currentTimeUs)
{
	UNUSED(currentTimeUs);
	hcsr04_ultrasound4_start_sequence();
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
}

void ultrasound5Update(timeUs_t currentTimeUs)
{
	UNUSED(currentTimeUs);
	hcsr04_ultrasound5_start_sequence();
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
}

void ultrasound6Update(timeUs_t currentTimeUs)
{
	UNUSED(currentTimeUs);
	hcsr04_ultrasound6_start_sequence();
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
}

static int32_t applyUltrasound1MedianFilter(int32_t newUltrasoundData)
{
	static int32_t ultrasoundFilterSamples[NUMBER_OF_SAMPLES];
	static int currentSampleIndex = 0;
	static bool medianFilterEnabled = false;
	int nextSampleIndex;
	
	/* ULTRASOUND_OUT_OF_RANGE = -1 */
	if (newUltrasoundData > ULTRASOUND_OUT_OF_RANGE) {
		/* Only accept the data within the range */
		nextSampleIndex = (currentSampleIndex + 1);
		
		if (nextSampleIndex == NUMBER_OF_SAMPLES) {
			nextSampleIndex = 0;
			medianFilterEnabled = true;
		}
		
		ultrasoundFilterSamples[currentSampleIndex] = newUltrasoundData;
		currentSampleIndex = nextSampleIndex;
	}
	
	if (medianFilterEnabled) {
		return quickMedianFilter5(ultrasoundFilterSamples);
	} else {
		return newUltrasoundData;
	}
}

static int32_t applyUltrasound2MedianFilter(int32_t newUltrasoundData)
{
	static int32_t ultrasoundFilterSamples[NUMBER_OF_SAMPLES];
	static int currentSampleIndex = 0;
	static bool medianFilterEnabled = false;
	int nextSampleIndex;
	
	/* ULTRASOUND_OUT_OF_RANGE = -1 */
	if (newUltrasoundData > ULTRASOUND_OUT_OF_RANGE) {
		/* Only accept the data within the range */
		nextSampleIndex = (currentSampleIndex + 1);
		
		if (nextSampleIndex == NUMBER_OF_SAMPLES) {
			nextSampleIndex = 0;
			medianFilterEnabled = true;
		}
		
		ultrasoundFilterSamples[currentSampleIndex] = newUltrasoundData;
		currentSampleIndex = nextSampleIndex;
	}
	
	if (medianFilterEnabled) {
		return quickMedianFilter5(ultrasoundFilterSamples);
	} else {
		return newUltrasoundData;
	}
}

static int32_t applyUltrasound3MedianFilter(int32_t newUltrasoundData)
{
	static int32_t ultrasoundFilterSamples[NUMBER_OF_SAMPLES];
	static int currentSampleIndex = 0;
	static bool medianFilterEnabled = false;
	int nextSampleIndex;
	
	/* ULTRASOUND_OUT_OF_RANGE = -1 */
	if (newUltrasoundData > ULTRASOUND_OUT_OF_RANGE) {
		/* Only accept the data within the range */
		nextSampleIndex = (currentSampleIndex + 1);
		
		if (nextSampleIndex == NUMBER_OF_SAMPLES) {
			nextSampleIndex = 0;
			medianFilterEnabled = true;
		}
		
		ultrasoundFilterSamples[currentSampleIndex] = newUltrasoundData;
		currentSampleIndex = nextSampleIndex;
	}
	
	if (medianFilterEnabled) {
		return quickMedianFilter5(ultrasoundFilterSamples);
	} else {
		return newUltrasoundData;
	}
}

static int32_t applyUltrasound4MedianFilter(int32_t newUltrasoundData)
{
	static int32_t ultrasoundFilterSamples[NUMBER_OF_SAMPLES];
	static int currentSampleIndex = 0;
	static bool medianFilterEnabled = false;
	int nextSampleIndex;
	
	/* ULTRASOUND_OUT_OF_RANGE = -1 */
	if (newUltrasoundData > ULTRASOUND_OUT_OF_RANGE) {
		/* Only accept the data within the range */
		nextSampleIndex = (currentSampleIndex + 1);
		
		if (nextSampleIndex == NUMBER_OF_SAMPLES) {
			nextSampleIndex = 0;
			medianFilterEnabled = true;
		}
		
		ultrasoundFilterSamples[currentSampleIndex] = newUltrasoundData;
		currentSampleIndex = nextSampleIndex;
	}
	
	if (medianFilterEnabled) {
		return quickMedianFilter5(ultrasoundFilterSamples);
	} else {
		return newUltrasoundData;
	}
}

static int32_t applyUltrasound5MedianFilter(int32_t newUltrasoundData)
{
	static int32_t ultrasoundFilterSamples[NUMBER_OF_SAMPLES];
	static int currentSampleIndex = 0;
	static bool medianFilterEnabled = false;
	int nextSampleIndex;
	
	/* ULTRASOUND_OUT_OF_RANGE = -1 */
	if (newUltrasoundData > ULTRASOUND_OUT_OF_RANGE) {
		/* Only accept the data within the range */
		nextSampleIndex = (currentSampleIndex + 1);
		
		if (nextSampleIndex == NUMBER_OF_SAMPLES) {
			nextSampleIndex = 0;
			medianFilterEnabled = true;
		}
		
		ultrasoundFilterSamples[currentSampleIndex] = newUltrasoundData;
		currentSampleIndex = nextSampleIndex;
	}
	
	if (medianFilterEnabled) {
		return quickMedianFilter5(ultrasoundFilterSamples);
	} else {
		return newUltrasoundData;
	}
}

static int32_t applyUltrasound6MedianFilter(int32_t newUltrasoundData)
{
	static int32_t ultrasoundFilterSamples[NUMBER_OF_SAMPLES];
	static int currentSampleIndex = 0;
	static bool medianFilterEnabled = false;
	int nextSampleIndex;
	
	/* ULTRASOUND_OUT_OF_RANGE = -1 */
	if (newUltrasoundData > ULTRASOUND_OUT_OF_RANGE) {
		/* Only accept the data within the range */
		nextSampleIndex = (currentSampleIndex + 1);
		
		if (nextSampleIndex == NUMBER_OF_SAMPLES) {
			nextSampleIndex = 0;
			medianFilterEnabled = true;
		}
		
		ultrasoundFilterSamples[currentSampleIndex] = newUltrasoundData;
		currentSampleIndex = nextSampleIndex;
	}
	
	if (medianFilterEnabled) {
		return quickMedianFilter5(ultrasoundFilterSamples);
	} else {
		return newUltrasoundData;
	}
}

/* Get the last distance (in cm) measured by hcsr04 ultrasound sensor, ULTRASOUND_OUT_OF_RANGE is returned when greater than 4 meters */
int32_t ultrasound1Read(void)
{
	int32_t distance = hcsr04_ultrasound1_get_distance();
	
	if (distance > HCSR04_MAX_RANGE_CM) {
		distance = ULTRASOUND_OUT_OF_RANGE;				// ULTRASOUND_OUT_OF_RANGE = -1
	}

	return applyUltrasound1MedianFilter(distance);
//	return distance;
}

/* Get the last distance (in cm) measured by hcsr04 ultrasound sensor, ULTRASOUND_OUT_OF_RANGE is returned when greater than 4 meters */
int32_t ultrasound2Read(void)
{
	int32_t distance = hcsr04_ultrasound2_get_distance();
	
	if (distance > HCSR04_MAX_RANGE_CM) {
		distance = ULTRASOUND_OUT_OF_RANGE;				// ULTRASOUND_OUT_OF_RANGE = -1
	}

	return applyUltrasound2MedianFilter(distance);
//	return distance;
}

/* Get the last distance (in cm) measured by hcsr04 ultrasound sensor, ULTRASOUND_OUT_OF_RANGE is returned when greater than 4 meters */
int32_t ultrasound3Read(void)
{
	int32_t distance = hcsr04_ultrasound3_get_distance();
	
	if (distance > HCSR04_MAX_RANGE_CM) {
		distance = ULTRASOUND_OUT_OF_RANGE;				// ULTRASOUND_OUT_OF_RANGE = -1
	}

	return applyUltrasound3MedianFilter(distance);
//	return distance;
}

/* Get the last distance (in cm) measured by hcsr04 ultrasound sensor, ULTRASOUND_OUT_OF_RANGE is returned when greater than 4 meters */
int32_t ultrasound4Read(void)
{
	int32_t distance = hcsr04_ultrasound4_get_distance();
	
	if (distance > HCSR04_MAX_RANGE_CM) {
		distance = ULTRASOUND_OUT_OF_RANGE;				// ULTRASOUND_OUT_OF_RANGE = -1
	}

	return applyUltrasound4MedianFilter(distance);
//	return distance;
}

/* Get the last distance (in cm) measured by hcsr04 ultrasound sensor, ULTRASOUND_OUT_OF_RANGE is returned when greater than 4 meters */
int32_t ultrasound5Read(void)
{
	int32_t distance = hcsr04_ultrasound5_get_distance();
	
	if (distance > HCSR04_MAX_RANGE_CM) {
		distance = ULTRASOUND_OUT_OF_RANGE;				// ULTRASOUND_OUT_OF_RANGE = -1
	}

	return applyUltrasound5MedianFilter(distance);
//	return distance;
}

/* Get the last distance (in cm) measured by hcsr04 ultrasound sensor, ULTRASOUND_OUT_OF_RANGE is returned when greater than 4 meters */
int32_t ultrasound6Read(void)
{
	int32_t distance = hcsr04_ultrasound6_get_distance();
	
	if (distance > HCSR04_MAX_RANGE_CM) {
		distance = ULTRASOUND_OUT_OF_RANGE;				// ULTRASOUND_OUT_OF_RANGE = -1
	}

	return applyUltrasound6MedianFilter(distance);
//	return distance;
}

#endif
