
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "maths.h"
#include "adc.h"
#include "filter.h"
//#include "ACS712.h"
#include "configMaster.h"

#define MOTOR_CURRENT_LPF_CUTOFF_FREQ					0.4f
#define MOTOR_CURRENT_FILTER_SAMPLING_FREQ				80000			// 80000 Hz

#define ADC_VREF										3300			// in mV, 3300 mV = 3.3 V

#define NUMBER_OF_SAMPLES								5

/* VNH5019 current sensing module has roughly 144 mV / A */

/* currentMeterValue in milliamp (mA) */
//float unfilteredCurrentMeterValue1 = 0.0f;
//float unfilteredCurrentMeterValue2 = 0.0f;
float filteredLeftMotorCurrentMeterValue = 0.0f;
float filteredRightMotorCurrentMeterValue = 0.0f;

//int32_t meanFilteredLeftMotorCurrentMeterValue = 0;
//int32_t meanFilteredRightMotorCurrentMeterValue = 0;

static float g_leftMotorMilliVoltsScaled3V3;
static float g_leftMotorMilliVoltsScaled5V;
static float g_rightMotorMilliVoltsScaled3V3;
static float g_rightMotorMilliVoltsScaled5V;

/* As ACS712 output voltage is 5V tolerance, STM32F407 ADC pin is 3.3V tolerance, 3.3K and 1.8K ohm resistors are utilised for voltage drop from 5V to 3V3 */
static float voltageDivider_R1 = 1.8f;		// R1 = 1.8k ohm
static float voltageDivider_R2 = 3.3f;		// R2 = 3.3k ohm

static char dataLog[100];

#if 0
/* Return value in milliamps (mA) */
static float convertADCCountToMilliAmps(uint16_t adcValue)
{
	float milliVolts;

//	printf("resolutionScale: %u\r\n", AdcConfig()->resolutionScale);
	
	/* ADC_VREF = 3300 mV = 3.3 V
	 * 
	 * ADC resolution is 12 bits, 2^12 = 4096
	 * 
	 * (3300 mV / 4096(2^12) - offset) / 144 (mV/A) = 0.00559488932291667 (Amps)
	 *
	 * 0.00559488932291667 (Amps) * 1000 = 5.59488932291667 (Milliamps)
	 */
	milliVolts = ((uint32_t)adcValue * ADC_VREF) / AdcConfig()->resolutionScale;		// ADC resolution 12-bit, resolutionScale = 2^12 = 4096
	milliVolts -= MotorCurrentMeterConfig()->currentMeterOffset;
	
//	printf("offset: %u\r\n", MotorCurrentMeterConfig()->currentMeterOffset);			// 0
//	printf("scale: %u\r\n", MotorCurrentMeterConfig()->currentMeterScale);				// 140
	
	/* Multiply by 1000 to convert amps to milliamps */
	return (1000 * milliVolts / MotorCurrentMeterConfig()->currentMeterScale);		// 140 mV / A for VNH5019 Motor Driver Sensing Unit
}
#endif

static int32_t calculateMeanValue(int32_t *samples)
{
	int32_t sumOfSamples = 0;
	
	for (int i = 0; i < NUMBER_OF_SAMPLES; i++) {
		sumOfSamples += samples[i];
	}
	
	return sumOfSamples / NUMBER_OF_SAMPLES;
}

static int32_t applyMeanCurrentValuesFilter(int32_t newCurrentMeterData)
{
	static int32_t meanCurrentFilterSamples[NUMBER_OF_SAMPLES];
	static int currentSampleIndex = 0;
	static bool meanFilterEnabled = false;
	int nextSampleIndex;

	if (newCurrentMeterData > 0) {
		nextSampleIndex = (currentSampleIndex + 1);
		
		if (nextSampleIndex == NUMBER_OF_SAMPLES) {
			nextSampleIndex = 0;
			meanFilterEnabled = true;
		}
		
		meanCurrentFilterSamples[currentSampleIndex] = newCurrentMeterData;
		currentSampleIndex = nextSampleIndex;
	}
	
	if (meanFilterEnabled) {
		return quickMedianFilter5(meanCurrentFilterSamples);
//		return calculateMeanValue(meanCurrentFilterSamples);
	} else {
		return newCurrentMeterData;
	}
}

static void updateACS712LeftMotorCurrent(void)
{
	static biquadFilter_t motorCurrentFilter;
	static bool isMotorCurrentFilterInitialised;
	
	if (!isMotorCurrentFilterInitialised) {
		biquadFilterInitLPF(&motorCurrentFilter, MOTOR_CURRENT_LPF_CUTOFF_FREQ, MOTOR_CURRENT_FILTER_SAMPLING_FREQ);
		isMotorCurrentFilterInitialised = true;
	}
	
	/* Get ADC value */
	uint16_t leftMotorCurrentSample = adcGetChannelSample(ADC_MOTOR_CURRENT1);
	
//	printf("leftMotorCurrentSample: %u\r\n", leftMotorCurrentSample);
	
	/* Convert ADC value to amps */
	/* Apply biquad 2nd-order low pass filter */
	leftMotorCurrentSample = biquadFilterApply(&motorCurrentFilter, leftMotorCurrentSample);
	
	g_leftMotorMilliVoltsScaled3V3 = (leftMotorCurrentSample / AdcConfig()->resolutionScale) * ADC_VREF;			// ADC_VREF = 3300 mV
	g_leftMotorMilliVoltsScaled5V = g_leftMotorMilliVoltsScaled3V3 * ((voltageDivider_R2 + voltageDivider_R1) / voltageDivider_R2);
	
//	printf("milliVolts: %.4f\t", g_leftMotorMilliVoltsScaled3V3);
//	printf("milliVoltsOrig: %.4f\t", g_leftMotorMilliVoltsScaled5V);

	filteredLeftMotorCurrentMeterValue = (g_leftMotorMilliVoltsScaled5V - MotorCurrentMeterConfig()->leftMotorCurrentMeterOffset) / MotorCurrentMeterConfig()->currentMeterScale;

	/* Using 5V voltage regulator, there is an 1 Amps offset coming out of the calculated current output value */
	filteredLeftMotorCurrentMeterValue = filteredLeftMotorCurrentMeterValue - 1.0f;
	
	/* IMPORTANT: For SBWMR 1, inverting hardware IP+ and IP- of current sensor configurations, so flip the sign of current data value */
//	filteredLeftMotorCurrentMeterValue = -filteredLeftMotorCurrentMeterValue;

//	sprintf(dataLog, "%.4f,%.4f,%.4f,%.4f", (float)leftMotorCurrentSample, g_leftMotorMilliVoltsScaled3V3, g_leftMotorMilliVoltsScaled5V, filteredRightMotorCurrentMeterValue);
//	printf("%s\r\n", dataLog);

//	unfilteredCurrentMeterValue1 = convertADCCountToMilliAmps(leftMotorCurrentSample);
//	filteredLeftMotorCurrentMeterValue = convertADCCountToMilliAmps(biquadFilterApply(&motorCurrentFilter, leftMotorCurrentSample));
	
//	meanFilteredLeftMotorCurrentMeterValue = (int32_t)round(filteredLeftMotorCurrentMeterValue);
	
//	meanFilteredLeftMotorCurrentMeterValue = applyMeanCurrentValuesFilter((int32_t)round(filteredLeftMotorCurrentMeterValue));
	
//	printf("f1: %d\t\t", (int32_t)round(filteredLeftMotorCurrentMeterValue));
//	printf("uf1: %d\t\tf1: %d\t\tuf2: %d\t\tf2: %d\r\n", (int32_t)round(unfilteredCurrentMeterValue1), (int32_t)round(filteredCurrentMeterValue1), (int32_t)round(unfilteredCurrentMeterValue2), (int32_t)round(filteredCurrentMeterValue2));
}

static void updateACS712RightMotorCurrent(void)
{
	static biquadFilter_t motorCurrentFilter;
	static bool isMotorCurrentFilterInitialised;
	
	if (!isMotorCurrentFilterInitialised) {
		biquadFilterInitLPF(&motorCurrentFilter, MOTOR_CURRENT_LPF_CUTOFF_FREQ, MOTOR_CURRENT_FILTER_SAMPLING_FREQ);
		isMotorCurrentFilterInitialised = true;
	}
	
	/* Get ADC value */
	uint16_t rightMotorCurrentSample = adcGetChannelSample(ADC_MOTOR_CURRENT2);
	
//	printf("rightMotorCurrentSample: %u\r\n", rightMotorCurrentSample);
	
	/* Convert ADC value to amps */
	/* Apply biquad 2nd-order low pass filter */
	rightMotorCurrentSample = biquadFilterApply(&motorCurrentFilter, rightMotorCurrentSample);

	g_rightMotorMilliVoltsScaled3V3 = (rightMotorCurrentSample / AdcConfig()->resolutionScale) * ADC_VREF;			// ADC_VREF = 3300 mV
	g_rightMotorMilliVoltsScaled5V = g_rightMotorMilliVoltsScaled3V3 * ((voltageDivider_R2 + voltageDivider_R1) / voltageDivider_R2);
	
//	printf("milliVolts: %.4f\t", g_milliVolts);
//	printf("milliVoltsOrig: %.4f\t", g_milliVoltsOrigin);
//	
	filteredRightMotorCurrentMeterValue = (g_rightMotorMilliVoltsScaled5V - MotorCurrentMeterConfig()->rightMotorCurrentMeterOffset) / MotorCurrentMeterConfig()->currentMeterScale;
	
	/* Using 5V voltage regulator, there is an 1 Amps offset coming out of the calculated current output value */
	filteredRightMotorCurrentMeterValue = filteredRightMotorCurrentMeterValue - 1.0f;

//	sprintf(dataLog, "%.4f,%.4f,%.4f,%.4f", (float)rightMotorCurrentSample, g_rightMotorMilliVoltsScaled3V3, g_rightMotorMilliVoltsScaled5V, filteredRightMotorCurrentMeterValue);
//	printf("%s\r\n", dataLog);

//	unfilteredCurrentMeterValue2 = convertADCCountToMilliAmps(rightMotorCurrentSample);
//	filteredRightMotorCurrentMeterValue = convertADCCountToMilliAmps(biquadFilterApply(&motorCurrentFilter, rightMotorCurrentSample));
	
//	meanFilteredRightMotorCurrentMeterValue = (int32_t)round(filteredRightMotorCurrentMeterValue);
	
//	meanFilteredRightMotorCurrentMeterValue = applyMeanCurrentValuesFilter((int32_t)round(filteredRightMotorCurrentMeterValue));
	
//	printf("f2: %d\r\n", (int32_t)round(filteredRightMotorCurrentMeterValue));
}

static void updateACS712CurrentDrawn(int32_t lastUpdateAt)
{
	
}

void updateACS712LeftMotorCurrentSensor(int32_t lastUpdateAt)
{
	updateACS712LeftMotorCurrent();
	
	updateACS712CurrentDrawn(lastUpdateAt);
}

void updateACS712RightMotorCurrentSensor(int32_t lastUpdateAt)
{
	updateACS712RightMotorCurrent();
	
	updateACS712CurrentDrawn(lastUpdateAt);
}
