
#include <stdio.h>
#include "exti.h"
#include "nvic.h"
#include "system.h"
#include "target.h"
#include "ultrasound_hcsr04.h"

#ifdef ULTRASOUND

static IO_t triggerIOPin;
static IO_t echoIOPin;
	
extiCallbackRec_t hcsr04_extiCallbackRec;

static uint32_t lastMeasurementAt;

volatile int32_t measurement = -1;

void hcsr04_extiHandler(extiCallbackRec_t *cb)
{
	static uint32_t start_timing;
	uint32_t stop_timing;
	UNUSED(cb);
	
	if (IORead(echoIOPin) != 0) {
		/* High level signal */
		start_timing = micros();
//		printf("start_timing: %u\r\n", start_timing);
	} else {
		/* Low level signal */
		stop_timing = micros();
//		printf("stop_timing: %u\r\n", stop_timing);
		
		if (stop_timing > start_timing) {
			measurement = stop_timing - start_timing;
//			printf("measurement: %d\r\n", measurement);
		}
	}
}

void hcsr04_init(const ultrasoundConfig_t *ultrasoundConfig, ultrasoundRange_t *ultrasoundRange)
{
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	ultrasoundRange->maxRangeCm = HCSR04_MAX_RANGE_CM;										// HCSR04_MAX_RANGE_CM = 400 cm = 4 m
	ultrasoundRange->detectionConeDeciDegrees = HCSR04_DETECTION_CONE_DECIDEGREES;			// HCSR04_DETECTION_CONE_DECIDEGREES = 300
	ultrasoundRange->detectionConeExtendedDeciDegrees = HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES;	// HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES = 450
	
#if defined(STM32F4)
	/* Enable SYSCFG clock, otherwise the EXTI irq handlers are not called */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif
	
	/* Configure trigger pin */
	triggerIOPin = IOGetByTag(ultrasoundConfig->triggerTag);
	IOInit(triggerIOPin, OWNER_ULTRASOUND_TRIGGER, 0);
	IOConfigGPIO(triggerIOPin, IOCFG_OUT_PP);
	
	/* Configure echo pin */
	echoIOPin = IOGetByTag(ultrasoundConfig->echoTag);
	IOInit(echoIOPin, OWNER_ULTRASOUND_ECHO, 0);
	IOConfigGPIO(echoIOPin, IOCFG_IN_FLOATING);
	
#ifdef USE_EXTI
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	EXTIHandlerInit(&hcsr04_extiCallbackRec, hcsr04_extiHandler);	// hcsr04_extiCallbackRec->fn = hcsr04_extiHandler
	EXTIConfig(echoIOPin, &hcsr04_extiCallbackRec, NVIC_PRIO_ULTRASOUND_EXTI, EXTI_Trigger_Rising_Falling);
	EXTIEnable(echoIOPin, true);
#endif

	/* force first measurement in hcsr04_get_distance() function */
	lastMeasurementAt = millis() - 60;
	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
}

void hcsr04_start_sequence(void)
{
	uint32_t now = millis();
		
	/* Repeated interval of trig signal should be greater than 60 ms to avoid interference between connective measurements */
	if (now < (lastMeasurementAt + 60)) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		return;
	}
	
	lastMeasurementAt = now;
	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	
	IOHi(triggerIOPin);
	delayMicroseconds(15);
	IOLo(triggerIOPin);
}

/* Get the distance which was measured by the last pulse (in centimetres) */
int32_t hcsr04_get_distance(void)
{
	/* First calculation: distance = [measurement (in us) * 340 (m/s) * 100 (cm to m) / 1000000 (s to us)] / 2
	 *
	 * 340 m/s = 340 * 100 / 1000000 = 0.034 cm/microsecond, convert to microsecond/cm gives 1 / 0.034 cm/microseconds = 29.411764705882352941176470588235
	 * 
	 * The amount of microseconds stored in measurement divided by 29.411764705882352941176470588235, i.e. measurement / 29.411764705882352941176470588235
	 * gives the twice the distance, dividing by 2 yields the final desired distance.
	 *
	 * measurement / (1 / 0.034 cm/us) / 2 = measurement / 29.411764705882352941176470588235 / 2 = measurement / (29.411764705882352941176470588235 * 2)
	 * = measurement / 58.82352941176470588235294117647 = measurement / 59
	 */
	int32_t distance = measurement / 59;
//	int32_t distance = (measurement * 340 * 100 / 1000000) / 2;
	
	return distance;
}

#endif
