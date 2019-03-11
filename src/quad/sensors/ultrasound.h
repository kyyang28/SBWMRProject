#ifndef __ULTRASOUND_H
#define __ULTRASOUND_H

#include "ultrasound_hcsr04.h"
#include "time.h"

#define ULTRASOUND_OUT_OF_RANGE					(-1)

void ultrasoundInit(const ultrasoundConfig_t *ultrasoundConfig);
void ultrasoundUpdate(timeUs_t currentTimeUs);
int32_t ultrasoundRead(void);

#endif	// __ULTRASOUND_H
