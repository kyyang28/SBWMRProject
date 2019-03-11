#ifndef __ULTRASOUND_H
#define __ULTRASOUND_H

#include "ultrasound_hcsr04.h"
#include "time.h"

#define ULTRASOUND_OUT_OF_RANGE					(-1)

void ultrasoundInit(const ultrasoundConfig_t *ultrasoundConfig);
void ultrasound1Update(timeUs_t currentTimeUs);
void ultrasound2Update(timeUs_t currentTimeUs);
void ultrasound3Update(timeUs_t currentTimeUs);
int32_t ultrasound1Read(void);
int32_t ultrasound2Read(void);
int32_t ultrasound3Read(void);

#endif	// __ULTRASOUND_H
