#ifndef VIRTUALINSTRUMENT_HW_H_
#define VIRTUALINSTRUMENT_HW_H_

#ifdef STM32F042x6
#include "hw_stm32f042.h"
#endif

#include <stdint.h>

void    HWInit();
int     HWTryEnableHSE(void);

void    HWClearPeriodMeasurement(void);
void    HWClearPulseCounter(void);
int     HWGetPeriodPulseWidth(unsigned int* period_out, unsigned int* pulse_width_out);
void 	HWSetGeneratorPWM(uint16_t prescaler, uint16_t period, uint16_t pulse_time, int phase);
void    HWStartTimeframe(uint32_t duration);        // rename
int     HWTimeframeElapsed(void);                   // rename


void    utilDelayMs(uint32_t milliseconds);

// temporary
void HWSetFreqMode(int mode, int edge) __attribute__ ((deprecated ("refactor")));

#endif /* VIRTUALINSTRUMENT_HW_H_ */