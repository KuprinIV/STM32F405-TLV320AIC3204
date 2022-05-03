#ifndef __FIR_FILTER_H
#define __FIR_FILTER_H

#include <stdint.h>

#define NTAP 31

typedef struct
{
	int32_t DC_gain;
	int16_t FirCoefs[NTAP];
	int16_t x[NTAP]; //input samples
}FIR_FilterData;

int16_t doFirFilter(FIR_FilterData* filter_data, int16_t NewSample);

#endif
