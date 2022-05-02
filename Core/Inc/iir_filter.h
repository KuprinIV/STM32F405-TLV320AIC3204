#ifndef __IIR_FILTER_H
#define __IIR_FILTER_H

#include <stdint.h>

#define MAX_ORDER 10

typedef struct
{
	uint8_t order;
	int16_t DC_gain;
	int16_t ACoefs[MAX_ORDER+1];
	int16_t BCoefs[MAX_ORDER+1];
	int32_t y[MAX_ORDER+1]; //output samples
	int16_t x[MAX_ORDER+1]; //input samples
}IIR_FilterData;

int16_t doFilter(IIR_FilterData* filter_data, int16_t NewSample);

#endif
