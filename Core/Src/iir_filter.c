/**************************************************************
WinFilter version 0.8
http://www.winfilter.20m.com
akundert@hotmail.com

Filter type: Low Pass
Filter model: Butterworth
Filter order: 2
Sampling Frequency: 1000 Hz
Cut Frequency: 100.000000 Hz
Coefficents Quantization: 16-bit

Z domain Zeros
z = -1.000000 + j 0.000000
z = -1.000000 + j 0.000000

Z domain Poles
z = 0.571472 + j -0.293599
z = 0.571472 + j 0.293599
***************************************************************/
#include "iir_filter.h"

int16_t doIirFilter(IIR_FilterData* filter_data, int16_t NewSample)
{
    //shift the old samples
    for(uint8_t n = filter_data->order; n > 0; n--)
    {
    	filter_data->x[n] = filter_data->x[n-1];
    	filter_data->y[n] = filter_data->y[n-1];
    }

    //Calculate the new output
    filter_data->x[0] = NewSample;
    filter_data->y[0] = filter_data->ACoefs[0] * filter_data->x[0];
    for(uint8_t n = 1; n <= filter_data->order; n++)
    	filter_data->y[0] += filter_data->ACoefs[n] * filter_data->x[n] - filter_data->BCoefs[n] * filter_data->y[n];

    filter_data->y[0] /= filter_data->BCoefs[0];

    return filter_data->y[0] / filter_data->DC_gain;
}
