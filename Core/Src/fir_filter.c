/**************************************************************
WinFilter version 0.8
http://www.winfilter.20m.com
akundert@hotmail.com

Filter type: Low Pass
Filter model: Butterworth
Filter order: 4
Sampling Frequency: 1000 Hz
Cut Frequency: 25.000000 Hz
Coefficents Quantization: 16-bit

Z domain Zeros
z = -1.000000 + j 0.000000
z = -1.000000 + j 0.000000
z = -1.000000 + j 0.000000
z = -1.000000 + j 0.000000

Z domain Poles
z = 0.862967 + j -0.052305
z = 0.862967 + j 0.052305
z = 0.931900 + j -0.136363
z = 0.931900 + j 0.136363
***************************************************************/
#include "fir_filter.h"

/**
 * @brief Apply FIR filter
 * @param: filter_data - data structure, that contains filter coefficients and previous input samples
 * @param: NewSample - new input data sample
 * @return: Filtered output data sample
 */
int16_t doFirFilter(FIR_FilterData* filter_data, int16_t NewSample)
{
    int32_t y=0;            //output sample

    //shift the old samples
    for(uint8_t n = NTAP-1; n > 0; n--)
    	filter_data->x[n] = filter_data->x[n-1];

    //Calculate the new output
    filter_data->x[0] = NewSample;
    for(uint8_t n = 0; n < NTAP; n++)
        y += filter_data->FirCoefs[n] * filter_data->x[n];

    return y / filter_data->DC_gain;
}
