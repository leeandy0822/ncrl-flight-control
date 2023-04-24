#include <stdbool.h>
#include "stm32f4xx.h"
#include "adc.h"
#include "vol_cur_sensor.h"

void voltage_read(float* data){
    // Wait for conversions to complete
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
    {
    }
    uint16_t temp = ADC_GetConversionValue(ADC1);
    *data = temp;
}

void current_read(float* data){
    // Wait for conversions to complete
    while (!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC))
    {
    }
    uint16_t temp = ADC_GetConversionValue(ADC2);
    *data = temp;
}


