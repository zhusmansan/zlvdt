#ifndef _DEVICE_PARAMS
#define _DEVICE_PARAMS


#include <stdint.h>

#pragma pack(push, 1)
typedef struct
{
    float hd_0_filtered_and_scaled_value;
    float hd_1_filtered_value;
    float hd_2_adc_value;
    float hd_3_scale_factor;
} holding_reg_params_t;
#pragma pack(pop)

extern holding_reg_params_t holding_reg_params;


void modbusTask(void *tParam);

#endif // !defined(_DEVICE_PARAMS)