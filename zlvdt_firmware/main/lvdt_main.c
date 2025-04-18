/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "driver/dac_cosine.h"
#include "xtensa/core-macros.h"
#include "modbus_params.h"
#include "soc/soc_caps.h"
#include <stdint.h>
#include "esp_err.h"

#define GEN_FREQ (TIMER_CLK_FREQ >> 12)

#define ADC_OVERSAMPLE_FACTOR (SOC_ADC_SAMPLE_FREQ_THRES_HIGH/(GEN_FREQ * 2))

#define Z_ADC_SAMPLERATE GEN_FREQ *ADC_OVERSAMPLE_FACTOR * 2

#define Z_ADC_UNIT ADC_UNIT_2
#define _EXAMPLE_ADC_UNIT_STR(unit) #unit
#define EXAMPLE_ADC_UNIT_STR(unit) _EXAMPLE_ADC_UNIT_STR(unit)
#define Z_ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_2
#define Z_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define Z_ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define Z_ADC_GET_DATA(p_data) ((p_data)->type1.data)
#define Z_ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

#define Z_ADC_CHAN_REF ADC_CHANNEL_4
#define Z_ADC_CHAN_FB ADC_CHANNEL_5

adc_continuous_config_t dig_cfg = {
    .sample_freq_hz = Z_ADC_SAMPLERATE,
    .conv_mode = Z_ADC_CONV_MODE,
    .pattern_num = 2,
    .format = Z_ADC_OUTPUT_TYPE,
    .adc_pattern = (adc_digi_pattern_config_t[]){{
                                                     .atten = ADC_ATTEN_DB_0,
                                                     .channel = Z_ADC_CHAN_REF,
                                                     .unit = Z_ADC_UNIT,
                                                     .bit_width = Z_ADC_BIT_WIDTH,
                                                 },
                                                 {
                                                     .atten = ADC_ATTEN_DB_0,
                                                     .channel = Z_ADC_CHAN_FB,
                                                     .unit = Z_ADC_UNIT,
                                                     .bit_width = Z_ADC_BIT_WIDTH,
                                                 }},
};

#define N_WAVEFORMS_AVERAGING 16

#define LPF_FACTOR 1.0 / 100
#define LPF_FACTOR_10 1.0 / 50
#define REF_LPF_FACTOR 1.0 / 400

#define CAL_COEF (1/23.7)

#define Z_READ_LEN ADC_OVERSAMPLE_FACTOR * 40 /*generous value for read buffer*/

static TaskHandle_t s_task_handle;

holding_reg_params_t holding_reg_params = {
    .hd_0_filtered_and_scaled_value = 0,
    .hd_1_filtered_value = 0,
    .hd_2_adc_value = 0,
    .hd_3_scale_factor = CAL_COEF,
};


static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 4096,
        .conv_frame_size = Z_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

void configureAndStartDac()
{
    dac_cosine_handle_t chan0_handle;
    /* Normally two channels can only be configured to one frequency
     * But we can set force_set_freq bit to force update the frequency
     * The example here will produce cosine wave at 8 KHz on both channels */
    dac_cosine_config_t cos0_cfg = {
        .chan_id = DAC_CHAN_0,
        .freq_hz = GEN_FREQ,
        .clk_src = DAC_COSINE_CLK_SRC_DEFAULT,
        .offset = 0,
        .phase = DAC_COSINE_PHASE_0,
        .atten = DAC_COSINE_ATTEN_DEFAULT,
        .flags.force_set_freq = false,
    };
    ESP_ERROR_CHECK(dac_cosine_new_channel(&cos0_cfg, &chan0_handle));

    ESP_ERROR_CHECK(dac_cosine_start(chan0_handle));
}

void adcContinousReadTask(void *tParam)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[Z_READ_LEN] = {0};
    memset(result, 0xcc, Z_READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(&handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    double refMax = 0;
    double fbMax = 0;
    double refSum = 0;
    double fbSum = 0;

    long refMaxPos = 0;
    long fbMaxPos = 0;

    double refMaxLPF = 0;
    
    double fbMaxLPF = 0;
    double fbMaxLPF_2 = 0;
    double fbMaxLPF_3 = 0;

    double positionLPFVal = 0;
    double positionLPFVal_2 = 0;
    double positionLPFVal_3 = 0;

    double phase = 1;

    uint32_t nval = 0;
    uint32_t waveform = 0;
    uint32_t j = 0;
    bool edge = false;

    while (1)
    {
        ret = adc_continuous_read(handle, result, Z_READ_LEN, &ret_num, 0);

        if (ret == ESP_OK)
        {

            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
            {
                
                adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
                uint32_t chan_num = Z_ADC_GET_CHANNEL(p);
                uint32_t data = Z_ADC_GET_DATA(p);

                if (chan_num == Z_ADC_CHAN_REF)
                {
                    j++;
                    // refSum += data;
                    if (refMax < data)
                    {
                        refMax = data;
                        refMaxPos = nval;
                    }
                    printf(">REF");
                }
                if (chan_num == Z_ADC_CHAN_FB)
                {
                    fbSum += data;
                    // printf(">fbData:%ld:%ld\n",j , data);
                    if (fbMax < data)
                    {
                        fbMax = data;
                        fbMaxPos = nval;
                    }
                    nval += 1;
                    printf(">FB");
                }
                printf(":%ld:%ld\n",j , data);
                

                if (!edge && nval != 0 && (nval % (ADC_OVERSAMPLE_FACTOR*6/4)) == 0)
                {
                    // if ((abs(fbMaxPos - refMaxPos) % ADC_OVERSAMPLE_FACTOR) <= ADC_OVERSAMPLE_FACTOR / 4 || (abs(fbMaxPos - refMaxPos) % ADC_OVERSAMPLE_FACTOR) > ADC_OVERSAMPLE_FACTOR / 4 * 3)
                    // {
                    //     phase = 1;
                    // }
                    // else
                    // {
                    //     phase = -1;
                    // }

                    // refMaxLPF += (refMax - refMaxLPF) * LPF_FACTOR;
                    fbMaxLPF += ( fbMax - fbMaxLPF) * LPF_FACTOR_10;
                    fbMaxLPF_2 += (fbMaxLPF - fbMaxLPF_2) * LPF_FACTOR_10;
                    fbMaxLPF_3 += (fbMaxLPF_2 - fbMaxLPF_3) * LPF_FACTOR_10;
                    // refMaxLPF = refMaxLPF == 0 ? 1 : refMaxLPF;
                    waveform++;
                    edge = true;
                    // positionLPFVal += ((fbMax) / refMax - positionLPFVal) * LPF_FACTOR;
                    // fbSum+=fbMax;
                    refSum += refMax;
                    refMax = 0;
                    fbMax = 0;

                }
                if ((waveform % 2 == 0) && edge)
                {
                    refSum = refSum == 0 ? 1 : refSum;

                    positionLPFVal += ((fbMax) - positionLPFVal) * (LPF_FACTOR );
                    positionLPFVal_2 += ((positionLPFVal) - positionLPFVal_2) * (LPF_FACTOR );
                    positionLPFVal_3 += ((positionLPFVal_2) - positionLPFVal_3) * (LPF_FACTOR );

                    // positionLPFVal += (((double)fbMax) / refMax - positionLPFVal) * LPF_FACTOR;
                    holding_reg_params.hd_0_filtered_and_scaled_value = CAL_COEF * positionLPFVal;
                    holding_reg_params.hd_1_filtered_value = positionLPFVal;
                    holding_reg_params.hd_2_adc_value = fbMax;

                    // printf(">positionLPFVal:%ld:%lf\n", j, positionLPFVal * CAL_COEF);
                    // printf(">positionLPFVal_2:%ld:%lf\n", j, positionLPFVal_2 * CAL_COEF);
                    // printf(">positionLPFVal_3:%ld:%lf\n", j, positionLPFVal_3 * CAL_COEF);

                    printf(">fbMaxLPF:%ld:%lf\n", j, fbMaxLPF * CAL_COEF);
                    printf(">fbMaxLPF_2:%ld:%lf\n", j, fbMaxLPF_2 * CAL_COEF);
                    printf(">fbMaxLPF_3:%ld:%lf\n", j, fbMaxLPF_3 * CAL_COEF);
                    // printf(">refMaxLPF:%ld:%lf\n", j, refMaxLPF);
                    // printf(">fbSum:%ld:%lf\n", j, ((double)fbSum)/N_WAVEFORMS_AVERAGING);
                    // printf(">refSum:%ld:%lf\n", j, refSum/N_WAVEFORMS_AVERAGING);
                    // printf(">refMax:%ld:%lf\n",j , refMax);
                    // printf(">fbMax:%ld:%lf\n",j , fbMax);
                    // printf(">phasediff:%d\n", (abs(fbMaxPos - refMaxPos + ADC_OVERSAMPLE_FACTOR) % ADC_OVERSAMPLE_FACTOR));
                    // printf(">refMaxPos:%ld\n", refMaxPos% ADC_OVERSAMPLE_FACTOR);
                    // printf(">fbMaxPos:%ld\n", fbMaxPos% ADC_OVERSAMPLE_FACTOR);

                    nval = 0;
                    
                    // waveform = 0;
                }
                edge = false;
            }
        }
        else if (ret == ESP_ERR_TIMEOUT)
        {
            // We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
            // break;
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}

void app_main(void)
{
    configureAndStartDac();
    xTaskCreate(adcContinousReadTask, /* Указатель на функцию, которая реализует задачу. */
                "ADC task",           /* Текстовое имя задачи. Этот параметр нужен только для
                                       упрощения отладки. */
                4096,                 /* Глубина стека */
                NULL,                 /* Мы не используем параметр задачи. */
                1,                    /* Задача будет запущена с приоритетом 1. */
                NULL);
    xTaskCreate(modbusTask,    /* Указатель на функцию, которая реализует задачу. */
                "MODBUS task", /* Текстовое имя задачи. Этот параметр нужен только для
                             упрощения отладки. */
                4096,          /* Глубина стека */
                NULL,          /* Мы не используем параметр задачи. */
                1,             /* Задача будет запущена с приоритетом 1. */
                NULL);
}