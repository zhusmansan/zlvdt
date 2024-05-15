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

#include <stdint.h>
#include "esp_err.h"

#define GEN_FREQ 3000

#define ADC_OVERSAMPLE_FACTOR 26

#define Z_ADC_SAMPLERATE GEN_FREQ *ADC_OVERSAMPLE_FACTOR

#define Z_ADC_UNIT ADC_UNIT_2
#define _EXAMPLE_ADC_UNIT_STR(unit) #unit
#define EXAMPLE_ADC_UNIT_STR(unit) _EXAMPLE_ADC_UNIT_STR(unit)
#define Z_ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_2
#define Z_ADC_ATTEN ADC_ATTEN_DB_12
#define Z_ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

#define Z_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define Z_ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define Z_ADC_GET_DATA(p_data) ((p_data)->type1.data)

#define N_WAVEFORMS_AVERAGING 15

#define LPF_FACTOR 1.0 / 199
#define REF_LPF_FACTOR 1.0 / 400

#define CAL_COEF 100.326059

#define Z_READ_LEN ADC_OVERSAMPLE_FACTOR * 40 /*generous value for read buffer*/

static adc_channel_t channel[2] = {ADC_CHANNEL_4, ADC_CHANNEL_5};

static TaskHandle_t s_task_handle;

holding_reg_params_t holding_reg_params = {.hd_0_filtered_and_scaled_value = 0,
    .hd_1_filtered_value = 0,
    .hd_2_adc_value = 0,
    .hd_3_scale_factor = CAL_COEF,
};


static const char *TAG = "ADC";

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 2048,
        .conv_frame_size = Z_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = Z_ADC_SAMPLERATE,
        .conv_mode = Z_ADC_CONV_MODE,
        .format = Z_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++)
    {
        adc_pattern[i].atten = Z_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = Z_ADC_UNIT;
        adc_pattern[i].bit_width = Z_ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
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
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    double refMax = 0;
    double fbMax = 0;

    long refMaxPos = 0;
    long fbMaxPos = 0;

    double refMaxLPF = 0;
    double fbMaxLPF = 0;

    double positionLPFVal = 0;

    double phase = 1;

    uint32_t nval = 0;

    char unit[] = EXAMPLE_ADC_UNIT_STR(Z_ADC_UNIT);

    while (1)
    {

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
                    /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */

                    if (chan_num < SOC_ADC_CHANNEL_NUM(Z_ADC_UNIT))
                    {
                        if (chan_num == ADC_CHANNEL_4)
                        {

                            if (refMax < data)
                            {
                                refMax = data;
                                refMaxPos = nval;
                            }
                        }
                        if (chan_num == ADC_CHANNEL_5)
                        {
                            if (fbMax < data)
                            {
                                fbMax = data;
                                fbMaxPos = nval;
                            }
                        }
                        nval += 1;
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Invalid data [%s_%" PRIu32 "_%" PRIx32 "]", unit, chan_num, data);
                    }

                    if (nval % (ADC_OVERSAMPLE_FACTOR) == 0)
                    {
                        if ((abs(fbMaxPos - refMaxPos) % ADC_OVERSAMPLE_FACTOR) <= ADC_OVERSAMPLE_FACTOR / 4 || (abs(fbMaxPos - refMaxPos) % ADC_OVERSAMPLE_FACTOR) > ADC_OVERSAMPLE_FACTOR / 4 * 3)
                        {
                            phase = 1;
                        }
                        else
                        {
                            phase = -1;
                        }

                        refMaxLPF += (refMax - refMaxLPF) * REF_LPF_FACTOR;
                        fbMaxLPF += (phase * fbMax - fbMaxLPF) * LPF_FACTOR;
                    }
                    if (nval % (ADC_OVERSAMPLE_FACTOR * N_WAVEFORMS_AVERAGING) == 0)
                    {

                        refMaxLPF = refMaxLPF == 0 ? 1 : refMaxLPF;
                        positionLPFVal = ((double)fbMaxLPF) / refMaxLPF;
                        holding_reg_params.hd_0_filtered_and_scaled_value = CAL_COEF * positionLPFVal;
                        holding_reg_params.hd_1_filtered_value = positionLPFVal;
                        holding_reg_params.hd_2_adc_value = fbMax;


                        // printf(">positionLPFVal:%lf\n", CAL_COEF * positionLPFVal);
                        // printf(">fbMaxLPF:%lf\n", fbMaxLPF);
                        // printf(">refMaxLPF:%lf\n", refMaxLPF);
                        // printf(">refMaxLPF:%lf\n", refMaxLPF);
                        // printf(">fbMaxLPF:%lf\n", fbMaxLPF);

                        refMax = 0;
                        fbMax = 0;

                        fbMaxPos = 0;

                        nval = 0;
                    }
                }
            }
            else if (ret == ESP_ERR_TIMEOUT)
            {
                // We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                // break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}

void app_main(void)
{
    configureAndStartDac();
    xTaskCreate(adcContinousReadTask,   /* Указатель на функцию, которая реализует задачу. */
                "ADC task", /* Текстовое имя задачи. Этот параметр нужен только для
                             упрощения отладки. */
                4096,     /* Глубина стека */
                NULL,     /* Мы не используем параметр задачи. */
                1,        /* Задача будет запущена с приоритетом 1. */
                NULL);
   xTaskCreate(modbusTask,   /* Указатель на функцию, которая реализует задачу. */
                "MODBUS task", /* Текстовое имя задачи. Этот параметр нужен только для
                             упрощения отладки. */
                4096,     /* Глубина стека */
                NULL,     /* Мы не используем параметр задачи. */
                1,        /* Задача будет запущена с приоритетом 1. */
                NULL);

}