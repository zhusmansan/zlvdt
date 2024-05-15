/*
 * SPDX-FileCopyrightText: 2016-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// FreeModbus Slave Example ESP32

#include <stdio.h>
#include <stdint.h>
#include "esp_err.h"
#include "mbcontroller.h"  // for mbcontroller defines and api
#include "modbus_params.h" // for modbus parameters structures
#include "esp_log.h"       // for log_write
#include "sdkconfig.h"

#define MB_PORT_NUM (CONFIG_MB_UART_PORT_NUM)   // Number of UART port used for Modbus connection
#define MB_SLAVE_ADDR (CONFIG_MB_SLAVE_ADDR)    // The address of device in Modbus network
#define MB_DEV_SPEED (CONFIG_MB_UART_BAUD_RATE) // The communication speed of the UART

#define MB_REG_HOLDING_START_AREA0 (HOLD_OFFSET(holding_data0))

#define MB_PAR_INFO_GET_TOUT (10) // Timeout for get parameter info
#define MB_READ_MASK (MB_EVENT_HOLDING_REG_RD)
#define MB_WRITE_MASK (MB_EVENT_HOLDING_REG_WR)
#define MB_READ_WRITE_MASK (MB_READ_MASK | MB_WRITE_MASK)

static const char *TAG = "MODBUS_SLAVE";

static portMUX_TYPE param_lock = portMUX_INITIALIZER_UNLOCKED;

void modbusTask(void *tParam)
{
    mb_param_info_t reg_info;               // keeps the Modbus registers access information
    mb_communication_info_t comm_info;      // Modbus communication parameters
    mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    void *mbc_slave_handler = NULL;

    ESP_ERROR_CHECK(mbc_slave_init(MB_PORT_SERIAL_SLAVE, &mbc_slave_handler)); // Initialization of Modbus controller

// Setup communication parameters and start stack
#if CONFIG_MB_COMM_MODE_ASCII
    comm_info.mode = MB_MODE_ASCII,
#elif CONFIG_MB_COMM_MODE_RTU
    comm_info.mode = MB_MODE_RTU,
#endif

    comm_info.slave_addr = MB_SLAVE_ADDR;
    comm_info.port = MB_PORT_NUM;
    comm_info.baudrate = MB_DEV_SPEED;
    comm_info.parity = MB_PARITY_NONE;
    ESP_ERROR_CHECK(mbc_slave_setup((void *)&comm_info));

    reg_area.type = MB_PARAM_HOLDING;                                              // Set type of register area
    reg_area.start_offset = 0;                                                     // Offset of register area in Modbus protocol
    reg_area.address = (void *)&holding_reg_params.hd_0_filtered_and_scaled_value; // Set pointer to storage instance
    reg_area.size = sizeof(holding_reg_params_t);
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    // Starts of modbus controller and stack
    ESP_ERROR_CHECK(mbc_slave_start());

    // Set UART pin numbers
    ESP_ERROR_CHECK(uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD,
                                 CONFIG_MB_UART_RXD, CONFIG_MB_UART_RTS,
                                 UART_PIN_NO_CHANGE));

    // Set UART driver mode to Half Duplex
    ESP_ERROR_CHECK(uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX));

    ESP_LOGI(TAG, "Modbus slave stack initialized.");
    ESP_LOGI(TAG, "Start modbus test...");

    while (true)
    {
        // Check for read/write events of Modbus master for certain events
        (void)mbc_slave_check_event(MB_READ_WRITE_MASK);
        ESP_ERROR_CHECK_WITHOUT_ABORT(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
        const char *rw_str = (reg_info.type & MB_READ_MASK) ? "READ" : "WRITE";

        // Filter events and process them accordingly
        if (reg_info.type & (MB_EVENT_HOLDING_REG_WR | MB_EVENT_HOLDING_REG_RD))
        {
            // Get parameter information from parameter queue
            ESP_LOGI(TAG, "HOLDING %s (%" PRIu32 " us), ADDR:%u, TYPE:%u, INST_ADDR:0x%" PRIx32 ", SIZE:%u",
                     rw_str,
                     reg_info.time_stamp,
                     (unsigned)reg_info.mb_offset,
                     (unsigned)reg_info.type,
                     (uint32_t)reg_info.address,
                     (unsigned)reg_info.size);
            if (reg_info.address == (uint8_t *)&holding_reg_params.hd_0_filtered_and_scaled_value)
            {
                portENTER_CRITICAL(&param_lock);
                portEXIT_CRITICAL(&param_lock);
            }
        }
    }
 
    // Destroy of Modbus controller on alarm
    ESP_LOGI(TAG, "Modbus controller destroyed.");
    vTaskDelay(100);
    ESP_ERROR_CHECK(mbc_slave_destroy());
}