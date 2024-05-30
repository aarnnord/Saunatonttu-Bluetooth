/**
 * This file is part of RuuviTagRec.
 *
 * RuuviTagRec is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RuuviTagRec is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RuuviTagRec.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Copyright 2020 Pascal Bodin
 */
#include <stdint.h>

#include "esp_log.h"

#include "app.h"
#include "ruuvi_tag.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>

#define BUF_SIZE (1024)


static void echo_task(uint16_t temperature)
{

    uart_port_t uart_num = UART_NUM_1; // Use UART1
    int tx_io_num = GPIO_NUM_17;       // Example TX pin
    int rx_io_num = GPIO_NUM_16;       // Example RX pin

    //const int uart_num = UART_NUM_1;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,    
        .rx_flow_ctrl_thresh = 122,
    };
    //Configure UART1 parameters
    uart_param_config(uart_num, &uart_config);
    //Set UART1 pins(TX: IO4, RX: I05)
    uart_set_pin(uart_num, tx_io_num, rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    //Install UART driver (we don't need an event queue here)
    //In this example we don't even use a buffer for sending data.
    uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);
    while(1) {
        //Write data back to UART
        uart_write_bytes(uart_num, (const float*) temperature, sizeof(temperature));
    }
}


void check_ruuvi_tag_data(uint8_t *data_ptr, uint8_t manu_data_length, uint8_t data_length) {

	if (data_length < manu_data_length) {
		ESP_LOGI(APP_TAG, "Ruuvi: data too short - %d - %d", manu_data_length, data_length);
		return;
	}

	// At this stage, we have another data format.
	if (*data_ptr == RUUVI_RAWV2) {
		// Check length.
		if (data_length < RUUVI_RAWV2_LENGTH) {
			ESP_LOGI(APP_TAG, "Ruuvi: RAWv2 data too short");
			return;
		}
		// Get temperature.
		data_ptr++;
		float temperature = (float)((int16_t)(*data_ptr << 8) + *(data_ptr + 1)) * 0.005;
		// Get humidity.
		data_ptr += 2;
		float humidity = (float)((uint16_t)(*data_ptr << 8) + *(data_ptr + 1)) * 0.0025;
		// Get pressure.
		data_ptr += 2;
		float pressure = (float)(((uint16_t)(*data_ptr << 8) + *(data_ptr + 1)) + 50000) / 100.0;
	
		ESP_LOGI(APP_TAG, "Ruuvi RAWv2: humidity: %.2f - temperature: %.2f - pressure: %.2f", humidity,
				temperature, pressure);
        echo_task(temperature);

		return;
	}
}
