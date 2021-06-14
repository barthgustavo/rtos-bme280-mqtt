#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "bme280.h"
#include "i2c.h"
#include "esp_log.h"

#define TAG_SENSOR "SENSOR"

SemaphoreHandle_t xMutex;

uint32_t c1 = 0, c2 = 0, c3 = 0;

void readTemperatureTask(void *arg)
{
    struct bme280_t bme280 = {
		.bus_write = BME280_I2C_bus_write,
		.bus_read = BME280_I2C_bus_read,
		.dev_addr = BME280_I2C_ADDRESS1,
		.delay_msec = BME280_delay_msek
	};

	s32 com_rslt;
	s32 v_uncomp_pressure_s32;
	s32 v_uncomp_temperature_s32;
	s32 v_uncomp_humidity_s32;

	com_rslt = bme280_init(&bme280);

    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
	com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

	com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);

    if (com_rslt == SUCCESS) {
		while(1) {
			vTaskDelay(50 / portTICK_PERIOD_MS);

			com_rslt = bme280_read_uncomp_pressure_temperature_humidity(&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

			if (com_rslt == SUCCESS) {
				ESP_LOGI(TAG_SENSOR, "%.2f degC / %.3f hPa / %.3f %%",
					bme280_compensate_temperature_double(v_uncomp_temperature_s32),
					bme280_compensate_pressure_double(v_uncomp_pressure_s32)/100,
					bme280_compensate_humidity_double(v_uncomp_humidity_s32));
			} else {
				ESP_LOGE(TAG_SENSOR, "measure error. code: %d", com_rslt);
			}
		}
	} else {
		ESP_LOGE(TAG_SENSOR, "init or setting error. code: %d", com_rslt);
	}

    vTaskDelete(NULL);
}

void publishResultsTask(void *arg)
{
    while(1)
    {
        //printf("2\n");
        //xSemaphoreTake(xMutex, portMAX_DELAY);
        c2++;
        //xSemaphoreGive(xMutex);

        vTaskDelay(10 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

void modbusTask(void *arg)
{
    while(1)
    {
        //printf("3\n");
        //xSemaphoreTake(xMutex, portMAX_DELAY);
        c3++;
        //xSemaphoreGive(xMutex);

        vTaskDelay(10 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

void printCounters(void *arg)
{
    
    while(1)
    {
        //xSemaphoreTake(xMutex, portMAX_DELAY);
        printf("Task1: %d, Task2: %d, Task3: %d\n", c1, c2, c3);
        //xSemaphoreGive(xMutex);

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    startI2C();

    xMutex = xSemaphoreCreateMutex();

    /*
        Parâmetros:
        1 - Método da task
        2 - Nome da task para fins de debug
        3 - Tamanho da stack da task
        4 - Ponteiro que a task receberá como parametro
        5 - Prioridade. Pode ser também utilizado portPRIVILEGE_BIT para dar privilegios de sistema à task
        6 - Parâmetro para caso necessita atribuir a referência da task à uma variável
        7 - Atribuição do core da ESP32. Pode ser 0, 1 ou "no-affinity".
    */
    xTaskCreatePinnedToCore(&readTemperatureTask, "read-temperature-task", 2048, NULL, 10, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&publishResultsTask,  "publish-results-task",  1024, NULL, 6, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&modbusTask,          "modbus-task",           1024, NULL, 2, NULL, tskNO_AFFINITY);

    xTaskCreatePinnedToCore(&printCounters,       "print-counters",        2048, NULL, 1, NULL, tskNO_AFFINITY);
}
