#include <string.h>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "sdkconfig.h"

#include "bmp280/bmp280.h"

#define I2C_MASTER_PORT 0

static const char *TAG = "bmp280";

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;
static uint8_t read_buff[32] = { 0 };
static uint8_t write_buff[32] = { 0 };

static void print_hex(void * data, size_t len)
{
	uint8_t * ptr = (uint8_t *) data;

	for (int i = 0; i < len; i++) {
		printf("0x%X ", *ptr);
		ptr++;
	}

	printf("\n");
}

/*!
 *  @brief Prints the execution status of the BMP280 sensor APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void print_rslt(const char api_name[], int8_t rslt)
{
	if (rslt != BMP280_OK)
	{
		ESP_LOGI(TAG, "%s\t", api_name);
		if (rslt == BMP280_E_NULL_PTR)
		{
			ESP_LOGE(TAG, "Error [%d] : Null pointer error", rslt);
        }
        else if (rslt == BMP280_E_COMM_FAIL)
        {
            ESP_LOGE(TAG, "Error [%d] : Bus communication failed", rslt);
        }
        else if (rslt == BMP280_E_IMPLAUS_TEMP)
        {
            ESP_LOGE(TAG, "Error [%d] : Invalid Temperature", rslt);
        }
        else if (rslt == BMP280_E_DEV_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Error [%d] : Device not found", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            ESP_LOGE(TAG, "Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
	ESP_LOGI(TAG, "i2c_master_init");
	i2c_master_bus_config_t i2c_mst_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = I2C_MASTER_PORT,
		.scl_io_num = 22,
		.sda_io_num = 21,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};

	ESP_LOGI(TAG, "i2c_new_master_bus");
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = BMP280_I2C_ADDR_PRIM,
		.scl_speed_hz = 100000,
	};

	ESP_LOGI(TAG, "i2c_master_bus_add_device");
	return i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
}

int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	ESP_LOGI(TAG, "i2c_reg_write:  i2c_addr = 0x%x, reg_addr = 0x%x, length = %d",  i2c_addr, reg_addr, length);
	if (length + 1 > sizeof(write_buff))
		return ESP_FAIL;

	memset(write_buff, 0, sizeof(write_buff));
	write_buff[0] = reg_addr;
	memcpy(&write_buff[1], reg_data, length);
	print_hex(write_buff, length + 1);
	esp_err_t ret = i2c_master_transmit(dev_handle, write_buff, length + 1, -1);
	
	if (ret == ESP_OK) {
		ESP_LOGI(TAG, "Write OK");
	} else if (ret == ESP_ERR_TIMEOUT) {
		ret = 2;
		ESP_LOGW(TAG, "Bus is busy");
	} else {
		ret = 1;
		ESP_LOGW(TAG, "Write Failed");
	}

	return ret;
}

int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	ESP_LOGI(TAG, "i2c_reg_read:  i2c_addr = 0x%x, reg_addr = 0x%x, length = %d",  i2c_addr, reg_addr, length);
	if (length + 1 > sizeof(read_buff))
		return ESP_FAIL;
	
	memset(read_buff, 0, sizeof(read_buff));
	read_buff[0] = reg_addr;
	print_hex(read_buff, length + 1);

	esp_err_t ret = i2c_master_receive(dev_handle, read_buff, length + 1, -1);

	if (ret == ESP_OK) {
		ESP_LOGI(TAG, "Read OK");
		//print_hex(read_buff, length + 1);
		print_hex(read_buff, sizeof(read_buff));
		memcpy(reg_data, &read_buff[1], length);
	} else if (ret == ESP_ERR_TIMEOUT) {
		ret = 2;
		ESP_LOGW(TAG, "Bus is busy");
	} else {
		ret = 1;
		ESP_LOGW(TAG, "Read failed");
	}

	return ret;
}

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as "bmg250_soft_reset",
 *      "bmg250_set_foc", "bmg250_perform_self_test"  and so on.
 *
 *  @param[in] period_ms  : the required wait time in milliseconds.
 *  @return void.
 *
 */
void delay_ms(uint32_t period_ms)
{
    vTaskDelay(period_ms / portTICK_PERIOD_MS);
}

static struct bmp280_dev bmp = {0};

esp_err_t sensor_init(void)
{
	struct bmp280_config conf;
	int8_t ret = 0;
	esp_err_t err = i2c_master_init();

	if (err != ESP_OK) {
		ESP_LOGE(TAG, "sensor_init: i2c_master_init ret = 0x%x", err);
		return err;
	}

	/* Map the delay function pointer with the function responsible for implementing the delay */
	bmp.delay_ms = delay_ms;

	/* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
	bmp.dev_id = BMP280_I2C_ADDR_PRIM;

	/* Select the interface mode as I2C */
	bmp.intf = BMP280_I2C_INTF;

	/* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
	bmp.read = i2c_reg_read;
	bmp.write = i2c_reg_write;

	for (int r = 5; r > 0; r--) {
		ret = bmp280_init(&bmp);
		print_rslt(" bmp280_init status", ret);
		if (ret == 0)
			break;
	}

	if (ret < 0) {
		return ESP_FAIL;
	}

	/* Always read the current settings before writing, especially when
	* all the configuration is not modified
	*/
	ret = bmp280_get_config(&conf, &bmp);
	print_rslt(" bmp280_get_config status", ret);
	if (ret < 0) {
		return ESP_FAIL;
	}

	/* configuring the temperature oversampling, filter coefficient and output data rate */
	/* Overwrite the desired settings */
	conf.filter = BMP280_FILTER_COEFF_2;

	/* Temperature oversampling set at 4x */
	conf.os_temp = BMP280_OS_4X;

	/* Pressure over sampling none (disabling pressure measurement) */
	conf.os_pres = BMP280_OS_NONE;

	/* Setting the output data rate as 1HZ(1000ms) */
	conf.odr = BMP280_ODR_1000_MS;
	ret = bmp280_set_config(&conf, &bmp);
	print_rslt(" bmp280_set_config status", ret);
	if (ret < 0) {
		return ESP_FAIL;
	}

	/* Always set the power mode after setting the configuration */
	ret = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
	print_rslt(" bmp280_set_power_mode status", ret);
	if (ret < 0) {
		return ESP_FAIL;
	}

	return ESP_OK;
}

#include <stdio.h>

esp_err_t sensor_read(void)
{
	struct bmp280_uncomp_data ucomp_data;
	int32_t temp32;
	double temp;
	int8_t rslt = 0;
	/* Reading the raw data from sensor */
	rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
	print_rslt(" bmp280_get_uncomp_data status", rslt);
	

	/* Getting the 32 bit compensated temperature */
	rslt = bmp280_get_comp_temp_32bit(&temp32, ucomp_data.uncomp_temp, &bmp);
	print_rslt(" bmp280_get_comp_temp_32bit status", rslt);

	/* Getting the compensated temperature as floating point value */
	rslt = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);
	print_rslt(" bmp280_get_comp_temp_double status", rslt);
	printf("UT: %ld, T32: %ld, T: %f \r\n", ucomp_data.uncomp_temp, temp32, temp);

	/* Sleep time between measurements = BMP280_ODR_1000_MS */
	bmp.delay_ms(1000);
	return rslt;
}