#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "bmp280/bmp280.h"

#define I2C_MASTER_PORT			0
#define I2C_MASTER_TX_BUF_DISABLE	0   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE	0   /*!< I2C master doesn't need buffer */
#define ACK_CHECK_EN			0x1 /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS			0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL				0x0 /*!< I2C ack value */
#define NACK_VAL			0x1 /*!< I2C nack value */

static const char *TAG = "i2c_sensor";

//#define ESP_LOGE( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR,   tag, format, ##__VA_ARGS__)
//#define ESP_LOGW( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN,    tag, format, ##__VA_ARGS__)
//#define ESP_LOGI( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,    tag, format, ##__VA_ARGS__)
//#define ESP_LOGD( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,   tag, format, ##__VA_ARGS__)
//#define ESP_LOGV( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, tag, format, ##__VA_ARGS__)

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
	ESP_LOGI(TAG, "i2c_master_init");
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = 21;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = 22;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	i2c_param_config(I2C_MASTER_PORT, &conf);
	return i2c_driver_install(I2C_MASTER_PORT, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	ESP_LOGI(TAG, "i2c_reg_write:  i2c_addr = 0x%x, reg_addr = 0x%x, length = %d",  i2c_addr, reg_addr, length);

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, i2c_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);

	if (length) {
		i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
	}

	for (int i = 0; i < length; i++) {
		i2c_master_write_byte(cmd, reg_data[i], ACK_CHECK_EN);
	}

	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	
	if (ret == ESP_OK) {
		ESP_LOGI(TAG, "Write OK");
	} else if (ret == ESP_ERR_TIMEOUT) {
		ESP_LOGW(TAG, "Bus is busy");
	} else {
		ESP_LOGW(TAG, "Write Failed");
	}

	//i2c_driver_delete(I2C_MASTER_PORT);
	return 0;
}

int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, i2c_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, i2c_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);

	if (length > 1) {
		i2c_master_read(cmd, reg_data, length - 1, ACK_VAL);
	}

	i2c_master_read_byte(cmd, reg_data + length - 1, NACK_VAL);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret == ESP_OK) {
		ESP_LOGI(TAG, "Read OK");
	} else if (ret == ESP_ERR_TIMEOUT) {
		ESP_LOGW(TAG, "Bus is busy");
	} else {
		ESP_LOGW(TAG, "Read failed");
	}

	//i2c_driver_delete(I2C_MASTER_PORT);
	return 0;
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

int check_sensor(void)
{
	esp_err_t err = 0;
	err = i2c_master_init();
	ESP_LOGI(TAG, "init: err = 0x%x\r\n", err);
	uint8_t data[4] = {0, 0, 0, 0};
	if (err == 0) {
		err = i2c_reg_read(0x76, 0xd0, data, 1);
		ESP_LOGI(TAG, "read: err = 0x%x\r\n", err);
		ESP_LOGI(TAG, "read: data[0] = 0x%x\r\n", data[0]);
		ESP_LOGI(TAG, "read: data[1] = 0x%x\r\n", data[1]);
		ESP_LOGI(TAG, "read: data[2] = 0x%x\r\n", data[2]);
		ESP_LOGI(TAG, "read: data[3] = 0x%x\r\n", data[3]);
	}

	return err;
}

static struct bmp280_dev bmp = {0};

esp_err_t sensor_init(void)
{
	struct bmp280_config conf;
	int8_t rslt = 0;
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

	rslt = bmp280_init(&bmp);

	/* Always read the current settings before writing, especially when
	* all the configuration is not modified
	*/
	rslt = bmp280_get_config(&conf, &bmp);

	/* configuring the temperature oversampling, filter coefficient and output data rate */
	/* Overwrite the desired settings */
	conf.filter = BMP280_FILTER_COEFF_2;

	/* Temperature oversampling set at 4x */
	conf.os_temp = BMP280_OS_4X;

	/* Pressure over sampling none (disabling pressure measurement) */
	conf.os_pres = BMP280_OS_NONE;

	/* Setting the output data rate as 1HZ(1000ms) */
	conf.odr = BMP280_ODR_1000_MS;
	rslt = bmp280_set_config(&conf, &bmp);

	/* Always set the power mode after setting the configuration */
	rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
	return rslt;
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

	/* Getting the 32 bit compensated temperature */
	rslt = bmp280_get_comp_temp_32bit(&temp32, ucomp_data.uncomp_temp, &bmp);

	/* Getting the compensated temperature as floating point value */
	rslt = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);
	printf("UT: %d, T32: %d, T: %f \r\n", ucomp_data.uncomp_temp, temp32, temp);

	/* Sleep time between measurements = BMP280_ODR_1000_MS */
	bmp.delay_ms(1000);
	return rslt;
}