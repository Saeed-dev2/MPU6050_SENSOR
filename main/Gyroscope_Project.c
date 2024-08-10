#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"

/*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SCL_IO           22 
/*!< GPIO number used for I2C master data  */   
#define I2C_MASTER_SDA_IO           21
/*!< I2C master clock frequency */    
#define I2C_MASTER_FREQ_HZ          100000 
/*!< I2C port number for master device */
#define I2C_MASTER_NUM              I2C_NUM_0 
/*!< I2C master doesn't need buffer */
#define I2C_MASTER_TX_BUF_DISABLE   0 
 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0
/*!< Slave address of the MPU6050 sensor */
#define MPU6050_SENSOR_ADDR         0x68 
 /*!< Register address of WHO_AM_I */
#define MPU6050_WHO_AM_I            0x75
/*!< Register address of Power Management 1 */
#define MPU6050_PWR_MGMT_1          0x6B 
/*!< Register address of Accelerometer X-axis high byte */
#define MPU6050_ACCEL_XOUT_H        0x3B 
/*!< Register address of Gyroscope X-axis high byte */
#define MPU6050_GYRO_XOUT_H         0x43 


/*!< Tag for logging information */
static const char *TAG = "MPU6050"; 
/**
 * @brief Initialize the I2C master interface
 *
 * This function configures the I2C master interface with the specified
 * parameters and installs the driver.
 *
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_FAIL: Failure
 */
static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Write a byte of data to a specific register of the MPU6050
 *
 * This function sends a command to the MPU6050 sensor to write a byte
 * of data to the specified register.
 *
 * @param addr I2C address of the MPU6050 sensor
 * @param reg Register address to write the data to
 * @param data Data byte to write to the register
 *
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_FAIL: Failure
 */
static esp_err_t i2c_master_write_slave(uint8_t addr, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Read data from a specific register of the MPU6050
 *
 * This function reads data from a specified register of the MPU6050 sensor
 * and stores the data in the provided buffer.
 *
 * @param addr I2C address of the MPU6050 sensor
 * @param reg Register address to read data from
 * @param data Buffer to store the read data
 * @param len Length of data to read
 *
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_FAIL: Failure
 */
static esp_err_t i2c_master_read_slave(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Initialize the MPU6050 sensor
 *
 * This function initializes the MPU6050 sensor by waking it up and verifying
 * its identity by reading the WHO_AM_I register.
 *
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_FAIL: Failure
 */
static esp_err_t mpu6050_init() {
    uint8_t who_am_i;
    esp_err_t ret;

    // Wake up MPU6050
    ret = i2c_master_write_slave(MPU6050_SENSOR_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return ret;
    }

    // Check WHO_AM_I register
    ret = i2c_master_read_slave(MPU6050_SENSOR_ADDR, MPU6050_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }

    if (who_am_i != 0x68) {
        ESP_LOGE(TAG, "WHO_AM_I register mismatch: 0x%x", who_am_i);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

/**
 * @brief Task to read accelerometer and gyroscope data from MPU6050
 *
 * This task continuously reads data from the accelerometer and gyroscope
 * registers of the MPU6050 sensor and logs the values.
 */
static void read_mpu6050_data() {
    uint8_t data[14];
    int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;

    while (1) {
        i2c_master_read_slave(MPU6050_SENSOR_ADDR, MPU6050_ACCEL_XOUT_H, data, 14);

        accel_x = (data[0] << 8) | data[1];
        accel_y = (data[2] << 8) | data[3];
        accel_z = (data[4] << 8) | data[5];
        gyro_x = (data[8] << 8) | data[9];
        gyro_y = (data[10] << 8) | data[11];
        gyro_z = (data[12] << 8) | data[13];

        ESP_LOGI(TAG, "Accel: (%d, %d, %d), Gyro: (%d, %d, %d)", accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Main application entry point
 *
 * This is the main application function that initializes the I2C master interface,
 * initializes the MPU6050 sensor, and creates a FreeRTOS task to read sensor data.
 */
void app_main() {
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(mpu6050_init());

    xTaskCreate(read_mpu6050_data, "read_mpu6050_data", 2048, NULL, 5, NULL);
}
