#include "analog.h"

#include "i2cdev.h"
#include "esp_log.h"
#include "esp_err.h"

#include <string.h>


static const char *TAG = "ANALOG_MODULE";


esp_err_t analog_init(AnalogSensor *sensor)
{
    if (!sensor) {
        ESP_LOGE(TAG, "analog_init(); NULL sensor pointer");
        return ESP_ERR_INVALID_ARG;
    }

    // Compute gain value from enum (ADS1015 is 12-bit, max value 2047)
    sensor->gain_val = ads111x_gain_values[sensor->gain];  // make sure ads101x_gain_values exists for 1015

    esp_err_t err;

    // Initialize I2C library
    err = i2cdev_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2cdev_init() failed: %d", err);
        return err;
    }

    // Initialize device descriptor
    memset(&sensor->dev, 0, sizeof(sensor->dev));
    err = ads111x_init_desc(&sensor->dev, sensor->i2c_addr, sensor->i2c_port, sensor->sda_pin, sensor->scl_pin);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ads111x_init_desc() failed: %d", err);
        return err;
    }

    // Configure ADS1015
    err = ads111x_set_mode(&sensor->dev, ADS111X_MODE_CONTINUOUS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ads111x_set_mode() failed: %d", err);
        return err;
    }

    err = ads111x_set_data_rate(&sensor->dev, ADS101X_DATA_RATE_3300);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ads111x_set_data_rate() failed: %d", err);
        return err;
    }

    err = ads111x_set_gain(&sensor->dev, sensor->gain);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ads111x_set_gain() failed: %d", err);
        return err;
    }

    // Initialize last voltages and conversion factors
    for (int i = 0; i < ANALOG_CHANNELS; i++) {
        sensor->last_voltage[i] = 0.0f;
        if (sensor->conversion_factor[i] == 0.0f)
            sensor->conversion_factor[i] = 1.0f;
    }

    ESP_LOGI(TAG, "ADS1015 initialized at I2C addr 0x%02X", sensor->i2c_addr);
    return ESP_OK;
}


esp_err_t analog_read(AnalogSensor *sensor)
{
    if (!sensor) {
        ESP_LOGE(TAG, "analog_read(); NULL sensor pointer");
        return ESP_ERR_INVALID_ARG;
    }
    for (int ch = 0; ch < ANALOG_CHANNELS; ch++)
    {
        // Select the channel
        esp_err_t err = ads111x_set_input_mux(&sensor->dev, (ads111x_mux_t)(ch));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set channel %d", ch);
            continue;
        }

        // Read raw value
        int16_t raw = 0;
        err = ads111x_get_value(&sensor->dev, &raw);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read channel %d", ch);
            continue;
        }

        // Convert to voltage and apply per-channel factor
        float voltage = (float)raw * sensor->gain_val / ADS111X_MAX_VALUE;
        sensor->last_voltage[ch] = voltage * sensor->conversion_factor[ch];
    }

    return ESP_OK;
}


esp_err_t analog_log(const AnalogSensor *sensor)
{
    if (!sensor) {
        ESP_LOGE(TAG, "analog_log(): NULL sensor pointer");
        return ESP_ERR_INVALID_ARG;
    }

    // Force debug logging for this module
    esp_log_level_set("*", ESP_LOG_DEBUG);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "ADS1015 Sensor @ I2C 0x%02X", sensor->i2c_addr);
    ESP_LOGI(TAG, "SDA pin: %d, SCL pin: %d", sensor->sda_pin, sensor->scl_pin);
    ESP_LOGI(TAG, "Gain: %d, Gain voltage per bit: %.6f", sensor->gain, sensor->gain_val);

    for (int ch = 0; ch < ANALOG_CHANNELS; ch++) {
        ESP_LOGI(TAG, "  \033[36mChannel %d\033[0m: Voltage = \033[1;33m%.4f V\033[0m (factor: %.3f)",
                 ch,
                 sensor->last_voltage[ch],
                 sensor->conversion_factor[ch]);
    }

    ESP_LOGI(TAG, "");
    return ESP_OK;
}