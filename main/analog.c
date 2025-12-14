#include "analog.h"

#include "i2cdev.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
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

    ////////////////////
    /// Remove befor flight!! Pull-up meeds to be external ~5k Ohm
    ///////////////////
    /// Configure GPIO (pull-ups)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << sensor->scl_pin) |
                        (1ULL << sensor->scl_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);


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
    for (int i = 0; i < 4; i++) {
        sensor->last_voltage[i] = 0.0f;
        if (sensor->conversion_factor[i] == 0.0f)
            sensor->conversion_factor[i] = 1.0f;
    }

    // Debug ESP_LOGI(TAG, "ADS1015 initialized at I2C addr 0x%02X", sensor->i2c_addr);
    return ESP_OK;
}


esp_err_t analog_read_all(AnalogSensor *sensor)
{
    if (!sensor) {
        ESP_LOGE(TAG, "analog_read(): NULL sensor");
        return ESP_ERR_INVALID_ARG;
    }

    // Debug ESP_LOGI(TAG, "Analog read started (continuous mode)");

    // Explicit mux mapping (important – do NOT rely on enum order blindly)
    static const ads111x_mux_t mux_map[4] = {
        ADS111X_MUX_1_GND,      //A0
        ADS111X_MUX_2_GND,      //A1
        ADS111X_MUX_3_GND,      //A2
        ADS111X_MUX_0_GND       //A3
    };

    for (int ch = 0; ch < 4; ch++) {

        // Select channel (starts new conversion automatically)
        esp_err_t err = ads111x_set_input_mux(&sensor->dev, mux_map[ch]);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set MUX channel %d", ch);
            continue;
        }

        // Wait ~2 conversion cycles to flush old sample
        // ADS1015 max rate = 3300 SPS → ~0.3 ms per sample
        vTaskDelay(pdMS_TO_TICKS(1));

        int16_t raw;

        // First read: may be stale (discard)
        err = ads111x_get_value(&sensor->dev, &raw);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Discard read failed on ch %d", ch);
            continue;
        }

        // Second read: valid
        err = ads111x_get_value(&sensor->dev, &raw);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Read failed on ch %d", ch);
            continue;
        }

        // ADS1015 = 12-bit signed value in bits [15:4]
        raw >>= 4;

        float voltage =
            (float)raw * sensor->gain_val / 2048.0f;

        sensor->last_voltage[ch] =
            voltage * sensor->conversion_factor[ch];

        ESP_LOGI(TAG,
                 "CH%d raw=%d voltage=%.4f V",
                 ch, raw, sensor->last_voltage[ch]);
    }

    return ESP_OK;
}


esp_err_t analog_read_ch(AnalogSensor *sensor, int channel)
{
    if (!sensor) {
        ESP_LOGE(TAG, "analog_read(): NULL sensor");
        return ESP_ERR_INVALID_ARG;
    }

    if (channel < 0 || channel > 3) {
        ESP_LOGE(TAG, "analog_read(): Channel out-of-range");
        return ESP_ERR_INVALID_ARG;
    }

    // Mux remapping
    static const ads111x_mux_t mux_map[4] = {
        ADS111X_MUX_1_GND,      //A0
        ADS111X_MUX_2_GND,      //A1
        ADS111X_MUX_3_GND,      //A2
        ADS111X_MUX_0_GND       //A3
    };

    // Select channel (starts new conversion automatically)
    esp_err_t err = ads111x_set_input_mux(&sensor->dev, mux_map[channel]);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set MUX channel %d", channel);
        return ESP_FAIL;
    }

    // Wait ~2 conversion cycles to flush old sample
    // ADS1015 max rate = 3300 SPS → ~0.3 ms per sample
    vTaskDelay(pdMS_TO_TICKS(1));
    int16_t raw;
    // First read: may be stale (discard)
    err = ads111x_get_value(&sensor->dev, &raw);
    err = ads111x_get_value(&sensor->dev, &raw);
    err = ads111x_get_value(&sensor->dev, &raw);
    err = ads111x_get_value(&sensor->dev, &raw);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Discard read failed on ch %d", channel);
        return ESP_FAIL;
    }

    // Second read: valid
    err = ads111x_get_value(&sensor->dev, &raw);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Read failed on ch %d", channel);
        return ESP_FAIL;
    }
    // ADS1015 = 12-bit signed value in bits [15:4]
    raw >>= 4;

    float voltage = (float)raw * sensor->gain_val / 2048.0f;
    sensor->last_voltage[channel] = voltage * sensor->conversion_factor[channel];
    // Debug ESP_LOGI(TAG, "CH%d raw=%d voltage=%.4f V", channel, raw, sensor->last_voltage[channel]);
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

    for (int ch = 0; ch < 4; ch++) {
        ESP_LOGI(TAG, "  \033[36mChannel %d\033[0m: Voltage = \033[1;33m%.4f V\033[0m (factor: %.3f)",
                 ch,
                 sensor->last_voltage[ch],
                 sensor->conversion_factor[ch]);
    }

    ESP_LOGI(TAG, "");
    return ESP_OK;
}