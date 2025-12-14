// i2c_scan.c
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <stddef.h>

static const char *TAG = "i2c_scan";

/**
 * Scan I2C on SDA=8, SCL=9 and return addresses found.
 *
 * @param found_addrs   Pointer to a buffer to receive addresses (uint8_t each).
 * @param max_addrs     Capacity of found_addrs buffer.
 * @param found_count   Out: number of addresses found.
 * @return ESP_OK on success (scan completed). If i2c driver can't be installed/used returns error.
 *
 * Notes:
 * - Uses I2C_NUM_0. If your app uses the same bus already, remove driver install/delete
 *   and just run the scanning loop.
 * - External pull-ups (~2.2k-10k) are recommended on SDA/SCL. Internal pullups are enabled here
 *   but are usually too weak for reliable I2C at higher speeds or on long lines.
 */
esp_err_t i2c_scan_gpio8_9(uint8_t *found_addrs, size_t max_addrs, size_t *found_count)
{
    if (!found_addrs || !found_count || max_addrs == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err;
    *found_count = 0;

    // Configure I2C master
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 8,
        .scl_io_num = 9,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 50000, // 100 kHz
    };

    err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    // Install driver (no RX/TX buffers for master)
    err = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    // Small timeout for cmd begin (in ticks). We'll use 10ms
    const int timeout_ms = 10;

    for (int addr = 1; addr < 128; ++addr) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        // Build a write of only the device address (no data). The ACK (or NACK) reveals presence.
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true /*ack_check*/);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(timeout_ms));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            // Device ACKed at this address
            if (*found_count < max_addrs) {
                found_addrs[*found_count] = (uint8_t)addr;
            }
            (*found_count)++;
            ESP_LOGI(TAG, "I2C device found at 0x%02X", addr);
        } else if (ret == ESP_ERR_TIMEOUT) {
            // Timeout communicating; might indicate bus problem — log and continue
            ESP_LOGW(TAG, "I2C timeout at address 0x%02X", addr);
        } else {
            // NACK or other error -> treat as no device, continue scanning
        }
    }

    // Uninstall driver to leave system as found (optional)
    i2c_driver_delete(I2C_NUM_0);

    // If more devices were found than the buffer can hold, return success but let caller know overflow
    if (*found_count > max_addrs) {
        ESP_LOGW(TAG, "Found %u devices but buffer only holds %u — truncating list",
                 (unsigned int)*found_count, (unsigned int)max_addrs);
    }

    return ESP_OK;
}
