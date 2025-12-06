#ifndef ANALOG_H
#define ANALOG_H

#include <ads111x.h>
#include <stdint.h>
#include "esp_err.h"

#define ANALOG_CHANNELS 4

typedef struct {
    uint8_t i2c_addr;                 // I2C address of the ADC
    i2c_port_t i2c_port;              // I2C port used
    i2c_dev_t dev;                     // internal descriptor (ads111x)
    int sda_pin;                       // SDA GPIO
    int scl_pin;                       // SCL GPIO
    ads111x_gain_t gain;               // Gain configuration (global for IC)
    float gain_val;                    // Corresponding voltage per bit (global)
    float conversion_factor[ANALOG_CHANNELS]; // Scaling per channel (voltage divider etc.)
    float last_voltage[ANALOG_CHANNELS];     // Last read voltages for each channel
} AnalogSensor;

/**
 * @brief Initialize an ADS111x/ADS1015 sensor and prepare it for readings.
 *
 * This function configures the I²C bus, initializes the device descriptor,
 * sets the conversion mode, data rate, and gain. It also initializes the
 * `last_voltage` array and ensures per-channel conversion factors are valid.
 *
 * Detailed logging is performed using ESP_LOG macros to indicate success
 * or specific errors encountered during initialization.
 *
 * @param sensor Pointer to an AnalogSensor structure. All fields must be
 *               set (I²C port, pins, address, gain, conversion factors)
 *               except for `dev` and `last_voltage`, which are handled
 *               internally.
 *
 * @return
 *      - ESP_OK: Initialization completed successfully.
 *      - ESP_ERR_INVALID_ARG: `sensor` is NULL.
 *      - Other ESP_ERR_* codes: If I²C initialization or device configuration fails.
 *
 * @note The sensor must not be used for readings before successful initialization.
 *       Logging will indicate the exact step where an error occurs.
 */
esp_err_t analog_init(AnalogSensor *sensor);


/**
 * @brief Read all analog channels from the sensor and update `last_voltage`.
 *
 * This function iterates over all channels of the ADC, selects the input
 * multiplexer for each channel, reads the raw ADC value, converts it to
 * voltage using the configured gain and per-channel conversion factor,
 * and stores it in `sensor->last_voltage`.
 *
 * Logging is performed for any read failures, allowing you to trace
 * which channels failed during a measurement.
 *
 * @param sensor Pointer to a properly initialized AnalogSensor structure.
 *
 * @return
 *      - ESP_OK: All channels successfully read and stored.
 *      - ESP_ERR_INVALID_ARG: `sensor` is NULL.
 *      - Other ESP_ERR_* codes may occur if I²C communication fails.
 *
 * @note Ensure that `analog_init()` was called successfully before calling
 *       this function. Conversion factors are applied automatically to each channel.
 */
esp_err_t analog_read(AnalogSensor *sensor);


/**
 * @brief Logs the latest ADC readings of all channels of an AnalogSensor.
 *
 * This function prints the voltage of each channel stored in the
 * `last_voltage` array of the sensor. It applies the per-channel
 * `conversion_factor` and formats the output in a human-readable way.
 * Useful for debugging or monitoring the analog inputs in real-time.
 *
 * @param sensor Pointer to the AnalogSensor structure containing the latest readings.
 *
 * @return
 * - ESP_OK if the readings were successfully logged.
 * - ESP_ERR_INVALID_ARG if the sensor pointer is NULL.
 *
 * @note The sensor must have been initialized using `analog_init()`
 *       and have valid readings in `last_voltage`.
 */
esp_err_t analog_log(const AnalogSensor *sensor);


#endif // ANALOG_H
