/**
 * @file analog.h
 * @brief High-level interface for ADS111x / ADS1015 analog-to-digital converters.
 *
 * This module provides a clean abstraction for using ADS111x-compatible
 * ADCs (including ADS1015 and ADS1115) on ESP-IDF systems. It handles
 * I²C initialization, device configuration, channel selection, raw-to-voltage
 * conversion, and structured logging of measured values.
 *
 * The interface is built around the @ref AnalogSensor structure, which stores
 * all configuration parameters, per-channel scaling factors, and the most
 * recent voltage readings.
 *
 * Typical usage flow:
 *  - Fill an @ref AnalogSensor structure with I²C and ADC configuration
 *  - Call @ref analog_init()
 *  - Periodically call @ref analog_read()
 *  - Optionally call @ref analog_log() for human-readable output
 *
 * This design allows multiple ADC instances, clear separation of hardware
 * configuration and measurement logic, and predictable behavior suitable
 * for control and data acquisition tasks.
 *
 * @note This module relies on the esp-idf-lib ADS111x driver and the i2cdev helper
 *       library. Proper I²C pull-up resistors are required on SDA and SCL.
 */

#ifndef ANALOG_H
#define ANALOG_H

#include <ads111x.h>
#include <stdint.h>
#include "esp_err.h"

#define ANALOG_CHANNELS 4

/**
 * @brief Runtime descriptor for an ADS111x / ADS1015 analog sensor.
 *
 * This structure contains all configuration data and runtime state required
 * to operate an ADS111x-family ADC. It encapsulates I²C configuration,
 * gain settings, per-channel scaling factors, and the most recent voltage
 * measurements.
 *
 * The ADC gain is global to the IC, while conversion factors and measurements
 * are stored per-channel to support external voltage dividers or signal conditioning.
 *
 * Members `dev` and `last_voltage` are managed internally by the driver and
 * must not be modified by the user after initialization.
 *
 * Typical lifecycle:
 *  - Populate configuration fields (address, I²C port, pins, gain, scaling)
 *  - Call @ref analog_init() to initialize hardware
 *  - Periodically call @ref analog_read() to update measurements
 *  - Use @ref analog_log() or access `last_voltage[]` directly
 */
typedef struct {
    uint8_t i2c_addr;                 /**< I²C address of the ADC device */
    i2c_port_t i2c_port;              /**< I²C port used by the device */
    i2c_dev_t dev;                    /**< Internal ADS111x device descriptor */
    int sda_pin;                      /**< GPIO used for I²C SDA */
    int scl_pin;                      /**< GPIO used for I²C SCL */
    ads111x_gain_t gain;              /**< Programmable gain (global to the IC) */
    float gain_val;                   /**< Voltage range corresponding to the selected gain */
    float conversion_factor[ANALOG_CHANNELS];
                                       /**< Per-channel scaling factor (e.g. voltage divider) */
    float last_voltage[ANALOG_CHANNELS];
                                       /**< Last measured voltage per channel */
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