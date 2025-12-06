/**
 * @file encoder.h
 * @brief Incremental rotary encoder interface using the ESP32 PCNT unit.
 *
 * This module provides functions to initialize, read, and reset
 * an incremental rotary encoder. The implementation uses the
 * ESP32 pulse counter (PCNT) hardware to count pulses efficiently,
 * supporting 2× counting and accumulation beyond the 16-bit hardware limit.
 *
 * Accumulated counts are stored in an int32_t, allowing long-term counting
 * without losing pulses.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_timer.h"


/**
 * @brief Encoder configuration and state structure.
 *
 * Contains the GPIO pins, pulse counter unit and channel, and any internal
 * state necessary to manage the encoder.
 */
typedef struct {
    int channelA_pin;                 /**< GPIO pin connected to encoder phase A */
    int channelB_pin;                 /**< GPIO pin connected to encoder phase B */
    pcnt_unit_handle_t pcnt_unit;        /**< Pulse counter unit handle */
    pcnt_channel_handle_t pcnt_channel;  /**< Pulse counter channel handle */
    int32_t accumulated_count;        /**< Absolute accumulated count */
    int32_t last_accumulated_count;   /**< Count at last speed calculation */
    int64_t last_speed_calc_time;     /**< Timestamp of last speed calculation (µs) */
} Encoder;


/**
 * @brief Initialize an incremental rotary encoder using the PCNT hardware.
 *
 * Configures the pulse counter unit, sets counting mode (2×), debounce filter,
 * threshold interrupts, and starts the counter.
 *
 * @param encoder Pointer to an Encoder structure with pins and PCNT unit/channel configured.
 *
 * @return
 *      - ESP_OK: Initialization succeeded.
 *      - ESP_ERR_INVALID_ARG: Null pointer or invalid parameters.
 *      - Other ESP_ERR_* codes may indicate PCNT configuration failure.
 *
 * @note The encoder accumulation is stored internally in a static int32_t variable.
 *       Multiple encoders require separate PCNT units.
 */
esp_err_t encoder_init(Encoder *encoder);

/**
 * @brief Reset the encoder counter and accumulated count.
 *
 * Stops the counter, clears both the hardware counter and the software accumulated count,
 * then resumes counting.
 *
 * @param encoder Pointer to an initialized Encoder structure.
 *
 * @return ESP_OK on success.
 */
esp_err_t encoder_reset(Encoder *encoder);

/**
 * @brief Read the total accumulated encoder count.
 *
 * Reads any remaining pulses from the hardware PCNT unit, adds them to
 * the software accumulated count, and returns the total.
 *
 * @param encoder Pointer to an initialized Encoder structure.
 *
 * @return The total number of pulses counted since last reset.
 *
 * @note This function ensures no pulses are lost by clearing the PCNT counter after reading.
 */
int32_t encoder_read(Encoder *encoder);

/**
 * @brief Calculate the motor speed in counts per second (cps) using a cyclic measurement.
 *
 * This function calculates the speed of a rotary encoder by measuring the number of counts
 * that have occurred since the last speed calculation. It waits until a minimum number of
 * counts (10 by default) have accumulated to reduce noise and improve measurement stability.
 *
 * The function uses `esp_timer_get_time()` to measure the elapsed time in microseconds
 * between successive measurements, and computes the speed as:
 * \f$ \text{speed (cps)} = \frac{\Delta \text{counts}}{\Delta t \text{(s)}} \f$
 *
 * The encoder's internal state is updated with the latest accumulated count and timestamp.
 *
 * @param encoder Pointer to an initialized Encoder structure. The structure must have
 *                valid `last_accumulated_count` and `last_speed_calc_time` fields.
 *
 * @return Current speed in counts per second (cps) as a float.
 *
 * @note
 * - The function blocks until at least 10 counts have occurred; the wait is implemented
 *   with `vTaskDelay()` to allow other tasks to run.
 * - For low-speed scenarios, this may introduce latency in speed reporting.
 * - Make sure the encoder counter is correctly initialized and running before calling this function.
 */
float encoder_calc_speed_cyclic(Encoder *encoder);

/**
 * @brief Calculate instantaneous motor speed based on a fixed number of encoder pulses.
 *
 * This function measures the time it takes for the encoder to generate a specified
 * number of pulses (pulse_threshold) and computes the speed in counts per second (CPS).
 * It blocks until the required number of pulses has been counted.
 *
 * @param encoder Pointer to the Encoder object being measured.
 * @param pulse_threshold Number of encoder pulses to wait for before calculating speed.
 *
 * @return Calculated speed in counts per second (CPS) over the measured pulse interval.
 *
 * @note This is a blocking function. For higher-speed motors, ensure that
 *       pulse_threshold is set high enough to reduce measurement noise,
 *       but low enough to minimize latency.
 */
float encoder_calc_speed_now(Encoder *encoder, int pulse_threshold);

/**
 * @brief Calculate instantaneous motor speed using a default pulse threshold.
 *
 * Wrapper around encoder_calc_speed_now() using a predefined default number
 * of pulses (e.g., DEFAULT_PULSE_THRESHOLD).
 *
 * @param encoder Pointer to the Encoder object being measured.
 *
 * @return Calculated speed in counts per second (CPS) over the measured pulse interval.
 *
 * @note This function is convenient for typical measurements where
 *       the default pulse threshold is sufficient.
 */
float encoder_calc_speed_now_(Encoder *encoder);



#endif // ENCODER_H
