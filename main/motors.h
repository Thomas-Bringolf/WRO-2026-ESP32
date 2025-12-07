/**
 * @file motors.h
 * @brief Motor and servo control for ESP32 using LEDC PWM and GPIO direction pins.
 *
 * This library provides high-level interfaces for controlling DC motors with ramping speed,
 * and servo motors with precise angle control. It also supports enabling/disabling motors
 * via a relay.
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <string.h>
#include <stdbool.h>
#include "esp_err.h"


/**
 * @struct Motor
 * @brief Represents a DC motor controlled via PWM and a direction GPIO pin.
 *
 * @var pwm_pin        GPIO pin used for PWM output.
 * @var dir_pin        GPIO pin used for direction control.
 * @var current_speed  Current motor speed, in the range [-1.0 .. 1.0].
 * @var step_size      Increment per step when ramping to a target speed.
 * @var step_delay_ms  Delay in milliseconds per ramp step.
 */
typedef struct {
    int pwm_pin;         /**< GPIO for PWM signal */
    int dir_pin;         /**< GPIO for direction */
    float current_speed; /**< Current speed [-1.0 .. 1.0] */
    float step_size;     /**< Increment per ramp step */
    int step_delay_ms;   /**< Delay per ramp step in milliseconds */
} Motor;

/**
 * @struct Servo
 * @brief Represents a servo motor controlled via PWM.
 *
 * @var pwm_pin     GPIO pin used for PWM output.
 * @var angle_range Maximum servo rotation in degrees (± angle_range).
 * @var center_duty PWM duty cycle corresponding to center position (0°).
 * @var min_duty    PWM duty cycle corresponding to minimum angle (-angle_range).
 * @var max_duty    PWM duty cycle corresponding to maximum angle (+angle_range).
 */
typedef struct {
    int pwm_pin;      /**< GPIO for PWM signal */
    int angle_range;  /**< Maximum angle in degrees */
    int center_duty;  /**< Duty cycle for center position */
    int min_duty;     /**< Duty cycle for minimum angle */
    int max_duty;     /**< Duty cycle for maximum angle */
} Servo;

/**
 * @brief Initialize a DC motor by configuring its PWM and direction GPIO.
 *
 * @param m Pointer to a Motor struct.
 */
esp_err_t motor_init(Motor *m);

/**
 * @brief Initialize a servo motor by configuring its PWM channel.
 *
 * @param s Pointer to a Servo struct.
 */
esp_err_t servo_init(Servo *s);

/**
 * @brief Set a target speed for a DC motor and ramp gradually.
 *
 * The speed is automatically adjusted in steps defined by Motor.step_size
 * with delays defined by Motor.step_delay_ms.
 *
 * @param m Pointer to the Motor struct.
 * @param target_speed Target speed [-1.0 .. 1.0].
 * @param debug If true, prints debug information to console.
 */
void motor_set_speed(Motor *m, float target_speed, bool debug);

/**
 * @brief Stop the motor immediately by setting its speed to zero.
 *
 * @param m Pointer to the Motor struct.
 */
void motor_stop(Motor *m);

/**
 * @brief Set a servo motor to a specific angle.
 *
 * The angle is clamped to the ±angle_range defined in the Servo struct.
 *
 * @param s Pointer to the Servo struct.
 * @param angle_deg Target angle in degrees.
 */
void servo_sets_angle(Servo *s, int angle_deg);

/**
 * @brief Initializes the relay pin.
 *
 * Configuration of the specified GPIO pin to later power the motors.
 *
 * @param PWRrelaiPin GPIO pin controlling the relay.
 */
void relay_init(int relay_pin);

/**
 * @brief Enable all motors via a relay pin.
 *
 * Sets the specified GPIO pin high to power the motors.
 *
 * @param PWRrelaiPin GPIO pin controlling the relay.
 */
void relay_enable(int relay_pin);

/**
 * @brief Disable all motors via a relay pin.
 *
 * Sets the specified GPIO pin low to cut motor power.
 *
 * @param PWRrelaiPin GPIO pin controlling the relay.
 */
void relay_disable(int relay_pin);

#endif // MOTORS_H
