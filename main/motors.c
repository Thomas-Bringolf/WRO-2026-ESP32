#include "motors.h"

#include <stdio.h>

#include "driver/gpio.h"
#include "soc/gpio_struct.h"   

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#define MOTOR_PWM_FREQ_HZ 20000      // 10 kHz
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_10_BIT  //WARNING! Must be 10-bit, or else polarity flip will not work!
#define MOTOR_LEDC_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_CHANNEL LEDC_CHANNEL_0

#define SERVO_PWM_FREQ_HZ 100      // 100 Hz
#define SERVO_PWM_RESOLUTION LEDC_TIMER_13_BIT
#define SERVO_LEDC_MODE LEDC_LOW_SPEED_MODE
#define SERVO_PWM_CHANNEL LEDC_CHANNEL_1

#define TAG "MOTORS_MODULE"

static void motor_apply_speed(Motor *m, float speed)
{
    // Ensure speed is in [-1, 1] or clamp it
    if(speed > 1.0f) speed = 1.0f;
    if(speed < -1.0f) speed = -1.0f;

    int duty_max = 1023; // 10-bit resolution
    int duty_raw;

    if(speed >= 0) {
        duty_raw = (int)(duty_max * speed);
        gpio_set_level(m->dir_pin, 0);
    } else {
        duty_raw = duty_max - (int)(duty_max * speed);
        gpio_set_level(m->dir_pin, 1);
    }

    // Apply the PWM duty
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_PWM_CHANNEL, duty_raw);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_PWM_CHANNEL);
}


esp_err_t motor_init(Motor *m)
{
    if (!m) {
        ESP_LOGE(TAG, "motor_init(); NULL sensor pointer");
        return ESP_ERR_INVALID_ARG;
    }

    // Configure direction pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << m->dir_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config() failed: %d", err);
        return err;
    }

    // Configure LEDC timer
    ledc_timer_config_t timer_conf = {
        .speed_mode      = MOTOR_LEDC_MODE,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config() failed: %d", err);
        return err;
    }

    // Configure LEDC channel
    ledc_channel_config_t channel_conf = {
        .gpio_num   = m->pwm_pin,
        .speed_mode = MOTOR_LEDC_MODE,
        .channel    = MOTOR_PWM_CHANNEL,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    err = ledc_channel_config(&channel_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config() failed: %d", err);
        return err;
    }

    ESP_LOGI(TAG, "Motor initialized: PWM pin %d, DIR pin %d", m->pwm_pin, m->dir_pin);
    return ESP_OK;
}


esp_err_t servo_init(Servo *s)
{
    if (!s) {
        ESP_LOGE(TAG, "servo_init(); NULL sensor pointer");
        return ESP_ERR_INVALID_ARG;
    }

    // Configure LEDC timer
    ledc_timer_config_t timer_conf = {
        .speed_mode      = SERVO_LEDC_MODE,
        .duty_resolution = SERVO_PWM_RESOLUTION,
        .timer_num       = LEDC_TIMER_1,
        .freq_hz         = SERVO_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config() failed: %d", err);
        return err;
    }

    // Configure LEDC channel
    ledc_channel_config_t channel_conf = {
        .gpio_num   = s->pwm_pin,
        .speed_mode = SERVO_LEDC_MODE,
        .channel    = SERVO_PWM_CHANNEL,
        .timer_sel  = LEDC_TIMER_1,
        .duty       = 0,
        .hpoint     = 0
    };
    err = ledc_channel_config(&channel_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config() failed: %d", err);
        return err;
    }

    ESP_LOGI(TAG, "Servo initialized: PWM pin %d, angle range %d", s->pwm_pin, s->angle_range);
    return ESP_OK;
}


void motor_set_speed(Motor *m, float target_speed, bool debug)
{
    if(target_speed > 1.0f) target_speed = 1.0f;
    if(target_speed < -1.0f) target_speed = -1.0f;

    if(debug) printf("Target speed: %.2f\n", target_speed);

    float speed = m->current_speed;
    while((speed < target_speed - m->step_size) || (speed > target_speed + m->step_size)) {
        if(speed < target_speed) speed += m->step_size;
        else speed -= m->step_size;

        motor_apply_speed(m, speed);
        m->current_speed = speed;
        vTaskDelay(pdMS_TO_TICKS(m->step_delay_ms));
    }

    // final value
    if(debug) printf("Motor FINAL Speed reached\n");
    motor_apply_speed(m, target_speed);
    m->current_speed = target_speed;
}


void motor_stop(Motor *m)
{
    motor_apply_speed(m, 0.0f);
    m->current_speed = 0.0f;
    printf("Motor stopped safely.\n");
}


void servo_sets_angle(Servo *s, int angle_deg)
{
    if(angle_deg > s->angle_range) angle_deg = s->angle_range;
    if(angle_deg < -s->angle_range) angle_deg = -s->angle_range;

    // Map angle to duty cycle
    float duty_span = (s->max_duty - s->min_duty)/2;
    float duty = (s->center_duty) + (angle_deg / s->angle_range) * duty_span;

    ledc_set_duty(SERVO_LEDC_MODE, SERVO_PWM_CHANNEL, duty);
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_PWM_CHANNEL);

    printf("Servo angle set to %d degrees (duty: %f)\n", angle_deg, duty);
}


void motors_enable(int PWRrelaiPin)
{
    // Set relay output pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PWRrelaiPin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(PWRrelaiPin, 1);
    ESP_LOGI(TAG, "Motors enabled via relay on GPIO %d\n", PWRrelaiPin);
}


void motors_disable(int PWRrelaiPin)
{
    gpio_set_level(PWRrelaiPin, 0);
    ESP_LOGI(TAG, "Motors disabled via relay on GPIO %d\n", PWRrelaiPin);
}

