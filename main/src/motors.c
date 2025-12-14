#include "motors.h"

#include "freertos/FreeRTOS.h"
#include "soc/gpio_struct.h"   
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include <stdio.h>

#define MOTOR_PWM_FREQ_HZ 20000      // 10 kHz
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_10_BIT  //WARNING! Must be 10-bit, or else polarity flip will not work!
#define MOTOR_LEDC_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_CHANNEL LEDC_CHANNEL_0

#define SERVO_PWM_FREQ_HZ 100      // 100 Hz
#define SERVO_PWM_RESOLUTION LEDC_TIMER_13_BIT
#define SERVO_LEDC_MODE LEDC_LOW_SPEED_MODE
#define SERVO_PWM_CHANNEL LEDC_CHANNEL_1

#define DEBUG_MODE true
#define TAG "MOTORS_MODULE"

static void motor_apply_speed(Motor *m, float speed) {
    // Ensure speed is in [-1, 1] or clamp it
    if(speed > 1.0f) speed = 1.0f;
    if(speed < -1.0f) speed = -1.0f;

    int duty_max = 1023; // 10-bit resolution
    int duty_raw;

    if(speed >= 0) {
        duty_raw = (int)(duty_max * speed);
        gpio_set_level(m->dir_pin, 0);
    } else {
        duty_raw = duty_max + (int)(duty_max * speed);
        gpio_set_level(m->dir_pin, 1);
    }

    // Apply the PWM duty
    if (DEBUG_MODE) ESP_LOGI(TAG, "motor_apply_speed(); Servo duty set to %d", duty_raw);
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_PWM_CHANNEL, duty_raw);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_PWM_CHANNEL);
}


esp_err_t motor_init(Motor *m) {
    if (!m) {
        if (DEBUG_MODE) ESP_LOGE(TAG, "motor_init(); NULL sensor pointer");
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
        ESP_LOGE(TAG, "gpio_config(); failed: %d", err);
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
        ESP_LOGE(TAG, "motor_init(); ledc_timer_config() failed: %d", err);
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
        ESP_LOGE(TAG, "motor_init(); ledc_channel_config() failed: %d", err);
        return err;
    }

    ESP_LOGI(TAG, "motor_init(); Motor initialized: PWM pin %d, DIR pin %d", m->pwm_pin, m->dir_pin);
    return ESP_OK;
}


esp_err_t servo_init(Servo *s) {
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
        ESP_LOGE(TAG, "servo_init(); ledc_timer_config() failed: %d", err);
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
        ESP_LOGE(TAG, "servo_init(); ledc_channel_config() failed: %d", err);
        return err;
    }

    if (DEBUG_MODE) ESP_LOGI(TAG, "servo_init(); Servo initialized: PWM pin %d, angle range %d", s->pwm_pin, s->angle_range);
    return ESP_OK;
}


void motor_set_speed(Motor *m, float target_speed) {
    if(target_speed > 1.0f) target_speed = 1.0f;
    if(target_speed < -1.0f) target_speed = -1.0f;

    if(DEBUG_MODE) ESP_LOGI(TAG, "motor_set_speed(); Target speed: %.2f\n", target_speed);

    float speed = m->current_speed;
    while((speed < target_speed - m->step_size) || (speed > target_speed + m->step_size)) {
        if(speed < target_speed) speed += m->step_size;
        else speed -= m->step_size;

        motor_apply_speed(m, speed);
        m->current_speed = speed;
        vTaskDelay(pdMS_TO_TICKS(m->step_delay_ms));
    }

    // final value
    if(DEBUG_MODE) ESP_LOGI(TAG, "motor_set_speed(); Motor FINAL Speed reached\n");
    motor_apply_speed(m, target_speed);
    m->current_speed = target_speed;
}


void motor_stop(Motor *m) {
    motor_apply_speed(m, 0.0f);
    m->current_speed = 0.0f;
    if (DEBUG_MODE) ESP_LOGI(TAG, "Motor stopped safely.\n");
}


esp_err_t servo_sets_angle(Servo *s, float angle_deg) {
    esp_err_t err = ESP_OK;
    if(angle_deg > s->angle_range) angle_deg = s->angle_range;
    if(angle_deg < -s->angle_range) angle_deg = -s->angle_range;
    s->last_angle = angle_deg;

    // Map angle to duty cycle
    float duty_span = (s->max_duty - s->min_duty)/2;
    float duty = (s->center_duty) + (angle_deg / (float)s->angle_range) * duty_span;

    err = err | ledc_set_duty(SERVO_LEDC_MODE, SERVO_PWM_CHANNEL, duty);
    err = err | ledc_update_duty(SERVO_LEDC_MODE, SERVO_PWM_CHANNEL);

    if (DEBUG_MODE) ESP_LOGI(TAG, "servo_set_angle(); Servo angle set to %f degrees (duty: %f)\n", angle_deg, duty);
    return err;
}


void relay_init(int relay_pin) {
    // Set relay output pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << relay_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(relay_pin, 0);
    ESP_LOGI(TAG, "relay_init(); Relay initialized: GPIO %d", relay_pin);

}


void relay_enable(int relay_pin) {
    gpio_set_level(relay_pin, 1);
    ESP_LOGW(TAG, "");
    ESP_LOGW(TAG, "relay_enable(); Motors ENABLED via relay on GPIO %d", relay_pin);
    ESP_LOGW(TAG, "");

}


void relay_disable(int relay_pin) {
    gpio_set_level(relay_pin, 0);
    ESP_LOGI(TAG, "relay_diable(); Motors disabled via relay on GPIO %d", relay_pin);
}

