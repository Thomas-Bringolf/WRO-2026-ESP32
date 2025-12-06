#include "motors.h"
#include "spi.h"
#include "analog.h"

#include "freertos/FreeRTOS.h"
#include "driver/spi_slave.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

//Relay
#define PWRrelai 4

// Motor
#define MotorPWM 5
#define MOTOR_DIR_PIN 6

// Servo
#define SERVO_PWM_PIN 7

// SPI
#define SPI_MOSI   12
#define SPI_MISO   13
#define SPI_SCLK   15
#define SPI_CS     14
#define SPI_REQ    16
#define SPI_RCV_HOST    SPI2_HOST

// Analog
#define I2C_ADDR 0x48   //connect ADDR to GND
#define I2C_PORT_NUM I2C_NUM_0
#define SDA_PIN 8
#define SCL_PIN 9

#define TAG "MAIN_MODULE"

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_DEBUG);
    esp_log_level_set("SPI_MODULE", ESP_LOG_DEBUG);

    Motor motor = {
        .pwm_pin = MotorPWM,
        .dir_pin = MOTOR_DIR_PIN,
        .current_speed = 0.0f,
        .step_size = 0.01f,       // speed increment per step
        .step_delay_ms = 10     // delay per step in ms
    };
    motor_init(&motor);

    Servo servo = {
        .pwm_pin = SERVO_PWM_PIN,
        .min_duty = 500,
        .max_duty = 2500,
        .center_duty = 1500,
        .angle_range = 90
    };
    servo_init(&servo);

    Spi spi = {
        .bufsize = 256,
        .host = SPI_RCV_HOST,
        .dma_chan = SPI_DMA_CH_AUTO,
        .buscfg = {
            .mosi_io_num = SPI_MOSI,
            .miso_io_num = SPI_MISO,
            .sclk_io_num = SPI_SCLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
        },
        .slvcfg = {
            .mode = 0,
            .spics_io_num = SPI_CS,
            .queue_size = 1,
            .flags = 0,
        },
        .req_pin = SPI_REQ
    };
    spi_slave_init(&spi);

    AnalogSensor adc = {
        .i2c_addr = I2C_ADDR,
        .i2c_port = I2C_PORT_NUM,
        .sda_pin = SDA_PIN,
        .scl_pin = SCL_PIN,
        .gain = ADS111X_GAIN_4V096,
        .conversion_factor[0] = 1.0f,
        .conversion_factor[1] = 1.0f,
        .conversion_factor[2] = 1.0f,
        .conversion_factor[3] = 1.0f
    };
    analog_init(&adc);

    while (1) {
        // Read all channels
        analog_read(&adc);

        // Log all channels
        analog_log(&adc);

        vTaskDelay(pdMS_TO_TICKS(500)); // 500 ms delay
    }
}





/**
    SpiMessage message = {0};
    while (true) {
        spi_slave_rx(&spi);
        if(spi_decode_message(&spi, &message) == ESP_OK) {
            message_log(&message);
            
            // send test message
            message_setString(&message, "Hey tu :)");
            message.type = MSG_REP_ASCII;
            message_log(&message);
            spi_encode_message(&spi, &message);
            spi_slave_tx(&spi);
        
        }    
    }
*/

/**    
    bool keepOnGoing = true;
    while (keepOnGoing) {
        motors_enable(PWRrelai);
        printf("Setting servo to -45 degrees\n");
        servo_sets_angle(&servo, -45);

        vTaskDelay(pdMS_TO_TICKS(2000));
        printf("Setting servo to +45 degrees\n");
        servo_sets_angle(&servo, 45);

        vTaskDelay(pdMS_TO_TICKS(2000));
        printf("Setting servo to 0 degrees\n");
        servo_sets_angle(&servo, 0);

        vTaskDelay(pdMS_TO_TICKS(2000));
        
        printf("Motor forward \n");
        motor_set_speed(&motor, 1.0f, true);
        vTaskDelay(pdMS_TO_TICKS(4000));

        printf("Motor stop\n");
        motor_stop(&motor);
        vTaskDelay(pdMS_TO_TICKS(1000));

        printf("Motor backward \n");
        motor_set_speed(&motor, -1.0f, true);
        vTaskDelay(pdMS_TO_TICKS(4000));

        printf("Motor stop\n");
        motor_stop(&motor);
        vTaskDelay(pdMS_TO_TICKS(1000));
        keepOnGoing = false;
    }
    motors_disable(PWRrelai);
*/
