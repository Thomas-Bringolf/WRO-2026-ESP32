#include "analog.h"
#include "encoder.h"
#include "motors.h"
#include "spi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

//Relay
#define PWR_RELAY 4

// Motor
#define MOTOR_PWM 5
#define MOTOR_DIR 6

// Servo
#define SERVO_PWM 7

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
#define I2C_SDA 8
#define I2C_SCL 9


// Encoder
#define ENCODER_CHANNEL_A   4
#define ENCODER_CHANNEL_B   5

#define TAG "MAIN_MODULE"


void app_main(void) {

    relay_init(PWR_RELAY);

    Motor motor = {
        .pwm_pin = MOTOR_PWM,
        .dir_pin = MOTOR_DIR,
        .current_speed = 0.0f,
        .step_size = 0.01f,       // speed increment per step
        .step_delay_ms = 10     // delay per step in ms
    };
    motor_init(&motor);

    Servo servo = {
        .pwm_pin = SERVO_PWM,
        .min_duty = 500,
        .max_duty = 2500,
        .center_duty = 1500,
        .angle_range = 90
    };
    servo_init(&servo);

    Encoder encoder = {
        .channelA_pin = ENCODER_CHANNEL_A,
        .channelB_pin = ENCODER_CHANNEL_B,
    };
    encoder_init(&encoder);

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
        .sda_pin = I2C_SDA,
        .scl_pin = I2C_SCL,
        .gain = ADS111X_GAIN_4V096,
        .conversion_factor[0] = 1.0f,
        .conversion_factor[1] = 1.0f,
        .conversion_factor[2] = 1.0f,
        .conversion_factor[3] = 1.0f
    };
    analog_init(&adc);

    SpiMessage rcv_message = {0};
    SpiMessage send_message = {0};

    while (true) {
        spi_recive_message(&spi, &rcv_message);
        if (rcv_message.reg == 0x01) {
            if (rcv_message.type != MSG_SET_REG_U16) {
                send_error(&spi, "Message Type not supported by Register 0x01");
            } else if (message_toUint16(&rcv_message) == 0) {
                relay_disable(PWR_RELAY);
                send_message.type = MSG_SLAVE_ACK;
                spi_send_message(&spi, &send_message);
            } else if (message_toUint16(&rcv_message) == 1) {
                relay_enable(PWR_RELAY);
                send_message.type = MSG_SLAVE_ACK;
                spi_send_message(&spi, &send_message);
            } else {
                send_error(&spi, "Value not supported by Register 0x01");

            }
            
        } 
    }
}


/**
    while (1) {
        // Read all channels
        analog_read(&adc);

        // Log all channels
        analog_log(&adc);

        vTaskDelay(pdMS_TO_TICKS(500)); // 500 ms delay
    }
*/



/**    
    bool keepOnGoing = true;
    while (keepOnGoing) {
        motors_enable(PWR_RELAI);
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
    motors_disable(PWR_RELAI);
*/
