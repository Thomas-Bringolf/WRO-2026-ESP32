#include "analog.h"
#include "encoder.h"
#include "motors.h"
#include "spi.h"
#include "dispatch.h"
//#include "i2c_scann.c"

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
#define PWR_RELAY 15

// Motor
#define MOTOR_PWM 5
#define MOTOR_DIR 6

// Servo
#define SERVO_PWM 7
#define SERVO_LEDC_MODE LEDC_LOW_SPEED_MODE
#define SERVO_PWM_CHANNEL LEDC_CHANNEL_1
// SPI
#define SPI_MOSI   11
#define SPI_MISO   13
#define SPI_SCLK   12
#define SPI_CS     10
#define SPI_REQ    14
#define SPI_RCV_HOST    SPI2_HOST

// Analog
#define I2C_ADDR 0x48   //connect ADDR to GND
#define I2C_PORT_NUM I2C_NUM_0
#define I2C_SDA 8
#define I2C_SCL 9

// Encoder
#define ENCODER_CHANNEL_A   17
#define ENCODER_CHANNEL_B   16

// Message handlers
#define MESSAGE_HANDLERS_COUNT 10

// SPI registers
#define REG_INVALID                             0   // Used for ERR / ACK
#define REG_PLATFORM_ENABLE                     2   // UINT16: 0=Disable, 1=Enable Platform
#define REG_MOTOR_DISTANCE_RESET                3   // UINT16: 0=Do nothing, 1=Reset

#define REG_ANALOG_BATTERY_VOLTAGE              5   // FLOAT32 (read-only): 0..20 [V]
#define REG_SUBSCRIBE_ANALOG_BATTERY_VOLTAGE_MS 6   // UINT16: 0=Disable, 1..60000 [ms]
#define REG_ANALOG_BATTERY_CURRENT              7   // FLOAT32 (read-only): -20..0..20 [A]
#define REG_SUBSCRIBE_ANALOG_BATTERY_CURRENT_MS 8   // UINT16: 0=Disable, 1..60000 [ms]
#define REG_ANALOG_MOTOR_CURRENT                9   // FLOAT32 (read-only): 0..20 [A]
#define REG_SUBSCRIBE_ANALOG_MOTOR_CURRENT_MS   10  // UINT16: 0=Disable, 1..60000 [ms]

#define REG_SERVO_ANGLE_TARGET                  15  // FLOAT32: -90..0..90 [°], default=0
#define REG_MOTOR_SPEED_TARGET                  16  // FLOAT32: Motor speed setpoint [m/s]
#define REG_MOTOR_SPEED_ACTUAL                  17  // FLOAT32 (read-only): -10..0..10 [m/s]
#define REG_SUBSCRIBE_MOTOR_SPEED_ACTUAL_MS     18  // UINT16: 0=Disable, 1..60000 [ms]
#define REG_MOTOR_DISTANCE_TOTAL                19  // FLOAT32 (read-only): total distance [m]
#define REG_SUBSCRIBE_MOTOR_DISTANCE_TOTAL_MS   20  // UINT16: 0=Disable, 1..60000 [ms]
#define REG_SUBSCRIBE_MOTOR_DISTANCE_TOTAL_M    21  // FLOAT32: 0=Disable, n [m] step between updates


#define TAG "MAIN_MODULE"

Motor motor = {
    .pwm_pin = MOTOR_PWM,
    .dir_pin = MOTOR_DIR,
    .current_speed = 0.0f,
    .step_size = 0.01f,       // speed increment per step
    .step_delay_ms = 2     // delay per step in ms
};

Servo servo = {
    .pwm_pin = SERVO_PWM,
    .min_duty = 560,
    .max_duty = 1850,
    .center_duty = 1205,
    .angle_range = 90
};

Encoder encoder = {
    .channelA_pin = ENCODER_CHANNEL_A,
    .channelB_pin = ENCODER_CHANNEL_B,
};

Spi spi = {
    .bufsize = 1024,
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

int32_t enc_accumulator;
SpiMessage rcv_message = {0};

esp_err_t callback_voltage_getter(Spi *spi, SpiMessage *msg) {
    int channel;
    switch (msg->reg) {
        case REG_ANALOG_BATTERY_VOLTAGE:
            channel = 0;
            break;

        case REG_ANALOG_BATTERY_CURRENT:
            channel = 1;
            break;

        case REG_ANALOG_MOTOR_CURRENT:
            channel = 2;
            break;

        default:
            message_sendError(spi, TAG, "voltage_getter(); invalide register provided %u", msg->reg);
            return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = analog_read_ch(&adc, channel);
    if (err != ESP_OK) {
        message_sendError(spi, TAG, "An error occured while reading analog sensor");
    } else {
        message_sendFloat32(spi, msg->reg, adc.last_voltage[channel]);
    }
    return ESP_OK;

}

esp_err_t callback_servo_angle_getter(Spi *spi, SpiMessage *msg) {
    message_sendFloat32(spi, msg->reg, servo.last_angle);
    return ESP_OK;
}

esp_err_t callback_servo_angle_setter(Spi *spi, SpiMessage *msg) {
    if (msg->type != MSG_SET_REG_F32) {
        message_sendError(spi, TAG, "Message type %s is not supported by register %u", MsgType_toString(msg->type), msg->reg);
        return ESP_FAIL;
    }
    esp_err_t err = servo_sets_angle(&servo, message_toFloat32(msg));
    if (err) {
        message_sendError(spi, TAG, "error occured while setting servo angel");
    } else {
        message_sendACK(spi);
    }
    return err;
}


message_handler_t handlers[MESSAGE_HANDLERS_COUNT] = {
    [0] = {
        .reg = REG_ANALOG_BATTERY_VOLTAGE,
        .read_only = true,
        .getter_callback = callback_voltage_getter,
        .setter_callback = NULL
    },
    [1] = {
        .reg = REG_ANALOG_BATTERY_CURRENT,
        .read_only = true,
        .getter_callback = callback_voltage_getter,
        .setter_callback = NULL
    },
    [2] = {
        .reg = REG_ANALOG_MOTOR_CURRENT,
        .read_only = true,
        .getter_callback = callback_voltage_getter,
        .setter_callback = NULL
    },
    [3] = {
        .reg = REG_SERVO_ANGLE_TARGET,
        .read_only = false,
        .getter_callback = callback_servo_angle_getter,
        .setter_callback = callback_servo_angle_setter
    }
};

void app_main(void) {
    relay_init(PWR_RELAY);

    motor_init(&motor);
    servo_init(&servo);
    encoder_init(&encoder);
    spi_slave_init(&spi);
    analog_init(&adc);

    relay_enable(PWR_RELAY);
    encoder_reset(&encoder);

    vTaskDelay(pdMS_TO_TICKS(1000));

    while (true) {
        if (spi_receive_message(&spi, &rcv_message)) {
            message_sendError(&spi, TAG, "error while reciving transaction");
        }
        dispatch_message(&spi, &rcv_message, handlers, MESSAGE_HANDLERS_COUNT);
    }

    relay_disable(PWR_RELAY);

}



/** Accummulator
//        enc_accumulator = encoder_read(&encoder);
//        send_message.type = MSG_REP_I32;
//        send_message.reg = 02;
//        message_setInt32(&send_message, enc_accumulator);
//        spi_send_message(&spi, &send_message); 
 */


/**   SPI + simple relay control example
    while (true) {
        spi_receive_message(&spi, &rcv_message);
        if (rcv_message.reg == 0x01) {
            if (rcv_message.type != MSG_SET_REG_U16) {
                send_error(&spi, TAG, "Message Type not supported by Register 0x01");

            } else if (message_toUint16(&rcv_message) == 0) {
                relay_disable(PWR_RELAY);
                send_message.type = MSG_SLAVE_ACK;
                spi_send_message(&spi, &send_message);

            } else if (message_toUint16(&rcv_message) == 1) {
                relay_enable(PWR_RELAY);
                send_message.type = MSG_SLAVE_ACK;
                spi_send_message(&spi, &send_message);
                
            } else {
                send_error(&spi, TAG, "Value not supported by Register 0x01");
            }
            
        } 
    }
*/


/** ADC example
    while (1) {
        // Read all channels
        analog_read(&adc);

        // Log all channels
        analog_log(&adc);

        vTaskDelay(pdMS_TO_TICKS(500)); // 500 ms delay
    }
*/


/**  Motor (Open-loop) + Servo Example
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



