#include "spi.h"
#include "dispatch.h"
#include "esp_log.h"
#include "esp_err.h"


#define DEBUG_MODE true
#define TAG "MESSAGE_MODULE"


esp_err_t dispatch_message(Spi *spi, const SpiMessage *rcv_msg, const message_handler_t *handlers, size_t handler_count) {
    if (!spi || !rcv_msg) return ESP_ERR_INVALID_ARG;

    // If it is a ping message, respond imediately with ACK
    if (rcv_msg->type == MSG_PING) {
        message_sendACK(spi);
        return ESP_OK;
    }

    for (size_t i = 0; i < handler_count; i++) {
        // Check wether register match
        if (handlers[i].reg != rcv_msg->reg) continue;

        // Check getter callback validity
        if (!handlers[i].getter_callback) {
            message_sendError(spi, TAG, "No getter callback defined for register %u", handlers[i].reg);
            return ESP_FAIL;
        }

        // Otherwise If it is a get message, call the getter and pass t
        if (rcv_msg->type == MSG_GET_REG) {
            return handlers[i].getter_callback(spi, rcv_msg);
        }

        // It is not a get message, if it is a read-only register send an error
        if (handlers[i].read_only) {
            message_sendError(spi, TAG, "Write operation (%s) is not supported by read-only register %u", MsgType_toString(rcv_msg->type), handlers[i].reg);
            return ESP_FAIL;
        }

        // Check setter callback validity
        if (!handlers[i].setter_callback) {
            message_sendError(spi, TAG, "No setter callback defined for register %u", handlers[i].reg);
            return ESP_FAIL;
        }

        return handlers[i].setter_callback(spi, rcv_msg);
    }

    message_sendError(spi, TAG, "Register %u not implemented ;<|", rcv_msg->reg);
    return ESP_FAIL;
}

