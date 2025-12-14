#ifndef DISPATCH_H
#define DISPATCH_H

#include "spi.h"

#include "esp_err.h"

typedef esp_err_t (*msg_callback_t)(Spi *spi, SpiMessage *msg);

typedef struct {
    uint16_t reg;               // register to match
    bool read_only;
    msg_callback_t getter_callback;      // function to call on match
    msg_callback_t setter_callback;      // function to call on match
} message_handler_t;



esp_err_t dispatch_message(Spi *spi, const SpiMessage *rcv_msg, const message_handler_t *handlers, size_t handler_count);


#endif // SPI_DISPATCH_H