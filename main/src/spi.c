#include "spi.h"

#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#define DEBUG_MODE false
#define TAG "SPI_MODULE"

#define SPI_FRAME_OVERHEAD 9



bool is_buf_empty(char *buf, size_t size) {
    for (size_t i = 0; i < size; i++) {
        if (buf[i] != 0) {
            return false;
        }
    }
    return true;
}


void log_first_bytes_hex(char *buf, int len) {
    char line[len * 3 + 1]; // 2 hex chars + space per byte + null terminator

    for (size_t i = 0; i < len; i++) {
        sprintf(&line[i * 3], "%02X ", buf[i]);
    }
    line[len * 3] = '\0'; // null-terminate

    if (DEBUG_MODE) ESP_LOGI(TAG, "First %d bytes: %s", (int)len, line);
}


const char* MsgType_toString(MsgType type) {
    switch (type) {

        // --- Master → Slave ---
        case MSG_MASTER_ACK:        return "MSG_MASTER_ACK";
        case MSG_MASTER_ERR:        return "MSG_MASTER_ERR";
        case MSG_SET_REG_ASCII:     return "MSG_SET_REG_ASCII";
        case MSG_SET_REG_U16:       return "MSG_SET_REG_U16";
        case MSG_SET_REG_F32:       return "MSG_SET_REG_F32";
        case MSG_GET_REG:           return "MSG_GET_REG";
        case MSG_PING:              return "MSG_PING";

        // --- Slave → Master ---
        case MSG_SLAVE_ACK:         return "MSG_SLAVE_ACK";
        case MSG_SLAVE_ERR:         return "MSG_SLAVE_ERR";
        case MSG_REP_ASCII:         return "MSG_REP_ASCII";
        case MSG_REP_U16:           return "MSG_REP_U16";
        case MSG_REP_I32:           return "MSG_REP_I32";
        case MSG_REP_F32:           return "MSG_REP_F32";

        default:                    return "UNKNOWN_MSG_TYPE";
    }
}


char* message_toString(const SpiMessage *message) {
        // Check if message is valid
    if (!message || !message->content || message->length == 0) {
        ESP_LOGE(TAG, "message_toString(); Error when extracting string from message");
        return NULL;
    }
    // Prepare buffer to hold null-terminated string
    char *str = malloc(message->length + 1 );

    // Copy the message to the new buffer
    memcpy(str, message->content, message->length);

    // Add null terminator
    str[message->length] = '\0';
    return str;
}


uint16_t message_toUint16(const SpiMessage *message) {
    // Check if message is valid
    if (!message || !message->content || message->length != 2) {
        ESP_LOGE(TAG, "message_toUnit16(); Error when extracting uint16 from message");
        return 0;
    }

    // Big-endian content copy
    uint16_t value = (message->content[0] << 8) |
                     (message->content[1]);

    return value;
}


int32_t message_toInt32(const SpiMessage *message) {
    // Check if message is valid
    if (!message || !message->content || message->length != 4) {
        ESP_LOGE(TAG, "message_toInt32(); Error when extracting int32 from message");
        return 0;
    }

    // Big-endian content copy
    int32_t value = (message->content[0] << 24) |
                    (message->content[1] << 16) |
                    (message->content[2] << 8) |
                    (message->content[3]);

    return value;
}


float message_toFloat32(const SpiMessage *message) {
    if (!message || !message->content || message->length != 4) {
        ESP_LOGE(TAG, "message_toFloat32(); Error when extracting float32 from message");
        return 0.0f;
    }

    uint32_t temp = ((uint32_t)message->content[0] << 24) |
                    ((uint32_t)message->content[1] << 16) |
                    ((uint32_t)message->content[2] << 8) |
                    ((uint32_t)message->content[3]);

    float value;
    memcpy(&value, &temp, sizeof(float));  // safe copy to float
    return value;
}


esp_err_t message_sendString(Spi *spi, uint16_t reg, const char *fmt, ...) {
    if (!spi || !fmt) {
        ESP_LOGE(TAG, "message_sendString(); ESP_ERR_INVALID_ARG, no spi object or String given");
        return ESP_ERR_INVALID_ARG;
    }

    // Allocate max possible amount of memory for the current sendbuf size plus one byte for null-char
    char buffer[spi->bufsize - SPI_FRAME_OVERHEAD + 1];

    // Some black-magic to build the string
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len < 0) {
        // encoding or format error
        ESP_LOGE(TAG, "message_sendString(); string encoding error");
        return ESP_FAIL;
    } else if ((size_t)len >= sizeof(buffer)) {
        // output was truncated
        ESP_LOGW(TAG, "message_sendString(); message was truncated while encoding. Original size: %d | Truncated size: %d", len, sizeof(buffer));
    }

    SpiMessage message = {
        .type = MSG_REP_ASCII,
        .reg = reg,
        .length =strlen(buffer) // Calculate message size. Stop at the first null-char
    };

    // If the message is an empty sting, we just allocate one byte.
    if (message.length == 0) {
        message.content = NULL;
    } else {
        message.content = malloc(message.length);
        if (!message.content) {
            ESP_LOGE(TAG, "message_sendString(); ESP_ERR_NO_MEM, could not allocate memory");
            return ESP_ERR_NO_MEM;
        }
        memcpy(message.content, buffer, message.length);
    }

    if (DEBUG_MODE) message_log(&message);
    spi_encode_message(spi, &message);
    spi_slave_tx(spi);
    return ESP_OK;
}


esp_err_t message_sendUint16(Spi *spi, uint16_t reg, uint16_t value) {
    if (!spi) {
        ESP_LOGE(TAG, "message_sendUint16(); ESP_ERR_INVALID_ARG, null spi pointer");
        return ESP_ERR_INVALID_ARG;
    }

    SpiMessage message = {
        .type = MSG_REP_U16,
        .reg = reg,
        .length = 2
    };

    message.content = malloc(2);
    if (!message.content) {
        ESP_LOGE(TAG, "message_sendUint16(); ESP_ERR_NO_MEM, could not allocate memory");
        return ESP_ERR_NO_MEM;
    }

    // Big-endian encoding
    message.content[0] = (value >> 8) & 0xFF;
    message.content[1] = value & 0xFF;

    if (DEBUG_MODE) message_log(&message);
    spi_encode_message(spi, &message);
    spi_slave_tx(spi);
    return ESP_OK;
}


esp_err_t message_sendInt32(Spi *spi, uint16_t reg, int32_t value) {
    if (!spi) {
        ESP_LOGE(TAG, "message_sendInt32(); ESP_ERR_INVALID_ARG, null spi pointer");
        return ESP_ERR_INVALID_ARG;
    }

    SpiMessage message = {
        .type = MSG_REP_I32,
        .reg = reg,
        .length = 4
    };

    message.content = malloc(4);
    if (!message.content) {
        ESP_LOGE(TAG, "message_sendInt32(); ESP_ERR_NO_MEM, could not allocate memory");
        return ESP_ERR_NO_MEM;
    }
    // Big-endian encoding
    message.content[0] = (value >> 24) & 0xFF;
    message.content[1] = (value >> 16) & 0xFF;
    message.content[2] = (value >> 8) & 0xFF;
    message.content[3] = value & 0xFF;

    if (DEBUG_MODE) message_log(&message);
    spi_encode_message(spi, &message);
    spi_slave_tx(spi);
    return ESP_OK;
}


esp_err_t message_sendFloat32(Spi *spi, uint16_t reg, float value) {
    if (!spi) {
        ESP_LOGE(TAG, "message_sendFloat32(); ESP_ERR_INVALID_ARG, null spi pointer");
        return ESP_ERR_INVALID_ARG;
    }

    SpiMessage message = {
        .type = MSG_REP_F32,
        .reg = reg,
        .length = 4
    };

    message.content = malloc(4);
    if (!message.content) {
        ESP_LOGE(TAG, "message_sendFloat32(); ESP_ERR_NO_MEM, could not allocate memory");
        return ESP_ERR_NO_MEM;
    }

    // Convert float → uint32_t bit representation
    uint32_t temp;
    memcpy(&temp, &value, sizeof(float));

    // Big-endian encoding
    message.content[0] = (temp >> 24) & 0xFF;
    message.content[1] = (temp >> 16) & 0xFF;
    message.content[2] = (temp >> 8) & 0xFF;
    message.content[3] = temp & 0xFF;

    if (DEBUG_MODE) message_log(&message);
    spi_encode_message(spi, &message);
    spi_slave_tx(spi);
    return ESP_OK;
}


esp_err_t message_sendError(Spi *spi, const char *tag, const char *fmt, ...) {
    if (!spi || !fmt) {
        ESP_LOGE(TAG, "message_setString(); ESP_ERR_INVALID_ARG, no spi object or String given");
        return ESP_ERR_INVALID_ARG;
    }

    // Allocate max possible amount of memory for the current sendbuf size plus one byte for null-char
    char buffer[spi->bufsize - SPI_FRAME_OVERHEAD + 1];

    // Some black-magic to build the string
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len < 0) {
        // encoding or format error
        ESP_LOGE(TAG, "message_sendError(); string encoding error");
    } else if ((size_t)len >= sizeof(buffer)) {
        // output was truncated
        ESP_LOGW(TAG, "message_sendError(); message was truncated while encoding. Original size: %d | Truncated size: %d", len, sizeof(buffer));
    }

    SpiMessage message = {
        .type = MSG_SLAVE_ERR,
        .reg = 0,
        .length =strlen(buffer) // Calculate message size. Stop at the first null-char
    };

    // If the message is an empty sting, we just allocate one byte.
    if (message.length == 0) {
        message.content = malloc(1);
        memset(message.content, 0, 1);
    } else {
        message.content = malloc(message.length);
        if (!message.content) {
            ESP_LOGE(TAG, "message_setString(); ESP_ERR_NO_MEM, could not allocate memory");
            return ESP_ERR_NO_MEM;
        }
        memcpy(message.content, buffer, message.length);
    }

    // Log the error
    ESP_LOGE(tag, "%s", buffer);
    // Send the error via spi
    if (DEBUG_MODE) message_log(&message);
    spi_encode_message(spi, &message);
    spi_slave_tx(spi);

    return ESP_OK;
}


esp_err_t message_sendACK(Spi *spi) {
    if (!spi) {
        ESP_LOGE(TAG, "message_sendACK(); ESP_ERR_INVALID_ARG, null spi pointer");
        return ESP_ERR_INVALID_ARG;
    }

    SpiMessage message = {
        .type = MSG_SLAVE_ACK,
        .reg = 0,
        .length = 0
    };

    message.content = NULL;
    if (DEBUG_MODE) message_log(&message);
    spi_encode_message(spi, &message);
    spi_slave_tx(spi);
    return ESP_OK;
}


esp_err_t message_log(const SpiMessage *message) {
    if (!message) {
        ESP_LOGE(TAG, "message_log(): NULL message pointer");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "");
    // Determine direction and pick color
    const char *dir_str;
    const char *dir_color;

    if (message->type >= 0x01 && message->type <= 0x010) {
        dir_str = "\033[46m [MASTER ⇒ SLAVE]";
        dir_color = "     ";  // Magenta (send)
    } else if (message->type >= 0x11 && message->type <= 0x20) {
        dir_str = "\033[42m [SLAVE ⇒ MASTER]";
        dir_color = "     ";  // Green (receive)
    } else {
        dir_str = "\033[95m [UNKNOWN]";
        dir_color = "     ";  // magenta
    }

    const char *type_str = MsgType_toString(message->type);

    // Header
    ESP_LOGI(TAG, "%s%s\033[0m SPI Message:\033[0m", dir_color, dir_str);

    // Fields
    ESP_LOGI(TAG, "      \033[35mtype\033[0m   = \033[1;37m%s\033[0m (0x%02X)", type_str, message->type);
    ESP_LOGI(TAG, "      \033[35mreg\033[0m    = \033[1;37m%u\033[0m", message->reg);
    ESP_LOGI(TAG, "      \033[35mlength\033[0m = \033[1;37m%u\033[0m", message->length);

    // ---- Decode content based on message type ----
    switch (message->type) {

        // ASCII messages
        case MSG_MASTER_ERR:
        case MSG_SLAVE_ERR:
        case MSG_SET_REG_ASCII:
        case MSG_REP_ASCII:
            ESP_LOGI(TAG, "      \033[35mcontent (ASCII)\033[0m = \033[91m\"%.*s\"\033[0m",
                     message->length,
                     message->content);
            break;

        // uint16 messages
        case MSG_SET_REG_U16:
        case MSG_REP_U16:
            if (message->length == 2) {
                uint16_t value = message_toUint16(message);
                ESP_LOGI(TAG, "      \033[35mcontent (uint16)\033[0m = \033[91m%u\033[0m", value);
            } else {
                ESP_LOGW(TAG, "      \033[0mcontent (uint16) INVALID LENGTH: %u", message->length);
            }
            break;

        // uint16 messages
        case MSG_SET_REG_I32:
        case MSG_REP_I32:
            if (message->length == 4) {
                int32_t value = message_toInt32(message);
                ESP_LOGI(TAG, "      \033[35mcontent (int32)\033[0m = \033[91m%d\033[0m", value);
            } else {
                ESP_LOGW(TAG, "      \033[0mcontent (int32) INVALID LENGTH: %u", message->length);
            }
            break;

        // float32 messages
        case MSG_SET_REG_F32:
        case MSG_REP_F32:
            if (message->length == 4) {
                uint32_t temp = ((uint32_t)message->content[0] << 24) |
                                ((uint32_t)message->content[1] << 16) |
                                ((uint32_t)message->content[2] << 8) |
                                ((uint32_t)message->content[3]);
                float value;
                memcpy(&value, &temp, sizeof(float));
                ESP_LOGI(TAG, "      \033[35mcontent (float32)\033[0m = \033[91m%f\033[0m", value);
            } else {
                ESP_LOGW(TAG, "      content (float32) INVALID LENGTH: %u", message->length);
            }
            break;

        // Messages with no payload
        case MSG_MASTER_ACK:
        case MSG_SLAVE_ACK:
        case MSG_GET_REG:
        case MSG_PING:
            ESP_LOGI(TAG, "      \033[35mcontent\033[0m = \033[1;37m<none>\033[0m");
            break;

        default:
            ESP_LOGW(TAG, "      content = <unknown message type>");
            break;
    }
    ESP_LOGI(TAG, "");
    return ESP_OK;
}


esp_err_t spi_slave_init(Spi *spi) {

    if (spi_slave_initialize(spi->host, &(spi->buscfg), &(spi->slvcfg), spi->dma_chan) != ESP_OK) {
        ESP_LOGE(TAG, "spi_slave_init(); spi_slave_initialize FAILED");
        return ESP_FAIL;
    }

    // RX buffer allocation (aligned for DMA)
    spi->recvbuf = spi_bus_dma_memory_alloc(spi->host, spi->bufsize, 0);
    if (!spi->recvbuf) {
        ESP_LOGE(TAG, "spi_slave_init(); ESP_ERR_NO_MEM, Failed to allocate DMA receive buffer");
        return ESP_ERR_NO_MEM;
    }
    memset(spi->recvbuf, 0, spi->bufsize);

    // TX buffer allocation (aligned for DMA)
    spi->sendbuf = spi_bus_dma_memory_alloc(spi->host, spi->bufsize, 0);
    if (!spi->sendbuf) {
        ESP_LOGE(TAG, "spi_slave_init(); ESP_ERR_NO_MEM, Failed to allocate DMA send buffer");
        return ESP_ERR_NO_MEM;
    }
    memset(spi->sendbuf, 0, spi->bufsize);

    // Set REQ output pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << spi->req_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    if (gpio_config(&io_conf) != ESP_OK) {
        ESP_LOGE(TAG, "spi_slave_init(); Failed to configure REQ GPIO pin");
        return ESP_FAIL;
    }
    gpio_set_level(spi->req_pin, false);

    // Prepare transaction
    spi->transaction.length = spi->bufsize * 8;   // length in bits
    spi->transaction.rx_buffer = spi->recvbuf;
    spi->transaction.tx_buffer = spi->sendbuf;

    ESP_LOGI(TAG, "SPI initialized successfully");
    return ESP_OK;
}


esp_err_t spi_slave_rx(Spi *spi) {
    // clear recive buffer (Unsessesary since normaly overwritten in a synchonous recive operation)
    //memset(spi->recvbuf, 0, spi->bufsize);

    // Queue transaction
    gpio_set_level(spi->req_pin, false);
    esp_err_t return_code = spi_slave_transmit(spi->host, &(spi->transaction), portMAX_DELAY);
    if (return_code != ESP_OK) {
        ESP_LOGE(TAG, "spi_slave_rx(); Failed to queue SPI transaction: %s", esp_err_to_name(return_code));
        return return_code;
    }

    //Clear send buffer after transaction
    memset(spi->sendbuf, 0, spi->bufsize);
    if (DEBUG_MODE) ESP_LOGI(TAG, "spi_slave_rx(); SPI read transaction successful");
    return ESP_OK;
}


esp_err_t spi_slave_tx(Spi *spi) {
    //Prepare retun_code
    esp_err_t return_code = ESP_OK;

    // Clear RX buffer
    memset(spi->recvbuf, 0, spi->bufsize);

    // Queue transaction
    return_code = spi_slave_queue_trans(spi->host, &(spi->transaction), pdMS_TO_TICKS(TX_TIMEOUT_MS));
    if (return_code == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "Failed add  SPI transaction to queue, TIMEOUT of %dms reached", TX_TIMEOUT_MS);
        return return_code;
    } else if (return_code != ESP_OK) {
        ESP_LOGE(TAG, "spi_slave_tx(); Failed to queue SPI transaction: %s", esp_err_to_name(return_code));
        return return_code;
    }

    // REQ to master + Wait for transaction result
    gpio_set_level(spi->req_pin, true);
    return_code = spi_slave_get_trans_result(spi->host, &(spi->transaction_result), pdMS_TO_TICKS(TX_TIMEOUT_MS));
    if (return_code == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "Failed transmit SPI transaction to master, TIMEOUT of %dms reached", TX_TIMEOUT_MS);
        return return_code;
    } else if (return_code != ESP_OK) {
        ESP_LOGE(TAG, "spi_slave_tx(); Failed to get SPI transaction result: %s", esp_err_to_name(return_code));
        return return_code;
    }
    if (DEBUG_MODE) ESP_LOGI(TAG, "spi_slave_tx(); SPI send transaction successful");

    //Clear send buffer after transaction
    memset(spi->sendbuf, 0, spi->bufsize);
    if (!is_buf_empty(spi->recvbuf, spi->bufsize)) {
        ESP_LOGE(TAG, "spi_slave_tx(); ESP_ERR_INVALID_RESPONSE, recived message while tansmitting");
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}


esp_err_t spi_decode_message(Spi *spi, SpiMessage *message) {
    // Formal Check of Start Byte
    if (spi->recvbuf[0] != 0x55) {
        ESP_LOGE(TAG, "spi_decode_message(); Invalid start byte 0x%02X", spi->recvbuf[0]);
        return ESP_FAIL;
    }

    message->type = spi->recvbuf[1];
    message->reg = (spi->recvbuf[2] << 8) | spi->recvbuf[3];
    message->length = (spi->recvbuf[4] << 8) | spi->recvbuf[5];
    message->checksum = (spi->recvbuf[6 + message->length] << 8) | spi->recvbuf[7 + message->length];

    // Check if buffer is big enougth
    if (spi->bufsize < 9 + message->length) {
        ESP_LOGE(TAG, "spi_decode_message(); SPI buffer too small for message length %u", message->length);
        return ESP_ERR_INVALID_SIZE;
    }

    // Formal check of end Byte
    if (spi->recvbuf[8 + message->length] != 0xAA) {
        ESP_LOGE(TAG, "spi_decode_message(); Invalid end byte or length mismatch");
        return ESP_FAIL;
    }

    // If content buffer is already allocated, free it first
    if (message->content) {
        free(message->content);
        message->content = NULL;
    }

    // Allocate content buffer
    message->content = (char *)malloc(message->length); // +1 for null terminator
    if (!message->content && message->length != 0) {
        ESP_LOGE(TAG, "spi_decode_message(); ESP_ERR_NO_MEM, could not allocate content buffer");
        return ESP_ERR_NO_MEM;
    }
    memcpy(message->content, &spi->recvbuf[6], message->length);

    // Calculate checksum
    uint16_t computed_checksum = 0;
    // CRC-16/ARC over [MsgType] + [Register] + [ContentSize] + [Content], Polynomial: 0x8005, Initial value: 0x0000
    for (size_t i = 1; i < 6 + message->length; i++) {
        computed_checksum ^= (uint16_t)(spi->recvbuf[i]);
        for (int j = 0; j < 8; j++) {
            if (computed_checksum & 0x0001) {
                computed_checksum = (computed_checksum >> 1) ^ 0xA001;
            } else {
                computed_checksum >>= 1;
            }
        }
    }

    if (computed_checksum != message->checksum) {
        ESP_LOGE(TAG, "spi_decode_message(); Checksum mismatch (computed: 0x%04X, received: 0x%04X)", computed_checksum, message->checksum);
        return ESP_ERR_INVALID_CRC;
    }
    return ESP_OK;
}


esp_err_t spi_encode_message(Spi *spi, SpiMessage *message) {
    esp_err_t err = ESP_OK;

    // Check if spi send buffer will be lare enough at all
    if (spi->bufsize < SPI_FRAME_OVERHEAD) {
        ESP_LOGE(TAG, "spi_encode_message(); ESP_ERR_INVALID_SIZE, SPI buffer size too small to contain a valid message");
        err = ESP_ERR_INVALID_SIZE;
        goto cleanup;
    }

    // Check if message type is legale
    switch (message->type) {
        //--- Master to Slave --- (All illegale)
        case MSG_MASTER_ACK:
        case MSG_MASTER_ERR:
        case MSG_SET_REG_ASCII:
        case MSG_SET_REG_U16:
        case MSG_SET_REG_F32:
        case MSG_PING:
        case MSG_GET_REG:
            ESP_LOGE(TAG, "spi_encode_message(); ESP_ERR_NOT_ALLOWED, SPI message of type %02X illegale", message->type);
            err = ESP_ERR_NOT_ALLOWED;
            goto cleanup;

        //--- Slave to Master ---
        case MSG_SLAVE_ACK:
        case MSG_SLAVE_ERR:
        case MSG_REP_ASCII:
        case MSG_REP_U16:
        case MSG_REP_I32:
        case MSG_REP_F32:
            break;

        default:
            ESP_LOGE(TAG, "spi_encode_message(); ESP_ERR_NOT_ALLOWED, SPI message of type %02X does not exist", message->type);
            err = ESP_ERR_NOT_ALLOWED;
            goto cleanup;
    }

    // Check buffer size again now that we know the size of the message content
    if (message->length > spi->bufsize - SPI_FRAME_OVERHEAD) {
        ESP_LOGE(TAG, "spi_encode_message(); ESP_ERR_INVALID_SIZE, SPI buffer size too small to contain the message of lenth %d", message->length);
        err = ESP_ERR_INVALID_SIZE;
        goto cleanup;
    }

    // [1-9] Build sendbuff
    // 1. Clear sendbuf
    memset(spi->sendbuf, 0, spi->bufsize);
    // 2. Add start byte
    spi->sendbuf[0] = 0x55;
    // 3. Copy MsgType
    spi->sendbuf[1] = message->type;
    // 4. Copy register number (uint16, big-endian)
    spi->sendbuf[2] = (message->reg >> 8) & 0x00FF;     // high byte
    spi->sendbuf[3] = message->reg & 0x00FF;            // low byte
    // 5. Copy message length (uint16, big-endian)
    spi->sendbuf[4] = (message->length >> 8) & 0x00FF;  // high byte
    spi->sendbuf[5] = message->length & 0x00FF;         // low byte
    // 6. Copy message content (here the null char at the end will not be copied!)
    memcpy(spi->sendbuf + 6, message->content, message->length);
    // 7. Calculate CRC [CRC-16/ARC over [MsgType] + [Register] + [ContentSize] + [Content], Polynomial: 0x8005, Initial value: 0x0000]
    uint16_t computed_checksum = 0;
    for (size_t i = 1; i < 6 + message->length; i++) {
        computed_checksum ^= (uint16_t)(spi->sendbuf[i]);
        for (int j = 0; j < 8; j++) {
            if (computed_checksum & 0x0001) {
                computed_checksum = (computed_checksum >> 1) ^ 0xA001;
            } else {
                computed_checksum >>= 1;
            }
        }
    }
    // 8. Copy CRC (big-endian)
    spi->sendbuf[6 + message->length] = (computed_checksum >> 8) & 0x00FF;
    spi->sendbuf[7 + message->length] = computed_checksum & 0x00FF;
    // 9.Add end byte
    spi->sendbuf[8 + message->length] = 0xAA;

    // free message to avoid memory leaks
    cleanup:
        if (message->content != NULL) free(message->content);
        message->content = NULL;
        return err;
}


esp_err_t spi_receive_message(Spi *spi, SpiMessage *message) {
    spi_slave_rx(spi);
    if (spi_decode_message(spi, message) != ESP_OK) {
        ESP_LOGW(TAG, "Somthing went wrong while decoding message");
        return ESP_FAIL;
    }
    if (DEBUG_MODE) message_log(message);
    return ESP_OK;
}
