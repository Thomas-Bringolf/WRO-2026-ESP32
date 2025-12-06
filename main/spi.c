#include "spi.h"

#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "esp_err.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#define TAG "SPI_MODULE"


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

    ESP_LOGI(TAG, "First %d bytes: %s", (int)len, line);
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


uint16_t message_toUnit16(const SpiMessage *message) {
    // Check if message is valid
    if (!message || !message->content || message->length != 2) {
        ESP_LOGE(TAG, "message_toUnit16(); Error when extracting uint16 from message");
        return 0;
    }

    // Big-endian content copy
    uint16_t value = ((uint16_t)message->content[0] << 8) |
                     ((uint16_t)message->content[1]);

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


esp_err_t message_setString(SpiMessage *message, const char *str)
{
    if (!message || !str) {
        ESP_LOGE(TAG, "message_setString(); ESP_ERR_INVALID_ARG, no message or String given");
        return ESP_ERR_INVALID_ARG;
    }

    // Free existing content
    if (message->content) {
        free(message->content);
        message->content = NULL;
    }

    size_t len = strlen(str);
    message->content = malloc(len);
    if (!message->content) {
        ESP_LOGE(TAG, "message_setString(); ESP_ERR_NO_MEM, could not allocate memory");
        return ESP_ERR_NO_MEM;
    }

    memcpy(message->content, str, len);
    message->length = len;

    return ESP_OK;
}


esp_err_t message_setUint16(SpiMessage *message, uint16_t value)
{
    if (!message) {
        ESP_LOGE(TAG, "message_setUint16(); ESP_ERR_INVALID_ARG, null message pointer");
        return ESP_ERR_INVALID_ARG;
    }

    // Free existing content
    if (message->content) {
        free(message->content);
        message->content = NULL;
    }

    message->content = malloc(2);
    if (!message->content) {
        ESP_LOGE(TAG, "message_setUint16(); ESP_ERR_NO_MEM, could not allocate memory");
        return ESP_ERR_NO_MEM;
    }

    // Big-endian encoding
    message->content[0] = (value >> 8) & 0xFF;
    message->content[1] = value & 0xFF;

    message->length = 2;

    return ESP_OK;
}


esp_err_t message_setFloat32(SpiMessage *message, float value)
{
    if (!message) {
        ESP_LOGE(TAG, "message_setFloat32(); ESP_ERR_INVALID_ARG, null message pointer");
        return ESP_ERR_INVALID_ARG;
    }

    // Free existing content
    if (message->content) {
        free(message->content);
        message->content = NULL;
    }

    message->content = malloc(4);
    if (!message->content) {
        ESP_LOGE(TAG, "message_setFloat32(); ESP_ERR_NO_MEM, could not allocate memory");
        return ESP_ERR_NO_MEM;
    }

    // Convert float → uint32_t bit representation
    uint32_t temp;
    memcpy(&temp, &value, sizeof(float));

    // Big-endian encoding
    message->content[0] = (temp >> 24) & 0xFF;
    message->content[1] = (temp >> 16) & 0xFF;
    message->content[2] = (temp >> 8) & 0xFF;
    message->content[3] = temp & 0xFF;

    message->length = 4;

    return ESP_OK;
}


esp_err_t message_log(const SpiMessage *message)
{
    esp_log_level_set("*", ESP_LOG_DEBUG);
    esp_log_level_set("SPI_MODULE", ESP_LOG_DEBUG);
    if (!message) {
        ESP_LOGE(TAG, "message_log(): NULL message pointer");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "");
    // Determine direction and pick color
    const char *dir_str;
    const char *dir_color;

    if (message->type >= 0x01 && message->type <= 0x07) {
        dir_str = "MASTER → SLAVE";
        dir_color = "\033[46m";  // Magenta (send)
    } else if (message->type >= 0x11 && message->type <= 0x15) {
        dir_str = "SLAVE → MASTER";
        dir_color = "\033[42m";  // Green (receive)
    } else {
        dir_str = "UNKNOWN";
        dir_color = "\033[95m";  // magenta
    }

    const char *type_str = MsgType_toString(message->type);

    // Header
    ESP_LOGI(TAG, "%s[%s] SPI Message:\033[0m", dir_color, dir_str);

    // Fields
    ESP_LOGI(TAG, "  \033[35mtype\033[0m   = \033[1;37m%s\033[0m (0x%02X)", type_str, message->type);
    ESP_LOGI(TAG, "  \033[35mreg\033[0m    = \033[1;37m%u\033[0m", message->reg);
    ESP_LOGI(TAG, "  \033[35mlength\033[0m = \033[1;37m%u\033[0m", message->length);

    // ---- Decode content based on message type ----
    switch (message->type) {

        // ASCII messages
        case MSG_MASTER_ERR:
        case MSG_SLAVE_ERR:
        case MSG_SET_REG_ASCII:
        case MSG_REP_ASCII:
            ESP_LOGI(TAG, "  \033[35mcontent (ASCII)\033[0m = \033[91m\"%.*s\"\033[0m",
                     message->length,
                     message->content);
            break;

        // uint16 messages
        case MSG_SET_REG_U16:
        case MSG_REP_U16:
            if (message->length == 2) {
                uint16_t value = ((uint16_t)message->content[0] << 8) |
                                 ((uint16_t)message->content[1]);
                ESP_LOGI(TAG, "  \033[35mcontent (uint16)\033[0m = \033[91m%u\033[0m", value);
            } else {
                ESP_LOGW(TAG, "  content (uint16) INVALID LENGTH: %u", message->length);
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
                ESP_LOGI(TAG, "  \033[35mcontent (float32)\033[0m = \033[91m%f\033[0m", value);
            } else {
                ESP_LOGW(TAG, "  content (float32) INVALID LENGTH: %u", message->length);
            }
            break;

        // Messages with no payload
        case MSG_MASTER_ACK:
        case MSG_SLAVE_ACK:
        case MSG_GET_REG:
        case MSG_PING:
            ESP_LOGI(TAG, "  \033[35mcontent\033[0m = \033[1;37m<none>\033[0m");
            break;

        default:
            ESP_LOGW(TAG, "  content = <unknown message type>");
            break;
    }
    ESP_LOGI(TAG, "");
    return ESP_OK;
}


esp_err_t spi_slave_init(Spi *spi) {

    ESP_ERROR_CHECK(spi_slave_initialize(spi->host, &(spi->buscfg), &(spi->slvcfg), spi->dma_chan));

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

    // Set relay output pin
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

    ESP_LOGI(TAG, "spi_slave_init(); SPI initialized successfully");
    return ESP_OK;
}


esp_err_t spi_slave_rx(Spi *spi) {
    memset(spi->recvbuf, 0, spi->bufsize);
    
    // Queue transaction
    esp_err_t return_code = spi_slave_transmit(spi->host, &(spi->transaction), portMAX_DELAY);
    if (return_code != ESP_OK) {
        ESP_LOGE(TAG, "spi_slave_rx(); Failed to queue SPI transaction: %s", esp_err_to_name(return_code));
        return return_code;
    }

    //Clear send buffer after transaction
    memset(spi->sendbuf, 0, spi->bufsize);
    ESP_LOGV(TAG, "spi_slave_rx(); SPI transmit successful");
    return ESP_OK;
}


esp_err_t spi_slave_tx(Spi *spi) {
    //Prepare retun_code
    esp_err_t return_code = ESP_OK;

    // Clear RX buffer
    memset(spi->recvbuf, 0, spi->bufsize);

    // Queue transaction
    return_code = spi_slave_queue_trans(spi->host, &(spi->transaction), portMAX_DELAY);
    if (return_code != ESP_OK) {
        ESP_LOGE(TAG, "spi_slave_tx(); Failed to queue SPI transaction: %s", esp_err_to_name(return_code));
        return return_code;
    }

    // REQ to master + Wait for transaction result
    gpio_set_level(spi->req_pin, true);
    return_code = spi_slave_get_trans_result(spi->host, &(spi->transaction_result), portMAX_DELAY);
    if (return_code != ESP_OK) {
        ESP_LOGE(TAG, "spi_slave_tx(); Failed to get SPI transaction result: %s", esp_err_to_name(return_code));
        return return_code;
    }
    gpio_set_level(spi->req_pin, false);
    
    //Clear send buffer after transaction
    memset(spi->sendbuf, 0, spi->bufsize);
    ESP_LOGV(TAG, "spi_slave_tx(); SPI transmit successful");
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
    if (!message->content) {
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
    if (spi->bufsize < 9) {
        ESP_LOGE(TAG, "spi_encode_message(); ESP_ERR_INVALID_SIZE, SPI buffer size too small to contain a valid message");
        return ESP_ERR_INVALID_SIZE;
    }
    
    switch (message->type) {
        //--- Master to Slave --- (All illegale)
        case MSG_MASTER_ACK:
            message->length = -1;
            break;

        case MSG_MASTER_ERR:
            message->length = -1;
            break;

        case MSG_SET_REG_ASCII:
            message->length = -1;
            break;

        case MSG_SET_REG_U16:
            message->length = -1;
            break;

        case MSG_SET_REG_F32:
            message->length = -1;
            break;

        case MSG_PING:
            message->length = -1;
            break;

        case MSG_GET_REG:
            message->length = -1;
            break;

        //--- Slave to Master ---
        case MSG_SLAVE_ACK:
            break;

        case MSG_SLAVE_ERR:
            break;

        case MSG_REP_ASCII:
            break;

        case MSG_REP_U16:
            break;

        case MSG_REP_F32:
            break;

        default:
            message->length = -2;
            break;
    }

    if (message->length == -1) {
        ESP_LOGE(TAG, "spi_encode_message(); ESP_ERR_NOT_ALLOWED, SPI message of type %02X illegal", message->type);
        return ESP_ERR_NOT_ALLOWED;
    } else if (message->length == -2) {
        ESP_LOGE(TAG, "spi_encode_message(); ESP_ERR_NOT_ALLOWED, SPI message of type %02X does not exist", message->type);
        return ESP_ERR_NOT_ALLOWED;
    }

    if (message->length > spi->bufsize - 9) {
        ESP_LOGE(TAG, "spi_encode_message(); ESP_ERR_INVALID_SIZE, SPI buffer size too small to contain the message of lenth %d", message->length);
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Clear sendbuf
    memset(spi->sendbuf, 0, spi->bufsize);

    // Add start byte
    spi->sendbuf[0] = 0x55;
    
    // Copy MsgType
    spi->sendbuf[1] = message->type;
    
    // Copy register number (uint16, big-endian)
    spi->sendbuf[2] = (message->reg >> 8) & 0x00FF;     // high byte
    spi->sendbuf[3] = message->reg & 0x00FF;            // low byte
    
    // Copy message length (uint16, big-endian)
    spi->sendbuf[4] = (message->length >> 8) & 0x00FF;  // high byte
    spi->sendbuf[5] = message->length & 0x00FF;         // low byte

    // Copy message content (here the null char at the end will not be copied!)
    memcpy(spi->sendbuf + 6, message->content, message->length);

    // Calculate CRC [CRC-16/ARC over [MsgType] + [Register] + [ContentSize] + [Content], Polynomial: 0x8005, Initial value: 0x0000]
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

    // Copy CRC (big-endian)
    spi->sendbuf[6 + message->length] = (computed_checksum >> 8) & 0x00FF;
    spi->sendbuf[7 + message->length] = computed_checksum & 0x00FF;

    // Add end byte
    spi->sendbuf[8 + message->length] = 0xAA;
    
    return ESP_OK;
}
