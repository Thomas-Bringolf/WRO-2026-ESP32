/**
 * @file SPI.h
 * @brief SPI slave communication library for structured message exchange.
 * @note One 256 Bytes message takes approx 3.3ms to be transmitted at 1 MHz
 * This library provides a complete SPI slave-side framework for reliable,
 * bidirectional communication with an SPI master using a custom message
 * protocol. It wraps the ESP-IDF SPI slave driver and adds higher-level
 * features such as:
 *
 * - A fixed message format with headers, payload, and CRC integrity checking
 * - Typed messages (ACK, errors, register access, ASCII, numeric data)
 * - Automatic encoding and decoding of messages
 * - Endianness-safe transmission of integers and floating-point values
 * - DMA-backed transmit and receive buffers
 * - Optional REQ GPIO signaling to synchronize with the SPI master
 *
 * The library is designed for embedded systems where the SPI slave exposes
 * registers or services to a master device and needs clear message semantics,
 * error detection, and predictable blocking behavior.
 *
 * It does not implement application logic itself, but instead offers a robust
 * transport layer on top of SPI for higher-level protocols.
 *  - ChatGPT
 */

#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include "driver/spi_slave.h"


/**
 * @struct Spi
 * @brief SPI slave device context and runtime state.
 *
 * This structure groups all configuration parameters, buffers, and
 * runtime state required to operate an SPI peripheral in **slave mode**
 * using the ESP-IDF SPI slave driver.
 *
 * It encapsulates:
 * - Hardware configuration (SPI host, bus, slave interface, DMA)
 * - Pre-allocated DMA-capable transmit and receive buffers
 * - The active SPI transaction and its result
 * - Optional GPIO-based request (REQ) signaling to the SPI master
 *
 * A single Spi instance represents one SPI slave interface and is intended
 * to be initialized once and reused for all subsequent SPI transactions.
 */
typedef struct {
    spi_host_device_t host;                 /**< SPI host device identifier */
    spi_bus_config_t buscfg;                /**< SPI bus configuration */
    spi_slave_interface_config_t slvcfg;    /**< SPI slave interface configuration */
    spi_slave_transaction_t transaction;    /**< Current SPI transaction descriptor */
    spi_slave_transaction_t *transaction_result; /**< Pointer to last completed transaction */
    spi_dma_chan_t dma_chan;                /**< DMA channel used by SPI */
    size_t bufsize;                         /**< Size of transmit, receive, and DMA buffers */
    char *recvbuf;                          /**< DMA-capable receive buffer */
    char *sendbuf;                          /**< DMA-capable transmit buffer */
    int req_pin;                            /**< GPIO pin for REQ (slave-to-master notification) */
} Spi;

/**
 * @enum MsgType
 * @brief Message type identifiers for the SPI communication protocol.
 *
 * Defines all valid message types exchanged between SPI master and slave.
 * The values encode both the direction and semantic meaning of a message.
 *
 * Message ranges are split by direction:
 * - 0x01–0x0F: Master → Slave messages
 * - 0x11–0x1F: Slave → Master messages
 *
 * Any message received or transmitted with a type outside this list is
 * considered invalid by the protocol.
 */
typedef enum {
    /* --- Master → Slave (not sent by slave) --- */
    MSG_MASTER_ACK        = 0x01, /**< Master acknowledge */
    MSG_MASTER_ERR        = 0x02, /**< Master error message (ASCII) */
    MSG_SET_REG_ASCII     = 0x03, /**< Write ASCII data to a register */
    MSG_SET_REG_U16       = 0x04, /**< Write uint16 data to a register */
    MSG_SET_REG_F32       = 0x05, /**< Write float32 data to a register */
    MSG_GET_REG           = 0x06, /**< Request register value */
    MSG_PING              = 0x07, /**< Liveness check ("ping") */

    /* --- Slave → Master --- */
    MSG_SLAVE_ACK         = 0x11, /**< Slave acknowledge */
    MSG_SLAVE_ERR         = 0x12, /**< Slave error message (ASCII) */
    MSG_REP_ASCII         = 0x13, /**< ASCII register reply */
    MSG_REP_U16           = 0x14, /**< uint16 register reply */
    MSG_REP_F32           = 0x15  /**< float32 register reply */
} MsgType;

/**
 * @struct SpiMessage
 * @brief Decoded or encoded SPI protocol message.
 *
 * Represents a single logical message exchanged over SPI, independent
 * of the underlying byte-level framing.
 *
 * The structure contains both metadata (type, register, length, checksum)
 * and a dynamically allocated payload buffer. The payload is not
 * null-terminated and may contain binary or ASCII data depending on
 * the message type.
 *
 * Memory ownership:
 * - The `content` buffer is managed dynamically and must be freed by
 *   the caller when no longer needed.
 */
typedef struct {
    MsgType  type;        /**< Message type identifier */
    uint16_t reg;         /**< Target register index or identifier */
    int16_t  length;      /**< Payload length in bytes */
    uint16_t checksum;    /**< CRC-16 checksum of the message */
    char    *content;     /**< Pointer to payload (binary or ASCII) */
} SpiMessage;

/**
 * @brief Checks if a buffer contains only zero bytes.
 *
 * Iterates through the buffer and returns true if all bytes are zero,
 * false otherwise.
 *
 * @param buf Pointer to the buffer to check.
 * @param size Size of the buffer in bytes.
 * @return true if all bytes are 0, false otherwise.
 */
bool is_buf_empty(char *buf, size_t size);

/**
 * @brief Logs the first N bytes of a buffer in hexadecimal format.
 *
 * Useful for debugging SPI messages or other binary data.
 *
 * @param buf Pointer to the buffer to log.
 * @param len Number of bytes from the buffer to print.
 */
void log_first_bytes_hex(char *buf, int len);

/**
 * @brief Convert a MsgType enum value to its corresponding string name.
 *
 * This function returns a human-readable string representing the symbolic
 * name of a message type (e.g., `"MSG_REP_ASCII"`).  
 * The returned string is a pointer to a constant, statically stored
 * string literal. It must not be modified or freed by the caller.
 *
 * @param type  The message type value to convert.
 *
 * @return A constant null-terminated string containing the symbolic name
 *         of the message type. Returns `"UNKNOWN_MSG_TYPE"` if the value
 *         does not match any known MsgType entry.
 */
const char* MsgType_toString(const MsgType type);

/**
 * @brief Convert the content of a SpiMessage to a null-terminated ASCII string.
 *
 * This function copies the raw content of a SpiMessage into a newly allocated
 * buffer and appends a null terminator. It is intended for ASCII/text messages
 * (e.g., MSG_REP_ASCII). The caller is responsible for freeing the returned string.
 *
 * @param message  Pointer to the SpiMessage to convert.
 *
 * @return Pointer to a newly allocated null-terminated string containing the
 *         message content, or NULL if the message is invalid or empty.
 */
char* message_toString(const SpiMessage *message);

/**
 * @brief Extract a 16-bit unsigned integer from a SpiMessage.
 *
 * This function interprets the first two bytes of the message content
 * as a big-endian unsigned 16-bit integer. It checks that the message
 * is valid and that the length is exactly 2 bytes.
 *
 * @param message  Pointer to the SpiMessage to extract from.
 *
 * @return The decoded uint16_t value. Returns 0 if the message is invalid.
 *
 * @note This function assumes the message content is encoded in big-endian
 *       (MSB first) order. Use caution if your protocol uses a different endianness.
 */
uint16_t message_toUint16(const SpiMessage *message);

/**
 * @brief Extract a 32-bit floating point value from a SpiMessage.
 *
 * This function interprets the first four bytes of the message content
 * as a big-endian IEEE 754 float. It checks that the message is valid
 * and that the length is exactly 4 bytes.
 *
 * @param message  Pointer to the SpiMessage to extract from.
 *
 * @return The decoded float value. Returns 0.0f if the message is invalid.
 *
 * @note Endianness is handled explicitly: the function expects big-endian
 *       byte order in the message buffer. Conversion is performed on
 *       little-endian or big-endian MCUs automatically.
 */
float message_toFloat32(const SpiMessage *message);

/**
 * @brief Set the content of a SpiMessage from a string (ASCII).
 * @param message Pointer to the SpiMessage to modify
 * @param str Null-terminated string to set
 * @return ESP_OK on success, ESP_ERR_NO_MEM if allocation failed
 *
 * @note The function allocates memory for message->content and frees
 *       any previously allocated content.
 */
esp_err_t message_setString(SpiMessage *message, const char *str);

/**
 * @brief Set the content of a SpiMessage from a 16-bit unsigned integer.
 *
 * The integer is encoded in **big-endian** (MSB first) format into
 * a newly allocated 2-byte buffer. Any previously allocated content
 * in the message will be freed. The message length is set to 2.
 *
 * @param message Pointer to the SpiMessage to modify.
 * @param value 16-bit unsigned integer to set as message content.
 *
 * @return
 * - ESP_OK on success
 * - ESP_ERR_INVALID_ARG if message is NULL
 * - ESP_ERR_NO_MEM if memory allocation fails
 */
esp_err_t message_setUint16(SpiMessage *message, uint16_t value);

/**
 * @brief Set the content of a SpiMessage from a 32-bit floating point value.
 *
 * The float is converted to IEEE 754 format and stored in a **big-endian**
 * 4-byte buffer. Any previously allocated content in the message will be freed.
 * The message length is set to 4.
 *
 * @param message Pointer to the SpiMessage to modify.
 * @param value 32-bit floating point value to set as message content.
 *
 * @return
 * - ESP_OK on success
 * - ESP_ERR_INVALID_ARG if message is NULL
 * - ESP_ERR_NO_MEM if memory allocation fails
 *
 * @note Endianness is explicitly handled for cross-platform SPI communication.
 */
esp_err_t message_setFloat32(SpiMessage *message, float value);

/**
 * @brief Log a decoded SpiMessage in a human-readable format.
 *
 * @param message Pointer to the message to log
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if message is NULL
 */
esp_err_t message_log(const SpiMessage *message);

/**
 * @brief Initializes the SPI slave interface.
 *
 * Allocates DMA-aligned receive and send buffers, configures GPIO pins,
 * and prepares the SPI transaction structure.
 *
 * @param spi Pointer to the SPI device structure to initialize.
 */
esp_err_t spi_slave_init(Spi *spi);

/**
 * @brief Performs a blocking SPI read (receive-only) transaction.
 *
 * Receives data from the SPI master and stores it in spi->recvbuf.
 * Clears the send buffer after the transaction.
 *
 * @param spi Pointer to the SPI device structure.
 */
esp_err_t spi_slave_rx(Spi *spi);

/**
 * @brief Performs a blocking SPI slave write (transmit-only) transaction.
 *
 * Sends data from spi->sendbuf to the SPI master.
 * Toggles the REQ pin to notify the master and waits for the transaction to complete.
 * Clears the send buffer after the transaction.
 * If any data is received during the transaction, sets status to SPI_TX_FAILED.
 *
 * @param spi Pointer to the SPI device structure.
 */
esp_err_t spi_slave_tx(Spi *spi);

/**
 * @brief Decodes a SPI message received from the master.
 *
 * This function parses the SPI receive buffer, extracts the message fields,
 * verifies the start and end bytes, allocates memory for the content,
 * and checks the CRC-16/ARC checksum.
 *
 * @note The `content` pointer inside `SpiMessage` must be initialized to NULL before
 *       calling this function. The function will call `free()` on it if non-NULL. Failing
 *       to do so may result in undefined behavior or a crash.
 *
 * @param spi Pointer to the SPI device structure containing the receive buffer and buffer size.
 * @param message Pointer to a SpiMessage structure where the decoded message will be stored (not null-terminated!).
 *                The function allocates memory for message->content.
 *
 * @return
 *      - ESP_OK: Message decoded successfully.
 *      - ESP_ERR_INVALID_SIZE: Buffer too small for the expected message length.
 *      - ESP_ERR_INVALID_CRC: Checksum mismatch detected.
 *      - ESP_FAIL: Invalid start or end byte.
 *      - ESP_ERR_NO_MEM: Memory allocation for message content failed.
 */
esp_err_t spi_decode_message(Spi *spi, SpiMessage *message);

/**
 * @brief Encodes a SPI message to be sent to the slave.
 * 
 * This function prepares the SPI send buffer, fills in the start byte, message type,
 * register, message length, content, calculates the CRC-16/ARC checksum, and appends the end byte.
 * It also verifies that the message type is allowed and that the buffer is large enough.
 * 
 * @param spi Pointer to the SPI device structure containing the send buffer and buffer size.
 * @param message Pointer to the SpiMessage structure containing the data to encode.
 *                message->content must be valid if message length > 0.
 *
 * @return
 *      - ESP_OK: Message encoded successfully.
 *      - ESP_ERR_INVALID_SIZE: SPI buffer too small for the encoded message.
 *      - ESP_ERR_NOT_ALLOWED: Message type is illegal or unknown.
 */
esp_err_t spi_encode_message(Spi *spi, SpiMessage *message);

/**
 * @brief Receives an SPI message.
 *
 * This function waits for a message from the SPI slave, decodes it,
 * logs the received message, and reports any decoding errors.
 *
 * @param[in] spi Pointer to the SPI interface structure.
 * @param[out] message Pointer to the message structure to store the received message.
 *
 * @return
 * - ESP_OK on successful reception and decoding.
 * - ESP_FAIL if decoding fails.
 */
esp_err_t spi_recive_message(Spi *spi, SpiMessage *message);

/**
 * @brief Sends an SPI message.
 *
 * This function logs the message, encodes it for SPI transmission,
 * sends it to the SPI slave, and then clears the message content.
 *
 * @param[in] spi Pointer to the SPI interface structure.
 * @param[in,out] message Pointer to the message structure to send.
 *
 * @return
 * - ESP_OK on success.
 */
esp_err_t spi_send_message(Spi *spi, SpiMessage *message);

#endif // SPI_H