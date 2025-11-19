/*
 * IMP44 MEMS Microphone Component
 * I2S interface driver for IMP441 microphone
 */

#ifndef IMP44_H
#define IMP44_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2s_std.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief IMP44 microphone configuration structure
 */
typedef struct {
    int bck_io_num;        /*!< BCK (bit clock) GPIO number */
    int ws_io_num;         /*!< WS (word select/LRCK) GPIO number */
    int data_in_num;       /*!< Data input GPIO number */
    uint32_t sample_rate;  /*!< Sample rate in Hz (e.g., 44100, 48000) */
    uint8_t bits_per_sample; /*!< Bits per sample (16, 24, 32) */
} imp44_config_t;

/**
 * @brief IMP44 microphone handle
 */
typedef struct {
    i2s_chan_handle_t rx_handle;
    imp44_config_t config;
    bool is_initialized;
} imp44_handle_t;

/**
 * @brief Initialize IMP44 microphone
 * 
 * @param handle Pointer to IMP44 handle
 * @param config Pointer to configuration structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t imp44_init(imp44_handle_t *handle, const imp44_config_t *config);

/**
 * @brief Deinitialize IMP44 microphone
 * 
 * @param handle Pointer to IMP44 handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t imp44_deinit(imp44_handle_t *handle);

/**
 * @brief Start microphone recording
 * 
 * @param handle Pointer to IMP44 handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t imp44_start(imp44_handle_t *handle);

/**
 * @brief Stop microphone recording
 * 
 * @param handle Pointer to IMP44 handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t imp44_stop(imp44_handle_t *handle);

/**
 * @brief Read audio data from microphone
 * 
 * @param handle Pointer to IMP44 handle
 * @param dest Destination buffer for audio data
 * @param size Size of data to read in bytes
 * @param bytes_read Pointer to store actual bytes read
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t imp44_read(imp44_handle_t *handle, void *dest, size_t size, size_t *bytes_read, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // IMP44_H
