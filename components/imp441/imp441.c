/*
 * IMP441 MEMS Microphone Component Implementation
 * I2S interface driver for IMP441 microphone
 */

#include "imp441.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "IMP441";

esp_err_t imp441_init(imp441_handle_t *handle, const imp441_config_t *config)
{
    if (handle == NULL || config == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->is_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Clear handle structure
    memset(handle, 0, sizeof(imp441_handle_t));
    
    // Store configuration
    memcpy(&handle->config, config, sizeof(imp441_config_t));

    // Configure I2S channel
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    
    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &handle->rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure I2S standard mode
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config->sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = config->bck_io_num,
            .ws = config->ws_io_num,
            .dout = I2S_GPIO_UNUSED,
            .din = config->data_in_num,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    
    // Override slot configuration for proper alignment
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    ret = i2s_channel_init_std_mode(handle->rx_handle, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S standard mode: %s", esp_err_to_name(ret));
        i2s_del_channel(handle->rx_handle);
        return ret;
    }

    handle->is_initialized = true;
    ESP_LOGI(TAG, "IMP441 initialized successfully");
    ESP_LOGI(TAG, "Sample rate: %lu Hz, Bits: %d", config->sample_rate, config->bits_per_sample);
    ESP_LOGI(TAG, "GPIO - BCK: %d, WS: %d, DIN: %d", 
             config->bck_io_num, config->ws_io_num, config->data_in_num);

    return ESP_OK;
}

esp_err_t imp441_deinit(imp441_handle_t *handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Invalid handle");
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->is_initialized) {
        ESP_LOGW(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Stop channel if running (ignore error if already stopped)
    i2s_channel_disable(handle->rx_handle);
    
    // Delete channel
    esp_err_t ret = i2s_del_channel(handle->rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Clear handle
    memset(handle, 0, sizeof(imp441_handle_t));
    
    ESP_LOGI(TAG, "IMP441 deinitialized");

    return ESP_OK;
}

esp_err_t imp441_start(imp441_handle_t *handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Invalid handle");
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->is_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = i2s_channel_enable(handle->rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "IMP441 started");
    return ESP_OK;
}

esp_err_t imp441_stop(imp441_handle_t *handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Invalid handle");
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->is_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = i2s_channel_disable(handle->rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "IMP441 stopped");
    return ESP_OK;
}

esp_err_t imp441_read(imp441_handle_t *handle, void *dest, size_t size, size_t *bytes_read, uint32_t timeout_ms)
{
    if (handle == NULL || dest == NULL || bytes_read == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->is_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = i2s_channel_read(handle->rx_handle, dest, size, bytes_read, timeout_ms);
    if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "Failed to read I2S data: %s", esp_err_to_name(ret));
        return ret;
    }

    return ret;
}
