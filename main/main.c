/*
 * IMP44 Microphone Spectrum Analyzer with 7-Segment Display
 * Real-time FFT audio spectrum analyzer displayed on 7-segment LEDs
 * Each segment represents a different frequency band
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_dsp.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include "imp441.h"

static const char *TAG = "SPECTRUM";

// GPIO Configuration for IMP441 microphone
#define I2S_BCK_IO      6
#define I2S_WS_IO       5
#define I2S_DATA_IN_IO  4

// LED Strip Configuration (7-segment display)
#define RMT_LED_STRIP_GPIO_NUM  8
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000
#define EXAMPLE_LED_NUMBERS     88  // Total LEDs (1 + 3 digits x 29 LEDs)

// 7-Segment Display Configuration
#define SEGMENTS_PER_DIGIT 8    // 7 segments + decimal point
#define DIGITS 3                // 3 digits
#define LEDS_PER_DIGIT 29       // LEDs per digit
#define LED_OFFSET 1            // First LED is offset by 1
#define NUM_BANDS 24            // 24 bands (8 segments × 3 digits, using all segments including dp)

// Segment mapping: a, b, c, d, e, f, g, dp
static const int seg_offset[SEGMENTS_PER_DIGIT] = {12, 16, 24, 0, 4, 8, 20, 28};
static const int seg_length[SEGMENTS_PER_DIGIT] = {4, 4, 4, 4, 4, 4, 4, 1};

// Audio Configuration
#define SAMPLE_RATE     16000
#define BITS_PER_SAMPLE 32
#define FFT_SIZE        512
#define READ_TIMEOUT_MS 1000
#define UPDATE_RATE_MS  30

// Buffers
static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];
static float fft_input[FFT_SIZE];
static float fft_output[FFT_SIZE];
static float fft_window[FFT_SIZE];
static float magnitude[FFT_SIZE / 2];
static float band_values[NUM_BANDS];
static float band_peak[NUM_BANDS];
static uint32_t peak_hold_time[NUM_BANDS];

// RMT handle
static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;

/**
 * @brief Convert HSV to RGB color
 */
static void hsv_to_rgb(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    
    float r1, g1, b1;
    
    if (h >= 0 && h < 60) {
        r1 = c; g1 = x; b1 = 0;
    } else if (h >= 60 && h < 120) {
        r1 = x; g1 = c; b1 = 0;
    } else if (h >= 120 && h < 180) {
        r1 = 0; g1 = c; b1 = x;
    } else if (h >= 180 && h < 240) {
        r1 = 0; g1 = x; b1 = c;
    } else if (h >= 240 && h < 300) {
        r1 = x; g1 = 0; b1 = c;
    } else {
        r1 = c; g1 = 0; b1 = x;
    }
    
    *r = (uint8_t)((r1 + m) * 255.0f);
    *g = (uint8_t)((g1 + m) * 255.0f);
    *b = (uint8_t)((b1 + m) * 255.0f);
}

static void init_window(void)
{
    for (int i = 0; i < FFT_SIZE; i++) {
        fft_window[i] = 0.5f - 0.5f * cosf(2.0f * M_PI * i / (FFT_SIZE - 1));
    }
}

/**
 * @brief Apply window function to input data
 */
static void apply_window(float *data, int size)
{
    for (int i = 0; i < size; i++) {
        data[i] *= fft_window[i];
    }
}

/**
 * @brief Calculate frequency bands from FFT magnitude
 */
static void calculate_bands(float *mag, float *bands, int fft_size, int num_bands)
{
    memset(bands, 0, num_bands * sizeof(float));
    
    // Skip very low frequencies (below ~50Hz) to reduce always-on bass
    int start_bin = 2;  // Start from bin 2 instead of 0
    int usable_bins = (fft_size / 2) - start_bin;
    
    // Logarithmic frequency distribution
    for (int i = 0; i < num_bands; i++) {
        float start_freq = powf(2.0f, (float)i * logf((float)usable_bins) / logf(2.0f) / (float)num_bands);
        float end_freq = powf(2.0f, (float)(i + 1) * logf((float)usable_bins) / logf(2.0f) / (float)num_bands);
        
        int start_bin_adj = start_bin + (int)start_freq;
        int end_bin_adj = start_bin + (int)end_freq;
        
        if (end_bin_adj > fft_size / 2) end_bin_adj = fft_size / 2;
        if (start_bin_adj >= end_bin_adj) start_bin_adj = end_bin_adj - 1;
        if (start_bin_adj < start_bin) start_bin_adj = start_bin;
        
        // Average magnitude in this band
        float sum = 0;
        int count = 0;
        for (int j = start_bin_adj; j < end_bin_adj; j++) {
            sum += mag[j];
            count++;
        }
        
        if (count > 0) {
            bands[i] = sum / count;
        }
    }
}

/**
 * @brief Update 7-segment display with spectrum data
 * Each segment shows a different frequency band
 */
static void update_7seg_spectrum(float *bands, int num_bands)
{
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Find max for normalization and detect silence
    float max_val = 0;
    float total_energy = 0;
    for (int i = 0; i < num_bands; i++) {
        if (bands[i] > max_val) max_val = bands[i];
        total_energy += bands[i];
    }
    
    // Silence detection - if total energy is very low, turn off LEDs
    float silence_threshold = 0.1f;  // Increased threshold for better silence detection
    if (total_energy < silence_threshold || max_val < 0.01f) {
        // No sound detected - clear all LEDs
        memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
        
        // Reset peak hold values
        memset(band_peak, 0, sizeof(band_peak));
        memset(peak_hold_time, 0, sizeof(peak_hold_time));
        return;
    }
    
    if (max_val < 1e-6f) max_val = 1.0f;
    
    // Clear all pixels
    memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
    
    // Map bands to segments (8 segments x 3 digits = 24 bands, including decimal points)
    for (int digit = 0; digit < DIGITS; digit++) {
        int digit_base = LED_OFFSET + (digit * LEDS_PER_DIGIT);  // Add offset for first LED
        
        for (int seg = 0; seg < SEGMENTS_PER_DIGIT; seg++) {  // Use all 8 segments including dp
            int band_idx = digit * SEGMENTS_PER_DIGIT + seg;
            if (band_idx >= num_bands) continue;
            
            // Normalize band value
            float normalized = bands[band_idx] / max_val;
            
            // Apply minimum threshold to avoid noise - increased threshold
            if (normalized < 0.15f) {
                normalized = 0;
            }
            
            // Update peak hold
            if (normalized > band_peak[band_idx]) {
                band_peak[band_idx] = normalized;
                peak_hold_time[band_idx] = current_time;
            } else if (current_time - peak_hold_time[band_idx] > 500) {
                band_peak[band_idx] *= 0.95f;
            }
            
            // Use current value for brightness
            float brightness = normalized;
            
            // Skip if brightness is too low
            if (brightness < 0.15f) continue;
            
            // Optimized color scheme for visual appeal
            // Smooth progression through vibrant colors
            float hue;
            float position = (float)band_idx / (float)(num_bands - 1);
            
            if (position < 0.33f) {
                // Bass: Deep blue to cyan (240° to 180°)
                hue = 240.0f - (position / 0.33f) * 60.0f;
            } else if (position < 0.66f) {
                // Mid: Cyan to green to yellow (180° to 60°)
                hue = 180.0f - ((position - 0.33f) / 0.33f) * 120.0f;
            } else {
                // Treble: Yellow to orange to red (60° to 0°)
                hue = 60.0f - ((position - 0.66f) / 0.34f) * 60.0f;
            }
            
            uint8_t r, g, b;
            hsv_to_rgb(hue, 1.0f, brightness, &r, &g, &b);            // Light up the segment LEDs
            int seg_start = digit_base + seg_offset[seg];
            int seg_len = seg_length[seg];
            
            for (int led = 0; led < seg_len; led++) {
                int led_index = seg_start + led;
                if (led_index >= 0 && led_index < EXAMPLE_LED_NUMBERS) {
                    int pix = led_index * 3;
                    led_strip_pixels[pix + 0] = g;  // WS2812 is GRB order
                    led_strip_pixels[pix + 1] = r;
                    led_strip_pixels[pix + 2] = b;
                }
            }
        }
    }
}

/**
 * @brief Initialize RMT for LED strip control
 */
static esp_err_t init_led_strip(void)
{
    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    esp_err_t ret = rmt_new_tx_channel(&tx_chan_config, &led_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Install LED strip encoder");
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ret = rmt_new_led_strip_encoder(&encoder_config, &led_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create encoder: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ret = rmt_enable(led_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Clear all LEDs
    memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
    rmt_transmit_config_t tx_config = {.loop_count = 0};
    rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config);
    rmt_tx_wait_all_done(led_chan, portMAX_DELAY);
    
    ESP_LOGI(TAG, "LED strip initialized on GPIO %d with %d LEDs", RMT_LED_STRIP_GPIO_NUM, EXAMPLE_LED_NUMBERS);
    return ESP_OK;
}

/**
 * @brief Audio processing and FFT analysis task
 */
static void spectrum_task(void *pvParameters)
{
    imp441_handle_t *mic_handle = (imp441_handle_t *)pvParameters;
    int32_t audio_buffer[FFT_SIZE];
    size_t bytes_read;
    
    ESP_LOGI(TAG, "Spectrum analyzer task started");
    
    // Initialize FFT window
    init_window();
    
    // Initialize DSP library
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, FFT_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize FFT: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    while (1) {
        // Read audio data from microphone
        ret = imp441_read(mic_handle, audio_buffer, sizeof(audio_buffer), &bytes_read, READ_TIMEOUT_MS);
        
        if (ret == ESP_OK) {
            size_t samples = bytes_read / sizeof(int32_t);
            
            if (samples >= FFT_SIZE) {
                // Convert int32 to float and normalize
                for (int i = 0; i < FFT_SIZE; i++) {
                    fft_input[i] = (float)audio_buffer[i] / (float)INT32_MAX;
                }
                
                // Apply window function
                apply_window(fft_input, FFT_SIZE);
                
                // Prepare complex input (real, imag, real, imag, ...)
                for (int i = 0; i < FFT_SIZE; i++) {
                    fft_output[i * 2] = fft_input[i];
                    fft_output[i * 2 + 1] = 0; // Imaginary part
                }
                
                // Perform FFT
                dsps_fft2r_fc32(fft_output, FFT_SIZE);
                dsps_bit_rev_fc32(fft_output, FFT_SIZE);
                
                // Calculate magnitude spectrum
                for (int i = 0; i < FFT_SIZE / 2; i++) {
                    float re = fft_output[i * 2];
                    float im = fft_output[i * 2 + 1];
                    magnitude[i] = sqrtf(re * re + im * im);
                }
                
                // Calculate frequency bands
                calculate_bands(magnitude, band_values, FFT_SIZE, NUM_BANDS);
                
                // Update 7-segment display
                update_7seg_spectrum(band_values, NUM_BANDS);
                
                // Transmit to LEDs
                rmt_transmit_config_t tx_config = {.loop_count = 0};
                rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config);
                rmt_tx_wait_all_done(led_chan, portMAX_DELAY);
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Read timeout");
        } else {
            ESP_LOGE(TAG, "Read error: %s", esp_err_to_name(ret));
        }
        
        vTaskDelay(pdMS_TO_TICKS(UPDATE_RATE_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "7-Segment Spectrum Analyzer Starting...");
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());

    // Small delay to allow system to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize LED strip (7-segment display)
    esp_err_t ret = init_led_strip();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LED strip");
        return;
    }

    ESP_LOGI(TAG, "7-Segment Display: %d LEDs, %d digits, %d bands", 
             EXAMPLE_LED_NUMBERS, DIGITS, NUM_BANDS);

    // Configure microphone
    imp441_config_t mic_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_in_num = I2S_DATA_IN_IO,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = 32
    };

    // Initialize microphone
    static imp441_handle_t mic_handle = {0};
    ret = imp441_init(&mic_handle, &mic_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize IMP441: %s", esp_err_to_name(ret));
        return;
    }

    // Small delay after init
    vTaskDelay(pdMS_TO_TICKS(100));

    // Start microphone
    ret = imp441_start(&mic_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start IMP441: %s", esp_err_to_name(ret));
        imp441_deinit(&mic_handle);
        return;
    }

    ESP_LOGI(TAG, "Microphone started successfully!");
    ESP_LOGI(TAG, "Starting spectrum analyzer on 7-segment display...");

    // Create spectrum analyzer task
    BaseType_t task_ret = xTaskCreate(spectrum_task, "spectrum_task", 16384, &mic_handle, 5, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create spectrum task");
        imp441_stop(&mic_handle);
        imp441_deinit(&mic_handle);
        return;
    }

    // Main loop - monitor system health
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    }
}
