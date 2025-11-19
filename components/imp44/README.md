# IMP44 Microphone Component

ESP-IDF component for the IMP441 MEMS microphone using I2S interface.

## Features

- I2S interface support
- Configurable sample rate (8kHz to 96kHz)
- Configurable bit depth (16, 24, 32 bits)
- Simple API for audio capture
- DMA-based data transfer

## Hardware Connections

| IMP441 Pin | ESP32 Pin | Description |
|------------|-----------|-------------|
| VDD        | 3.3V      | Power supply |
| GND        | GND       | Ground |
| SD         | GPIO (configurable) | Serial Data |
| WS         | GPIO (configurable) | Word Select (LRCK) |
| SCK        | GPIO (configurable) | Serial Clock (BCK) |
| L/R        | GND or 3.3V | Channel select (GND=Left, 3.3V=Right) |

## Usage Example

```c
#include "imp44.h"
#include "esp_log.h"

void app_main(void)
{
    // Configure microphone
    imp44_config_t mic_config = {
        .bck_io_num = 14,
        .ws_io_num = 15,
        .data_in_num = 32,
        .sample_rate = 16000,
        .bits_per_sample = 32
    };

    // Initialize microphone
    imp44_handle_t mic_handle = {0};
    esp_err_t ret = imp44_init(&mic_handle, &mic_config);
    if (ret != ESP_OK) {
        ESP_LOGE("APP", "Failed to initialize microphone");
        return;
    }

    // Start recording
    imp44_start(&mic_handle);

    // Read audio data
    int32_t audio_buffer[1024];
    size_t bytes_read;
    
    while (1) {
        ret = imp44_read(&mic_handle, audio_buffer, sizeof(audio_buffer), &bytes_read, 1000);
        if (ret == ESP_OK) {
            // Process audio data
            ESP_LOGI("APP", "Read %d bytes", bytes_read);
        }
    }

    // Stop and cleanup
    imp44_stop(&mic_handle);
    imp44_deinit(&mic_handle);
}
```

## API Reference

### Functions

- `imp44_init()` - Initialize the microphone with configuration
- `imp44_deinit()` - Deinitialize and free resources
- `imp44_start()` - Start audio capture
- `imp44_stop()` - Stop audio capture
- `imp44_read()` - Read audio data from microphone

### Configuration

The `imp44_config_t` structure contains:
- `bck_io_num`: GPIO for bit clock
- `ws_io_num`: GPIO for word select
- `data_in_num`: GPIO for data input
- `sample_rate`: Sample rate in Hz (e.g., 16000, 44100, 48000)
- `bits_per_sample`: Bits per sample (16, 24, or 32)

## Notes

- The component uses ESP-IDF's I2S driver in standard mode
- DMA is automatically configured for efficient data transfer
- Recommended sample rates: 16kHz (voice), 44.1kHz or 48kHz (audio)
- Use 32-bit samples for best quality with IMP441
