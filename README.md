# Audio Spectrum Analyzer for ESP32

Real-time FFT-based audio spectrum analyzer using IMP441 MEMS microphone and WS2812B LED display.

**Tested on:** 7-segment LED display (3 digits × 29 LEDs + 1 offset LED = 88 LEDs total)

## Features

- **Real-time FFT analysis** using ESP-DSP library (512-point FFT)
- **24 frequency bands** displayed on 7-segment LED display
- **Logarithmic frequency distribution** for better bass/treble resolution
- **Rainbow color gradient** from bass (blue) to treble (red)
- **Silence detection** - LEDs turn off when no audio is present
- **Peak hold** with automatic decay for smooth visual effect
- **Noise filtering** to eliminate background interference

## Hardware Requirements

### Microphone (IMP441)
- **VDD** → 3.3V
- **GND** → GND
- **SD** (Serial Data) → GPIO 4
- **WS** (Word Select/LRCK) → GPIO 5
- **SCK** (Serial Clock/BCK) → GPIO 6
- **L/R** → GND (left channel) or 3.3V (right channel)

### LED Display (WS2812B)
- **Data** → GPIO 8
- **VDD** → 5V (external power recommended for 88 LEDs)
- **GND** → GND

**Configuration:** 88 LEDs total
- 1 offset LED (index 0)
- 3 seven-segment digits (29 LEDs each)
- Each digit: 7 segments × 4 LEDs + 1 decimal point LED

## Audio Configuration

### Sampling & Processing
- **Sample Rate:** 16,000 Hz
- **Bit Depth:** 32-bit
- **FFT Size:** 512 samples
- **Update Rate:** 30ms (~33 FPS)
- **Frequency Range:** 62 Hz - 8,000 Hz

### Frequency Bands (24 bands total)

#### Digit 1 (Bands 0-7) - Bass/Low-Mid - Blue to Cyan
| Band | Segment | Frequency Range | Musical Content |
|------|---------|-----------------|-----------------|
| 0    | a       | ~62-80 Hz       | Sub-bass, kick drum |
| 1    | b       | ~80-100 Hz      | Bass guitar, bass drum |
| 2    | c       | ~100-130 Hz     | Bass fundamentals |
| 3    | d       | ~130-165 Hz     | Low male vocals |
| 4    | e       | ~165-210 Hz     | Low vocals, cello |
| 5    | f       | ~210-270 Hz     | Male vocals |
| 6    | g       | ~270-340 Hz     | Mid-range vocals |
| 7    | dp      | ~340-435 Hz     | Female vocals |

#### Digit 2 (Bands 8-15) - Mid Range - Green to Yellow
| Band | Segment | Frequency Range | Musical Content |
|------|---------|-----------------|-----------------|
| 8    | a       | ~435-560 Hz     | Vocals, guitars |
| 9    | b       | ~560-710 Hz     | Snare, guitars |
| 10   | c       | ~710-900 Hz     | Guitar harmonics |
| 11   | d       | ~900-1150 Hz    | Upper vocals |
| 12   | e       | ~1150-1450 Hz   | Presence range |
| 13   | f       | ~1450-1850 Hz   | Clarity range |
| 14   | g       | ~1850-2350 Hz   | Brightness |
| 15   | dp      | ~2350-3000 Hz   | High presence |

#### Digit 3 (Bands 16-23) - Treble - Orange to Red
| Band | Segment | Frequency Range | Musical Content |
|------|---------|-----------------|-----------------|
| 16   | a       | ~3000-3800 Hz   | Cymbals, hi-hats |
| 17   | b       | ~3800-4850 Hz   | Sibilance, brilliance |
| 18   | c       | ~4850-6200 Hz   | Air, sparkle |
| 19   | d       | ~6200-7900 Hz   | Very high treble |
| 20   | e       | ~7900-8000 Hz   | Upper limit |
| 21   | f       | ~7900-8000 Hz   | Upper limit |
| 22   | g       | ~7900-8000 Hz   | Upper limit |
| 23   | dp      | ~7900-8000 Hz   | Upper limit |

## Color Scheme

The display uses a smooth color gradient that transitions from cool to warm colors:

- **Bass (0-33%)**: Deep Blue → Cyan (240° → 180°)
- **Mid (33-66%)**: Cyan → Green → Yellow (180° → 60°)
- **Treble (66-100%)**: Yellow → Orange → Red (60° → 0°)

This creates an intuitive visual where:
- Cool blues indicate bass frequencies
- Vibrant greens/yellows show mid-range activity
- Warm reds highlight treble content

## Thresholds & Filtering

### Silence Detection
- **Total energy threshold:** 0.1
- **Maximum value threshold:** 0.01
- LEDs completely turn off when below threshold
- Peak hold values are reset during silence

### Noise Filtering
- **Minimum brightness threshold:** 15%
- Individual bands below 15% are suppressed
- Very low frequencies (0-62 Hz) are filtered out

### Peak Hold
- **Hold time:** 500ms
- **Decay rate:** 95% per update cycle
- Provides smooth visual decay after sound stops

## 7-Segment LED Mapping

Each digit uses the following LED mapping:
```
   a (12-15)
  ┌─────────┐
  │         │
f │         │ b (16-19)
(8-11)      │
  │    g    │
  ├─────────┤ (20-23)
  │         │
e │         │ c (24-27)
(4-7)       │
  │         │
  └─────────┘ • dp (28)
   d (0-3)
```

## Building and Flashing

```bash
# Navigate to project directory
cd c:\esp_projects\equalizer

# Configure (if needed)
idf.py menuconfig

# Build
idf.py build

# Flash to ESP32
idf.py -p COM<X> flash

# Monitor output
idf.py -p COM<X> monitor
```

## Dependencies

The project uses the following ESP-IDF components:
- **esp-dsp** v1.4.0+ (FFT processing)
- **driver** (RMT, I2S, GPIO)
- **freertos** (task management)
- **esp_system** (system functions)

Dependencies are automatically managed via `idf_component.yml`.

## Project Structure

```
equalizer/
├── main/
│   ├── main.c                    # Main application
│   ├── led_strip_encoder.c       # WS2812B RMT encoder
│   ├── led_strip_encoder.h       # Encoder header
│   ├── CMakeLists.txt            # Component build config
│   └── idf_component.yml         # Component dependencies
├── components/
│   └── imp441/                   # IMP441 microphone driver
│       ├── imp441.c              # Driver implementation
│       ├── imp441.h              # Public API
│       ├── CMakeLists.txt        # Component build config
│       └── README.md             # Component documentation
├── CMakeLists.txt                # Project build config
└── README.md                     # This file
```

## Memory Usage

- **FFT buffers:** ~4 KB (float arrays)
- **LED buffer:** 264 bytes (88 LEDs × 3 bytes)
- **Task stack:** 16 KB (spectrum analyzer task)
- **Free heap:** ~200 KB remaining (typical)

## Performance

- **Update rate:** 30ms (33 FPS)
- **Latency:** ~50ms (FFT processing + display)
- **CPU usage:** ~25% on single core
- **Response time:** Immediate for transients

## Customization

### Adjust Sensitivity
Edit `main.c`:
```c
// Increase for less sensitivity (higher threshold)
float silence_threshold = 0.1f;  // Line ~154

// Increase to filter more noise
if (normalized < 0.15f) // Line ~171
```

### Change Colors
Modify the color mapping in `update_7seg_spectrum()` function:
```c
// Example: Make all bands red
float hue = 0.0f;  // Red = 0°, Green = 120°, Blue = 240°
```

### Adjust Frequency Ranges
Modify `calculate_bands()`:
```c
// Change starting frequency bin
int start_bin = 2;  // Higher = skip more low frequencies
```

## Troubleshooting

### LEDs always on
- Increase silence threshold: `float silence_threshold = 0.5f;`
- Increase minimum brightness: `if (normalized < 0.25f)`

### No LEDs lighting up
- Decrease silence threshold: `float silence_threshold = 0.01f;`
- Check microphone wiring and I2S configuration
- Verify LED strip power supply and data connection

### Colors not smooth
- Check WS2812B timing (RMT encoder)
- Verify LED strip type (WS2812/WS2812B/SK6812)

### Poor frequency response
- Adjust FFT size (trade-off: resolution vs. latency)
- Modify `start_bin` to skip more/less low frequencies
- Change number of bands

## License

This project is provided as-is for educational and personal use.

## Credits

- ESP-IDF framework by Espressif Systems
- ESP-DSP library for FFT processing
- IMP441 MEMS microphone driver (custom implementation)
- WS2812B LED strip encoder (custom RMT implementation)

---

**Note:** Tested and verified on ESP32-C3 with 3-digit 7-segment WS2812B display (88 LEDs total).
