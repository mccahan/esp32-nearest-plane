# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-based flight tracker that displays the nearest aircraft on a 480x480 touchscreen (Guition ESP32-S3-4848S040). Fetches real-time ADS-B data from airplanes.live API and shows aircraft callsign, route, distance, altitude, speed, and a compass arrow pointing toward the plane.

## Build & Deploy Commands

```bash
# Build and flash via USB
pio run -t upload

# OTA deploy (build + upload + verify) - preferred for iteration
./scripts/deploy.sh <device-ip>

# Build only
pio run

# Get device IP/status via serial
./scripts/status.sh
```

## Screenshot Workflow

```bash
# Capture and download screenshot
curl -X POST http://<device-ip>/api/screenshot/capture
curl -o screenshot.bmp http://<device-ip>/api/screenshot/download

# Or view in browser
open http://<device-ip>/api/screenshot/view
```

## Architecture

### Main Components

- **src/main.cpp** - All flight tracker logic: API fetching, UI rendering, animations
- **src/web_server.cpp** - HTTP endpoints for screenshots, WiFi config, OTA, device info
- **src/screenshot.cpp** - BMP capture from LVGL display buffer

### Key Data Flow

1. Device fetches aircraft data from `api.airplanes.live/v2/point` every 30s
2. Filters out ground aircraft (< 1000ft altitude or < 50 knots)
3. Sorts by distance, picks nearest + 5 more for minimap
4. Updates UI: callsign with split-flap animation, compass arrow with smooth rotation
5. Position interpolated between API calls based on velocity/heading

### UI Structure

Single screen with:
- Background gradient image (`bg_image.c`)
- Callsign label with split-flap text animation
- Route badge (origin → destination from adsbdb.com lookup)
- Compass container with rotating arrow (`arrow_130.c`)
- Minimap showing up to 5 nearby aircraft (`small_arrow.c`)
- Stats: altitude, airspeed, distance

### Dynamic Search Radius

Automatically adjusts API query radius (10-100 miles) to maintain ~6 aircraft in results. Expands when too few planes found, contracts when too many.

### Configuration

- **Location**: Stored in NVS Preferences, configurable via web interface at device IP
- **WiFi**: Stored in NVS; falls back to AP mode "ESP32-Display" if unconfigured
- **secrets.h**: Optional default WiFi credentials (gitignored)

## Key Files

| File | Purpose |
|------|---------|
| `src/main.cpp` | Flight tracker logic, UI, API calls |
| `src/web_server.cpp` | HTTP API endpoints |
| `DISPLAY-CLAUDE.md` | Hardware specs, pin mappings, troubleshooting |
| `include/lv_conf.h` | LVGL 8.x configuration |
| `src/*.c` | Compiled font/image assets for LVGL |

## Adding Image/Font Assets

1. Create source image (PNG with transparency for overlays)
2. Convert to C array with LVGL image converter (RGB565 + Alpha format)
3. Add to `src/` as `.c` file
4. Declare with `LV_IMG_DECLARE(asset_name);`
5. Use with `lv_img_set_src(img_obj, &asset_name);`

## Common Patterns

### API Data Parsing
Aircraft data parsed from JSON into `AircraftData` struct. Distance/bearing calculated using Haversine formula from user coordinates.

### LVGL Animations
- Arrow rotation: smooth interpolation toward target angle at 90°/sec
- Split-flap callsign: character-by-character reveal with random cycling

### Web API Endpoints
- `POST /api/screenshot/capture` - Capture display
- `GET /api/screenshot/download` - Download BMP
- `GET /api/info` - Device stats
- `POST /api/location` - Set user coordinates

## Hardware Notes

- Display: 480x480 ST7701 RGB panel at 8MHz pixel clock
- Touch: GT911 on I2C (SDA=19, SCL=45), coordinates inverted
- Memory: 8MB PSRAM for double-buffered display, ~135KB heap
- Backlight: GPIO 38 (PWM)

See **DISPLAY-CLAUDE.md** for complete hardware reference and troubleshooting.
