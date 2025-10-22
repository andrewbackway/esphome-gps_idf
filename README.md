# ESPHome GPS External Component (ESP-IDF)

This is an ESPHome external component for processing NMEA GPS data using the on ESP32 devices, unlike the in built ESPHome GPS component, this works on ESP-IDF framework.  It parses NMEA sentences (GGA and RMC) from a GPS module connected via UART and exposes various GPS data as sensors and text sensors in ESPHome.

## Features

- Runs on ESP IDF Framework 
- Supports parsing of NMEA GGA and RMC sentences.
- Provides the following sensor data:
  - Latitude (°)
  - Longitude (°)
  - Altitude (m)
  - Speed (km/h)
  - Course (°)
  - Number of satellites
  - HDOP (Horizontal Dilution of Precision)
  - Date and time (ISO 8601 format)
  - Fix status (No Fix, 2D Fix, 3D Fix)
- Configurable via YAML in ESPHome.
- Uses ESP-IDF UART driver for reliable communication.
- Runs GPS parsing in a dedicated FreeRTOS task to avoid blocking the main ESPHome loop.

## Prerequisites

- ESPHome 2025.5.0 or later
- ESP32-based board (e.g., ESP32 DevKit)
- GPS module with UART interface (e.g., NEO-M8N, NEO-6M, NEO-7M)
- UART connection between the ESP32 and GPS module

## Installation

1. Add this repository as an external component in your ESPHome configuration YAML file:

```yaml
external_components:
  - source: github://andrewbackway/esphome-gps-idf
```

2. Configure the UART and GPS-IDF component in your YAML file (see the example below).

3. Compile and upload the firmware to your ESP32 device using ESPHome.

## Example Configuration

Below is a sample ESPHome configuration for using the GPS-IDF component:

```yaml
esphome:
  name: esphome-web-9dd449
  friendly_name: ESPHome GPS
  min_version: 2025.5.0
  name_add_mac_suffix: false

esp32:
  board: esp32dev
  framework:
    type: esp-idf

logger:

ota:
  - platform: esphome

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

web_server:
  version: 3
  local: true

external_components:
  - source: github://andrewbackway/esphome-gps_idf

uart:
  - id: uart_gps
    tx_pin: GPIO01
    rx_pin: GPIO03
    baud_rate: 9600

gps-idf:
  id: gps
  uart_id: uart_gps
  latitude:
    name: "GPS Latitude"
    unit_of_measurement: "°"
    accuracy_decimals: 6
  longitude:
    name: "GPS Longitude"
    unit_of_measurement: "°"
    accuracy_decimals: 6
  altitude:
    name: "GPS Altitude"
    unit_of_measurement: "m"
    accuracy_decimals: 1
  speed:
    name: "GPS Speed"
    unit_of_measurement: "km/h"
    accuracy_decimals: 1
  course:
    name: "GPS Course"
    unit_of_measurement: "°"
    accuracy_decimals: 1
  satellites:
    name: "GPS Satellites"
    accuracy_decimals: 0
  hdop:
    name: "GPS HDOP"
    accuracy_decimals: 2
  datetime:
    name: "GPS DateTime"
  fix_status:
    name: "GPS Fix Status"
```

## Hardware Setup

1. Connect the GPS module to the ESP32:
   - GPS TX to ESP32 GPIO03 (RX)
   - GPS RX to ESP32 GPIO01 (TX)
   - Power the GPS module (typically 3.3V or 5V, check your module's specifications)
   - Connect GND between the GPS module and ESP32
2. Ensure the GPS module has a clear view of the sky for satellite acquisition.

## Usage

- After uploading the firmware, the component will automatically start reading NMEA sentences from the GPS module via UART.
- The parsed data (latitude, longitude, altitude, etc.) will be published to the configured sensors in ESPHome.
- You can view the sensor data in the ESPHome dashboard or integrate it with Home Assistant.
- The component logs GPS data and errors to the ESPHome logger for debugging.

## Notes

- Ensure the GPS module is configured to output NMEA sentences at 9600 baud (default for most modules).
- The component supports both `$GPGGA`/`$GNGGA` and `$GPRMC`/`$GNRMC` sentences for compatibility with various GPS modules.
- If no valid GPS fix is available, sensors will report `NaN` (for numeric sensors) or empty strings (for text sensors).
- The `fix_status` sensor reports "No Fix", "2D Fix", or "3D Fix" based on the GPS fix quality.

## Troubleshooting

- **No GPS data**: Check UART wiring and ensure the GPS module has a clear view of the sky.
- **Invalid NMEA sentences**: Verify the baud rate matches the GPS module's configuration.
- **ESPHome logs errors**: Enable `logger` in your YAML configuration to see detailed logs for debugging.
- **Task creation failure**: Ensure sufficient memory is available on the ESP32 (the task uses a 4KB stack).

*Developed with Grok, and with a little troubleshooting*
