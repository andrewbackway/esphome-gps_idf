# Copilot Instructions for ESPHome GPS-IDF Component

## Project Overview
- This project is an ESPHome external component for ESP32 devices using the ESP-IDF framework.
- It parses NMEA GPS data (GGA and RMC sentences) from a UART-connected GPS module and exposes sensor data to ESPHome/Home Assistant.
- GPS parsing runs in a dedicated FreeRTOS task for non-blocking operation.

## Key Files & Structure
- `components/gps-idf/gps-idf.cpp` / `gps-idf.h`: Main logic for NMEA parsing, UART communication, and sensor updates.
- `components/gps-idf/__init__.py`: Python stub for ESPHome external component registration.
- `components/gps-idf/sample.yaml`: Example ESPHome configuration for integration.
- `README.md`: Hardware setup, configuration, and usage instructions.

## Build & Development Workflow
- Firmware is built and uploaded via ESPHome (not PlatformIO or IDF directly).
- YAML configuration must specify `framework: type: esp-idf` and reference this repo as an external component.
- Use the ESPHome dashboard or CLI for compiling and uploading firmware.
- Debugging is done via the ESPHome logger (enable `logger:` in YAML).

## Integration & Configuration
- All configuration is done in YAML (see `sample.yaml` and `README.md`).
- UART pins, baud rate, and sensor mappings are set in YAML, not code.
- Sensors report `NaN` or empty string if no valid GPS fix is available.
- `fix_status` sensor reports "No Fix", "2D Fix", or "3D Fix" based on parsed NMEA data.

## Patterns & Conventions
- NMEA parsing supports both `$GPGGA`/`$GNGGA` and `$GPRMC`/`$GNRMC` for compatibility.
- GPS parsing runs in a FreeRTOS task (not main loop) for reliability.
- UART uses ESP-IDF driver, not Arduino/ESPHome UART.
- All sensor data is exposed via ESPHome sensors/text sensors, not custom APIs.

## Troubleshooting & Debugging
- Check UART wiring and baud rate if no data is received.
- Use ESPHome logs for error details (task creation, NMEA parsing, etc.).
- Ensure GPS module outputs NMEA at 9600 baud (default).

## Example: Adding a New Sensor
- To expose new GPS data, update NMEA parsing in `gps-idf.cpp` and add a corresponding sensor definition in YAML.
- Follow the pattern in `sample.yaml` for sensor configuration.

## External Dependencies
- ESPHome 2025.5.0+ required.
- ESP32 board and UART GPS module (e.g., NEO-M8N).

---
For more details, see `README.md` and `components/gps-idf/sample.yaml`.
