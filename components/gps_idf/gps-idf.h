#pragma once

#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string>
#include <vector>

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/components/uart/uart.h"

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome {
namespace gps_idf {

enum SensorType : uint8_t {
  LATITUDE = 0,
  LONGITUDE,
  ALTITUDE,
  SPEED,
  COURSE,
  SATELLITES,
  HDOP,
};

enum TextSensorType : uint8_t {
  DATETIME = 0,
  FIX_STATUS,
};

class GPSIDFComponent : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void dump_config() override;

#ifdef USE_SENSOR
  void register_sensor(sensor::Sensor *sensor, SensorType type);
#endif
#ifdef USE_TEXT_SENSOR
  void register_text_sensor(text_sensor::TextSensor *sensor, TextSensorType type);
#endif

 protected:
#ifdef USE_SENSOR
  sensor::Sensor *latitude_sensor_{nullptr};
  sensor::Sensor *longitude_sensor_{nullptr};
  sensor::Sensor *altitude_sensor_{nullptr};
  sensor::Sensor *speed_sensor_{nullptr};
  sensor::Sensor *course_sensor_{nullptr};
  sensor::Sensor *satellites_sensor_{nullptr};
  sensor::Sensor *hdop_sensor_{nullptr};
#endif
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *datetime_sensor_{nullptr};
  text_sensor::TextSensor *fix_status_sensor_{nullptr};
#endif

  std::string buffer_;
  bool has_fix_{false};
  TaskHandle_t gps_task_handle_{nullptr};

  void process_nmea_sentence(const std::string &sentence);
  void parse_gga(const std::string &sentence);
  void parse_rmc(const std::string &sentence);
  std::vector<std::string> split(const std::string &str, char delimiter);
  float parse_coord(const std::string &value, const std::string &direction);
  void clear_sensors();

  static void gps_task(void *pvParameters);
};

}  // namespace gps_idf
}  // namespace esphome