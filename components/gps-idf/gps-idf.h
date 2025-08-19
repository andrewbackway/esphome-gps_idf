#pragma once

#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string>
#include <vector>

#include "esphome.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome {
namespace gps_idf {

class GPSIDFComponent : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Setters for YAML configuration
  void set_latitude_sensor(sensor::Sensor *sensor) {
    latitude_sensor_ = sensor;
  }
  void set_longitude_sensor(sensor::Sensor *sensor) {
    longitude_sensor_ = sensor;
  }
  void set_altitude_sensor(sensor::Sensor *sensor) {
    altitude_sensor_ = sensor;
  }
  void set_speed_sensor(sensor::Sensor *sensor) { speed_sensor_ = sensor; }
  void set_course_sensor(sensor::Sensor *sensor) { course_sensor_ = sensor; }
  void set_satellites_sensor(sensor::Sensor *sensor) {
    satellites_sensor_ = sensor;
  }
  void set_hdop_sensor(sensor::Sensor *sensor) { hdop_sensor_ = sensor; }
  void set_datetime_sensor(text_sensor::TextSensor *sensor) {
    datetime_sensor_ = sensor;
  }
  void set_fix_status_sensor(text_sensor::TextSensor *sensor) {
    fix_status_sensor_ = sensor;
  }
  void set_verbose_logging(bool verbose) { verbose_logging_ = verbose; }

 protected:
  sensor::Sensor *latitude_sensor_{nullptr};
  sensor::Sensor *longitude_sensor_{nullptr};
  sensor::Sensor *altitude_sensor_{nullptr};
  sensor::Sensor *speed_sensor_{nullptr};
  sensor::Sensor *course_sensor_{nullptr};
  sensor::Sensor *satellites_sensor_{nullptr};
  sensor::Sensor *hdop_sensor_{nullptr};
  text_sensor::TextSensor *datetime_sensor_{nullptr};
  text_sensor::TextSensor *fix_status_sensor_{nullptr};
  bool verbose_logging_{false};

  std::string buffer_;
  bool has_fix_{false};

  void process_nmea_sentence(const std::string &sentence);
  void parse_gga(const std::string &sentence);
  void parse_rmc(const std::string &sentence);
  std::vector<std::string> split(const std::string &str, char delimiter);
  float parse_coord(const std::string &value, const std::string &direction);
  void clear_sensors();
};

}  // namespace gps_idf
}  // namespace esphome