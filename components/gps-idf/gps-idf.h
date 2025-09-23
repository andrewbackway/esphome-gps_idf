#pragma once

#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <deque>
#include <string>
#include <vector>

#include "esphome.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/udp/udp_component.h"
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
  void set_latitude_sensor(sensor::Sensor *sensor) { latitude_sensor_ = sensor; }
  void set_longitude_sensor(sensor::Sensor *sensor) { longitude_sensor_ = sensor; }
  void set_altitude_sensor(sensor::Sensor *sensor) { altitude_sensor_ = sensor; }
  void set_speed_sensor(sensor::Sensor *sensor) { speed_sensor_ = sensor; }
  void set_course_sensor(sensor::Sensor *sensor) { course_sensor_ = sensor; }
  void set_satellites_sensor(sensor::Sensor *sensor) { satellites_sensor_ = sensor; }
  void set_hdop_sensor(sensor::Sensor *sensor) { hdop_sensor_ = sensor; }
  void set_datetime_sensor(text_sensor::TextSensor *sensor) { datetime_sensor_ = sensor; }
  void set_fix_status_sensor(text_sensor::TextSensor *sensor) { fix_status_sensor_ = sensor; }
  void set_udp_broadcast_enabled(bool enabled) { udp_broadcast_enabled_ = enabled; }
  void set_udp_broadcast_interval(uint32_t interval_ms) { udp_broadcast_interval_ms_ = interval_ms; }
  void add_udp_broadcast_sentence_filter(const std::string &sentence) { udp_broadcast_sentence_filter_.push_back(sentence); }
  void set_udp(udp::UDPComponent *udp) { udp_ = udp; }

 protected:
 
  sensor::Sensor *altitude_sensor_{nullptr};
  sensor::Sensor *latitude_sensor_{nullptr};
  sensor::Sensor *longitude_sensor_{nullptr};
  sensor::Sensor *altitude_sensor_{nullptr};
  sensor::Sensor *speed_sensor_{nullptr};
  sensor::Sensor *course_sensor_{nullptr};
  sensor::Sensor *satellites_sensor_{nullptr};
  sensor::Sensor *hdop_sensor_{nullptr};
  text_sensor::TextSensor *datetime_sensor_{nullptr};
  text_sensor::TextSensor *fix_status_sensor_{nullptr};

  std::string buffer_;
  std::deque<std::string> udp_queue_;
  SemaphoreHandle_t udp_queue_mutex_{nullptr};
  bool has_fix_{false};
  TaskHandle_t gps_task_handle_{nullptr};

  // UDP broadcast configuration
  bool udp_broadcast_enabled_{false};
  uint32_t udp_broadcast_interval_ms_{15000};
  std::vector<std::string> udp_broadcast_sentence_filter_;
  TickType_t last_broadcast_ticks_{0};

  udp::UDPComponent *udp_{nullptr};

  void process_nmea_sentence(const std::string &sentence);
  void parse_gga(const std::string &sentence);
  void parse_rmc(const std::string &sentence);
  std::vector<std::string> split(const std::string &str, char delimiter);
  float parse_coord(const std::string &value, const std::string &direction);
  void queue_udp_sentence(const std::string &sentence);
  void flush_udp_broadcast();

  static void gps_task(void *pvParameters);
};

}  // namespace gps_idf
}  // namespace esphome