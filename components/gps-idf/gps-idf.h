// my_components/gps_tiny/gps_tiny.h
#pragma once


#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"


#include "TinyGPS++.h"


namespace esphome {
namespace gps_tiny {


class GPSTinyComponent : public PollingComponent, public uart::UARTDevice {
public:
void set_publish_only_on_fix(bool v) { publish_only_on_fix_ = v; }


void set_latitude_sensor(sensor::Sensor *s) { latitude_ = s; }
void set_longitude_sensor(sensor::Sensor *s) { longitude_ = s; }
void set_altitude_sensor(sensor::Sensor *s) { altitude_ = s; }
void set_speed_sensor(sensor::Sensor *s) { speed_ = s; }
void set_course_sensor(sensor::Sensor *s) { course_ = s; }
void set_hdop_sensor(sensor::Sensor *s) { hdop_ = s; }
void set_satellites_sensor(sensor::Sensor *s) { satellites_ = s; }


void set_date_text_sensor(text_sensor::TextSensor *s) { date_ = s; }
void set_time_text_sensor(text_sensor::TextSensor *s) { time_ = s; }
void set_datetime_text_sensor(text_sensor::TextSensor *s) { datetime_ = s; }


void set_fix_binary_sensor(binary_sensor::BinarySensor *b) { fix_ = b; }


void setup() override {} // nothing to init; UART is managed by UARTDevice
void loop() override; // continuously feed TinyGPS++
void update() override; // publish values at the configured interval
void dump_config() override;


protected:
TinyGPSPlus gps_;
bool publish_only_on_fix_{true};


sensor::Sensor *latitude_{nullptr};
sensor::Sensor *longitude_{nullptr};
sensor::Sensor *altitude_{nullptr};
sensor::Sensor *speed_{nullptr};
sensor::Sensor *course_{nullptr};
sensor::Sensor *hdop_{nullptr};
sensor::Sensor *satellites_{nullptr};


text_sensor::TextSensor *date_{nullptr};
text_sensor::TextSensor *time_{nullptr};
text_sensor::TextSensor *datetime_{nullptr};


binary_sensor::BinarySensor *fix_{nullptr};
};


} // namespace gps_tiny
} // namespace esphome