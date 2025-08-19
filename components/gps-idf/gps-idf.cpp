#include "gps-idf.h"
#include "esphome/core/log.h"

namespace esphome {
namespace nmea_gps {

static const char *TAG = "nmea_gps";

void NMEAGPSComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up NMEA GPS component...");
  // UART is initialized by ESPHome's UARTDevice
  this->buffer_.reserve(256);  // Reserve buffer for NMEA sentences
}

void NMEAGPSComponent::loop() {
  // Read available UART data
  while (this->available()) {
    char c = this->read();
    if (c == '\n') {
      if (!this->buffer_.empty() && this->buffer_[0] == '$') {
        process_nmea_sentence(this->buffer_);
      }
      this->buffer_.clear();
    } else {
      this->buffer_ += c;
    }
  }
}

void NMEAGPSComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "NMEA GPS:");
  LOG_SENSOR("  ", "Latitude", this->latitude_sensor_);
  LOG_SENSOR("  ", "Longitude", this->longitude_sensor_);
  LOG_SENSOR("  ", "Altitude", this->altitude_sensor_);
  LOG_SENSOR("  ", "Speed", this->speed_sensor_);
  LOG_SENSOR("  ", "Course", this->course_sensor_);
  LOG_SENSOR("  ", "Satellites", this->satellites_sensor_);
  LOG_SENSOR("  ", "HDOP", this->hdop_sensor_);
  LOG_TEXT_SENSOR("  ", "DateTime", this->datetime_sensor_);
  LOG_TEXT_SENSOR("  ", "Fix Status", this->fix_status_sensor_);
  ESP_LOGCONFIG(TAG, "  Verbose Logging: %s", this->verbose_logging_ ? "ON" : "OFF");
}

void NMEAGPSComponent::process_nmea_sentence(const std::string &sentence) {
  if (this->verbose_logging_) {
    ESP_LOGD(TAG, "Received NMEA: %s", sentence.c_str());
  }

  if (sentence.rfind("$GPGGA", 0) == 0 || sentence.rfind("$GNGGA", 0) == 0) {
    parse_gga(sentence);
  } else if (sentence.rfind("$GPRMC", 0) == 0 || sentence.rfind("$GNRMC", 0) == 0) {
    parse_rmc(sentence);
  }
}

void NMEAGPSComponent::parse_gga(const std::string &sentence) {
  auto fields = split(sentence, ',');
  if (fields.size() < 10) {
    if (this->verbose_logging_) {
      ESP_LOGW(TAG, "Invalid GGA sentence: too few fields");
    }
    clear_sensors();
    return;
  }

  // Fix quality (0 = invalid, 1 = GPS fix, 2 = DGPS fix, etc.)
  int fix_quality = std::atoi(fields[6].c_str());
  this->has_fix_ = (fix_quality > 0);

  if (this->fix_status_sensor_) {
    if (!this->has_fix_) {
      this->fix_status_sensor_->publish_state("No Fix");
    } else {
      this->fix_status_sensor_->publish_state(fix_quality == 1 ? "2D Fix" : "3D Fix");
    }
  }

  if (!this->has_fix_) {
    clear_sensors();
    return;
  }

  // Latitude (e.g., 4807.038,N = 48 deg 07.038' N)
  if (this->latitude_sensor_ && !fields[2].empty() && !fields[3].empty()) {
    float lat = parse_coord(fields[2], fields[3]);
    this->latitude_sensor_->publish_state(lat);
  }

  // Longitude (e.g., 01131.000,E = 11 deg 31.000' E)
  if (this->longitude_sensor_ && !fields[4].empty() && !fields[5].empty()) {
    float lon = parse_coord(fields[4], fields[5]);
    this->longitude_sensor_->publish_state(lon);
  }

  // Number of satellites
  if (this->satellites_sensor_ && !fields[7].empty()) {
    this->satellites_sensor_->publish_state(std::atoi(fields[7].c_str()));
  }

  // HDOP
  if (this->hdop_sensor_ && !fields[8].empty()) {
    this->hdop_sensor_->publish_state(std::stof(fields[8]));
  }

  // Altitude (meters above mean sea level)
  if (this->altitude_sensor_ && !fields[9].empty() && !fields[10].empty() && fields[10] == "M") {
    this->altitude_sensor_->publish_state(std::stof(fields[9]));
  }
}

void NMEAGPSComponent::parse_rmc(const std::string &sentence) {
  auto fields = split(sentence, ',');
  if (fields.size() < 12) {
    if (this->verbose_logging_) {
      ESP_LOGW(TAG, "Invalid RMC sentence: too few fields");
    }
    return;
  }

  // Check if data is valid (A = OK, V = Warning)
  if (fields[2] != "A") {
    if (this->verbose_logging_) {
      ESP_LOGW(TAG, "RMC data invalid (status: %s)", fields[2].c_str());
    }
    return;
  }

  // Speed (knots to km/h)
  if (this->speed_sensor_ && !fields[7].empty()) {
    float speed_knots = std::stof(fields[7]);
    float speed_kmh = speed_knots * 1.852;  // Convert knots to km/h
    this->speed_sensor_->publish_state(speed_kmh);
  }

  // Course (degrees)
  if (this->course_sensor_ && !fields[8].empty()) {
    this->course_sensor_->publish_state(std::stof(fields[8]));
  }

  // Date and time (e.g., 120923,225446 = 12/09/23, 22:54:46 UTC)
  if (this->datetime_sensor_ && !fields[9].empty() && !fields[1].empty()) {
    std::string date = fields[9];  // DDMMYY
    std::string time = fields[1];  // HHMMSS.sss
    if (date.size() >= 6 && time.size() >= 6) {
      std::string datetime = "20" + date.substr(4, 2) + "-" + date.substr(2, 2) + "-" +
                            date.substr(0, 2) + "T" + time.substr(0, 2) + ":" +
                            time.substr(2, 2) + ":" + time.substr(4, 2) + "Z";
      this->datetime_sensor_->publish_state(datetime);
    }
  }
}

std::vector<std::string> NMEAGPSComponent::split(const std::string &str, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  for (char c : str) {
    if (c == delimiter) {
      tokens.push_back(token);
      token.clear();
    } else {
      token += c;
    }
  }
  if (!token.empty()) {
    tokens.push_back(token);
  }
  return tokens;
}

float NMEAGPSComponent::parse_coord(const std::string &value, const std::string &direction) {
  if (value.empty()) return 0.0f;
  float val = std::stof(value);
  int degrees = static_cast<int>(val / 100);
  float minutes = val - (degrees * 100);
  float decimal = degrees + (minutes / 60.0f);
  if (direction == "S" || direction == "W") {
    decimal = -decimal;
  }
  return decimal;
}

void NMEAGPSComponent::clear_sensors() {
  if (this->latitude_sensor_) this->latitude_sensor_->publish_state(NAN);
  if (this->longitude_sensor_) this->longitude_sensor_->publish_state(NAN);
  if (this->altitude_sensor_) this->altitude_sensor_->publish_state(NAN);
  if (this->speed_sensor_) this->speed_sensor_->publish_state(NAN);
  if (this->course_sensor_) this->course_sensor_->publish_state(NAN);
  if (this->satellites_sensor_) this->satellites_sensor_->publish_state(NAN);
  if (this->hdop_sensor_) this->hdop_sensor_->publish_state(NAN);
  if (this->datetime_sensor_) this->datetime_sensor_->publish_state("");
}

}  // namespace nmea_gps
}  // namespace esphome