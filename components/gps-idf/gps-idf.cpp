#include "gps-idf.h"


#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "esphome/core/log.h"

namespace esphome {
namespace gps_idf {

static const char *TAG = "gps_idf";

void GPSIDFComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up GPS-IDF component...");
  this->buffer_.reserve(256);  // Reserve buffer for NMEA sentences

  if (udp_broadcast_enabled_) {
    setup_udp_broadcast();
  }

  // Create FreeRTOS task for GPS processing
  BaseType_t result = xTaskCreate(
      gps_task, "gps_task", 4096, this, 5, &this->gps_task_handle_);
  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create GPS task");
  } else {
    ESP_LOGI(TAG, "GPS task created successfully");
  }
}

void GPSIDFComponent::setup_udp_broadcast() {
  ESP_LOGCONFIG(TAG, "Setting up UDP broadcast to %s:%d", udp_broadcast_address_.c_str(), udp_broadcast_port_);
  
  udp_socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (udp_socket_ < 0) {
    ESP_LOGE(TAG, "Failed to create UDP socket");
    return;
  }

  int broadcast = 1;
  if (setsockopt(udp_socket_, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
    ESP_LOGE(TAG, "Failed to enable broadcast");
    close(udp_socket_);
    udp_socket_ = -1;
    return;
  }

  memset(&udp_dest_addr_, 0, sizeof(udp_dest_addr_));
  udp_dest_addr_.sin_family = AF_INET;
  udp_dest_addr_.sin_port = htons(udp_broadcast_port_);
  udp_dest_addr_.sin_addr.s_addr = inet_addr(udp_broadcast_address_.c_str());

  ESP_LOGI(TAG, "UDP broadcast setup complete");
}

void GPSIDFComponent::gps_task(void *pvParameters) {
  GPSIDFComponent *gps = static_cast<GPSIDFComponent *>(pvParameters);
  while (true) {
    while (gps->available()) {
      char c = gps->read();
      if (c == '\n') {
        if (!gps->buffer_.empty() && gps->buffer_[0] == '$') {
          gps->process_nmea_sentence(gps->buffer_);
        }
        gps->buffer_.clear();
      } else {
        gps->buffer_ += c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to prevent CPU hogging
  }
}

void GPSIDFComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "GPS-IDF:");
  LOG_SENSOR("  ", "Latitude", this->latitude_sensor_);
  LOG_SENSOR("  ", "Longitude", this->longitude_sensor_);
  LOG_SENSOR("  ", "Altitude", this->altitude_sensor_);
  LOG_SENSOR("  ", "Speed", this->speed_sensor_);
  LOG_SENSOR("  ", "Course", this->course_sensor_);
  LOG_SENSOR("  ", "Satellites", this->satellites_sensor_);
  LOG_SENSOR("  ", "HDOP", this->hdop_sensor_);
  LOG_TEXT_SENSOR("  ", "DateTime", this->datetime_sensor_);
  LOG_TEXT_SENSOR("  ", "Fix Status", this->fix_status_sensor_);
  if (udp_broadcast_enabled_) {
    ESP_LOGCONFIG(TAG, "  UDP Broadcast: Enabled, Address: %s, Port: %d, Interval: %d ms",
                  udp_broadcast_address_.c_str(), udp_broadcast_port_, udp_broadcast_interval_ms_);
    ESP_LOGCONFIG(TAG, "  UDP Sentence Filter: %s", vector_to_string(udp_broadcast_sentence_filter_).c_str());
  } else {
    ESP_LOGCONFIG(TAG, "  UDP Broadcast: Disabled");
  }
}

void GPSIDFComponent::process_nmea_sentence(const std::string &sentence) {
  ESP_LOGD(TAG, "Received NMEA: %s", sentence.c_str());

  if (udp_broadcast_enabled_) {
    TickType_t current_ticks = xTaskGetTickCount();
    if ((current_ticks - last_broadcast_ticks_) * portTICK_PERIOD_MS >= udp_broadcast_interval_ms_) {
      for (const auto &filter : udp_broadcast_sentence_filter_) {
        if (sentence.rfind("$" + filter, 0) == 0 || sentence.rfind("$GN" + filter, 0) == 0) {
          send_udp_broadcast(sentence);
          last_broadcast_ticks_ = current_ticks;
          break;
        }
      }
    }
  }

  if (sentence.rfind("$GPGGA", 0) == 0 || sentence.rfind("$GNGGA", 0) == 0) {
    parse_gga(sentence);
  } else if (sentence.rfind("$GPRMC", 0) == 0 || sentence.rfind("$GNRMC", 0) == 0) {
    parse_rmc(sentence);
  }
}

void GPSIDFComponent::send_udp_broadcast(const std::string &sentence) {
  if (udp_socket_ < 0) {
    ESP_LOGW(TAG, "UDP socket not initialized, skipping broadcast");
    return;
  }

  std::string message = sentence + "\r\n";
  int err = sendto(udp_socket_, message.c_str(), message.length(), 0,
                   (struct sockaddr *)&udp_dest_addr_, sizeof(udp_dest_addr_));
  if (err < 0) {
    ESP_LOGE(TAG, "Failed to send UDP broadcast: %d", err);
  } else {
    ESP_LOGD(TAG, "Sent UDP broadcast: %s", sentence.c_str());
  }
}

void GPSIDFComponent::parse_gga(const std::string &sentence) {
  auto fields = split(sentence, ',');
  if (fields.size() < 10) {
    ESP_LOGD(TAG, "Invalid GGA sentence: too few fields");
    clear_sensors();
    return;
  }

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

  if (this->latitude_sensor_ && !fields[2].empty() && !fields[3].empty()) {
    float lat = parse_coord(fields[2], fields[3]);
    this->latitude_sensor_->publish_state(lat);
  }

  if (this->longitude_sensor_ && !fields[4].empty() && !fields[5].empty()) {
    float lon = parse_coord(fields[4], fields[5]);
    this->longitude_sensor_->publish_state(lon);
  }

  if (this->satellites_sensor_ && !fields[7].empty()) {
    this->satellites_sensor_->publish_state(std::atoi(fields[7].c_str()));
  }

  if (this->hdop_sensor_ && !fields[8].empty()) {
    this->hdop_sensor_->publish_state(std::stof(fields[8]));
  }

  if (this->altitude_sensor_ && !fields[9].empty() && !fields[10].empty() && fields[10] == "M") {
    this->altitude_sensor_->publish_state(std::stof(fields[9]));
  }
}

void GPSIDFComponent::parse_rmc(const std::string &sentence) {
  auto fields = split(sentence, ',');
  if (fields.size() < 12) {
    ESP_LOGD(TAG, "Invalid RMC sentence: too few fields");
    return;
  }

  if (fields[2] != "A") {
    ESP_LOGD(TAG, "RMC data invalid (status: %s)", fields[2].c_str());
    return;
  }

  if (this->speed_sensor_ && !fields[7].empty()) {
    float speed_knots = std::stof(fields[7]);
    float speed_kmh = speed_knots * 1.852;
    this->speed_sensor_->publish_state(speed_kmh);
  }

  if (this->course_sensor_ && !fields[8].empty()) {
    this->course_sensor_->publish_state(std::stof(fields[8]));
  }

  if (this->datetime_sensor_ && !fields[9].empty() && !fields[1].empty()) {
    std::string date = fields[9];
    std::string time = fields[1];
    if (date.size() >= 6 && time.size() >= 6) {
      std::string datetime = "20" + date.substr(4, 2) + "-" + date.substr(2, 2) + "-" +
                             date.substr(0, 2) + "T" + time.substr(0, 2) + ":" +
                             time.substr(2, 2) + ":" + time.substr(4, 2) + "Z";
      this->datetime_sensor_->publish_state(datetime);
    }
  }
}

std::vector<std::string> GPSIDFComponent::split(const std::string &str, char delimiter) {
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

float GPSIDFComponent::parse_coord(const std::string &value, const std::string &direction) {
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

void GPSIDFComponent::clear_sensors() {
  if (this->latitude_sensor_) this->latitude_sensor_->publish_state(NAN);
  if (this->longitude_sensor_) this->longitude_sensor_->publish_state(NAN);
  if (this->altitude_sensor_) this->altitude_sensor_->publish_state(NAN);
  if (this->speed_sensor_) this->speed_sensor_->publish_state(NAN);
  if (this->course_sensor_) this->course_sensor_->publish_state(NAN);
  if (this->satellites_sensor_) this->satellites_sensor_->publish_state(NAN);
  if (this->hdop_sensor_) this->hdop_sensor_->publish_state(NAN);
  if (this->datetime_sensor_) this->datetime_sensor_->publish_state("");
}

std::string GPSIDFComponent::vector_to_string(const std::vector<std::string> &vec) {
  std::string result;
  for (size_t i = 0; i < vec.size(); ++i) {
    result += vec[i];
    if (i < vec.size() - 1) {
      result += ", ";
    }
  }
  return result;
}

}  // namespace gps_idf
}  // namespace esphome