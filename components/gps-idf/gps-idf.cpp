#include "gps-idf.h"
#include "esphome/core/log.h"

namespace esphome {
namespace gps_idf {

static const char *const TAG = "gps_idf";

void GPSIDFComponent::setup() {
  ESP_LOGI(TAG, "Setting up GPSIDFComponent...");

  // Start FreeRTOS task for GPS parsing
  xTaskCreatePinnedToCore(
      gps_task,           // task entry
      "gps_task",         // task name
      4096,               // stack size
      this,               // parameter
      1,                  // priority
      &gps_task_handle_,  // task handle
      1                   // run on core 1
  ); 
}

void GPSIDFComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "GPSIDFComponent:");
  ESP_LOGCONFIG(TAG, "  UDP Broadcast: %s", udp_broadcast_enabled_ ? "enabled" : "disabled");
  if (udp_broadcast_enabled_) {
    ESP_LOGCONFIG(TAG, "  Broadcast Address: %s", udp_broadcast_address_.c_str());
    ESP_LOGCONFIG(TAG, "  Port: %d", udp_broadcast_port_);
    ESP_LOGCONFIG(TAG, "  Interval: %d ms", udp_broadcast_interval_ms_);
  }
}

void GPSIDFComponent::gps_task(void *pvParameters) {
  auto *self = static_cast<GPSIDFComponent *>(pvParameters);

  std::string sentence;
  sentence.reserve(128);

  while (true) {
    while (self->available()) 
    {
      char c;
      self->read_byte(reinterpret_cast<uint8_t *>(&c));

      if (c == '\n') {
        // End of NMEA sentence
        if (!sentence.empty()) {
          self->process_nmea_sentence(sentence);

          ESP_LOGI(TAG, "Processed NMEA sentence: %s", sentence.c_str());

          if (self->udp_broadcast_enabled_) {
            ESP_LOGI(TAG, "UDP Enabled");
            if (self->udp_socket_ < 0 ) {
              ESP_LOGI(TAG, "Setting up UDP");
              self->setup_udp_broadcast();
            } else {
              ESP_LOGI(TAG, "Connected to UDP");
              self->queue_udp_sentence(sentence);
              self->flush_udp_broadcast();
            }
          }

          sentence.clear();
        }
      } else if (c != '\r') {
        sentence.push_back(c);
      }
    }

    // Yield to FreeRTOS scheduler (prevents watchdog reset)
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void GPSIDFComponent::process_nmea_sentence(const std::string &sentence) {
  if (sentence.find("GGA") != std::string::npos) {
    parse_gga(sentence);
  } else if (sentence.find("RMC") != std::string::npos) {
    parse_rmc(sentence);
  }
}

void GPSIDFComponent::parse_gga(const std::string &sentence) {
  auto fields = split(sentence, ',');
  if (fields.size() < 10) return;

  float lat = parse_coord(fields[2], fields[3]);
  float lon = parse_coord(fields[4], fields[5]);
  int sats = fields[7].empty() ? 0 : atoi(fields[7].c_str());
  float hdop = fields[8].empty() ? 0 : atof(fields[8].c_str());

  if (latitude_sensor_ != nullptr) latitude_sensor_->publish_state(lat);
  if (longitude_sensor_ != nullptr) longitude_sensor_->publish_state(lon);
  if (satellites_sensor_ != nullptr) satellites_sensor_->publish_state(sats);
  if (hdop_sensor_ != nullptr) hdop_sensor_->publish_state(hdop);
}

void GPSIDFComponent::parse_rmc(const std::string &sentence) {
  auto fields = split(sentence, ',');
  if (fields.size() < 9) return;

  float speed_knots = fields[7].empty() ? 0 : atof(fields[7].c_str());
  float speed_kmh = speed_knots * 1.852f;
  float course = fields[8].empty() ? 0 : atof(fields[8].c_str());

  if (speed_sensor_ != nullptr) speed_sensor_->publish_state(speed_kmh);
  if (course_sensor_ != nullptr) course_sensor_->publish_state(course);

  if (fix_status_sensor_ != nullptr) {
    std::string status = (fields[2] == "A") ? "Fix" : "No Fix";
    fix_status_sensor_->publish_state(status);
  }

  if (datetime_sensor_ != nullptr) {
    // Date (DDMMYY) and Time (hhmmss.sss)
    std::string datetime = fields[9] + " " + fields[1];
    datetime_sensor_->publish_state(datetime);
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
      token.push_back(c);
    }
  }
  tokens.push_back(token);
  return tokens;
}

float GPSIDFComponent::parse_coord(const std::string &value, const std::string &direction) {
  if (value.empty() || direction.empty()) return 0.0f;

  double raw = atof(value.c_str());
  int degrees = static_cast<int>(raw / 100);
  double minutes = raw - (degrees * 100);
  double coord = degrees + minutes / 60.0;

  if (direction == "S" || direction == "W") coord = -coord;
  return static_cast<float>(coord);
}

bool GPSIDFComponent::setup_udp_broadcast() {
  ESP_LOGI(TAG, "Setting up UDP broadcast");

  udp_socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (udp_socket_ < 0) {
    ESP_LOGE(TAG, "Failed to create UDP socket");
    return false;
  }

  udp_dest_addr_.sin_family = AF_INET;
  udp_dest_addr_.sin_port = htons(udp_broadcast_port_);
  udp_dest_addr_.sin_addr.s_addr = inet_addr(udp_broadcast_address_.c_str());

  ESP_LOGI(TAG, "UDP broadcast setup complete");

  return true;
}

void GPSIDFComponent::queue_udp_sentence(const std::string &sentence) {
  udp_queue_.push_back(sentence);
}

void GPSIDFComponent::flush_udp_broadcast() {
  if (udp_socket_ < 0 || udp_queue_.empty()) return;

  TickType_t now = xTaskGetTickCount();
  if (now - last_broadcast_ticks_ < pdMS_TO_TICKS(udp_broadcast_interval_ms_)) {
    return;  // not time yet
  }
  last_broadcast_ticks_ = now;

  for (auto &s : udp_queue_) {
    ESP_LOGI(TAG, "Sending UDP broadcast: %s", s.c_str());
    sendto(udp_socket_, s.c_str(), s.size(), 0,
           (struct sockaddr *)&udp_dest_addr_, sizeof(udp_dest_addr_));
  }
  udp_queue_.clear();
}

}  // namespace gps_idf
}  // namespace esphome
