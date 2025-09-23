#include "gps-idf.h"

#include "esphome/core/log.h"

namespace esphome {
namespace gps_idf {

static const char *const TAG = "gps_idf";
static const size_t MAX_UDP_QUEUE_SIZE = 20;
static const size_t MAX_UDP_PAYLOAD = 1400;

void GPSIDFComponent::setup() {
  ESP_LOGI(TAG, "Setting up GPSIDFComponent...");

  if (udp_broadcast_sentence_filter_.empty()) {
    ESP_LOGI(TAG, "No filters configured, using defaults");
    udp_broadcast_sentence_filter_.push_back("GNGGA");
    udp_broadcast_sentence_filter_.push_back("GNGGA");
    udp_broadcast_sentence_filter_.push_back("GPGGA");
    udp_broadcast_sentence_filter_.push_back("GPRMC");
  }

  // Create mutex for UDP queue access
  udp_queue_mutex_ = xSemaphoreCreateMutex();
  if (!udp_queue_mutex_) {
    ESP_LOGE(TAG, "Failed to create udp_queue mutex");
  }

  // Start FreeRTOS task for GPS parsing
  xTaskCreatePinnedToCore(gps_task,           // task entry
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
  ESP_LOGCONFIG(TAG, "  UDP Broadcast: %s",
                udp_broadcast_enabled_ ? "enabled" : "disabled");
  if (udp_broadcast_enabled_) {
    ESP_LOGCONFIG(TAG, "  Interval: %d ms", udp_broadcast_interval_ms_);
    ESP_LOGCONFIG(TAG, "  Sentence Filter:");
    if (udp_broadcast_sentence_filter_.empty()) {
      ESP_LOGCONFIG(TAG, "    - (empty)");
    } else {
      for (const auto &filter : udp_broadcast_sentence_filter_) {
        ESP_LOGCONFIG(TAG, "    - %s", filter.c_str());
      }
    }
  }
}

void GPSIDFComponent::loop() {
  // This function is called repeatedly by the main ESPHome loop.
  // We check if it's time to send the queued UDP data.
  if (udp_broadcast_enabled_ && udp_ != nullptr) {
    flush_udp_broadcast();
  }
}

void GPSIDFComponent::gps_task(void *pvParameters) {
  auto *self = static_cast<GPSIDFComponent *>(pvParameters);

  std::string sentence;
  sentence.reserve(128);

  while (true) {
    while (self->available()) {
      char c;
      self->read_byte(reinterpret_cast<uint8_t *>(&c));

      if (c == '\n') {
        // End of NMEA sentence
        if (!sentence.empty()) {
          self->process_nmea_sentence(sentence);

          if (esphome::network::is_connected() && self->udp_broadcast_enabled_ && self->udp_ != nullptr) {
            // Only queue if no filter configured (empty) OR sentence matches one of the filters
            bool passes_filter = self->udp_broadcast_sentence_filter_.empty();

            if (!passes_filter) {
              for (const auto &f : self->udp_broadcast_sentence_filter_) {
                if (sentence.find(f) != std::string::npos) {
                  passes_filter = true;
                  break;
                }
              }
            }

            if (passes_filter) {
              self->queue_udp_sentence(sentence);
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
  float altitude = fields[9].empty() ? 0 : atof(fields[9].c_str());

  if (latitude_sensor_ != nullptr) latitude_sensor_->publish_state(lat);
  if (longitude_sensor_ != nullptr) longitude_sensor_->publish_state(lon);
  if (satellites_sensor_ != nullptr) satellites_sensor_->publish_state(sats);
  if (hdop_sensor_ != nullptr) hdop_sensor_->publish_state(hdop);
  if (altitude_sensor_ != nullptr) altitude_sensor_->publish_state(altitude);
}

void GPSIDFComponent::parse_rmc(const std::string &sentence) {
  auto fields = split(sentence, ',');
  if (fields.size() < 10) return;

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

std::vector<std::string> GPSIDFComponent::split(const std::string &str,
                                                char delimiter) {
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

float GPSIDFComponent::parse_coord(const std::string &value,
                                  const std::string &direction) {
  if (value.empty() || direction.empty()) return 0.0f;

  double raw = atof(value.c_str());
  int degrees = static_cast<int>(raw / 100);
  double minutes = raw - (degrees * 100);
  double coord = degrees + minutes / 60.0;

  if (direction == "S" || direction == "W") coord = -coord;
  return static_cast<float>(coord);
}

void GPSIDFComponent::queue_udp_sentence(const std::string &sentence) {
  std::string full_sentence = sentence + "\r\n";

  if (udp_queue_mutex_ && xSemaphoreTake(udp_queue_mutex_, pdMS_TO_TICKS(100))) {
    if (udp_queue_.size() >= MAX_UDP_QUEUE_SIZE) {
      ESP_LOGV(TAG, "UDP queue full. Dropping oldest sentence to make space.");
      udp_queue_.pop_front();
    }
    udp_queue_.push_back(full_sentence);
    xSemaphoreGive(udp_queue_mutex_);
  } else {
    ESP_LOGW(TAG, "queue_udp_sentence: could not take udp_queue_mutex, dropping sentence");
  }
}

void GPSIDFComponent::flush_udp_broadcast() {
  // This function is called from loop() and must be fast and non-blocking.
  if (!udp_ || udp_queue_.empty()) {
    return;
  }

  TickType_t now = xTaskGetTickCount();
  TickType_t interval_ticks = pdMS_TO_TICKS(udp_broadcast_interval_ms_);

  // Immediately return if it's not yet time to send. This is the common case.
  if ((now - last_broadcast_ticks_) < interval_ticks) {
    return;
  }
  last_broadcast_ticks_ = now;

  // Combine all queued sentences into a single payload.
  std::string payload;
  if (udp_queue_mutex_ && xSemaphoreTake(udp_queue_mutex_, pdMS_TO_TICKS(100))) {
    payload.reserve(udp_queue_.size() * 85);
    for (const auto &sentence : udp_queue_) payload.append(sentence);
    udp_queue_.clear();
    xSemaphoreGive(udp_queue_mutex_);
  } else {
    ESP_LOGW(TAG, "flush_udp_broadcast: could not take udp_queue_mutex");
    return;
  }

  ESP_LOGI(TAG, "Flushing %d bytes to UDP broadcast.", static_cast<int>(payload.size()));

  size_t offset = 0;
  while (offset < payload.size()) {
    size_t chunk_size = std::min(MAX_UDP_PAYLOAD, payload.size() - offset);
    udp_->send_packet(reinterpret_cast<const uint8_t *>(payload.c_str() + offset), chunk_size);
    ESP_LOGI(TAG, "flush_udp_broadcast: sent %d bytes", static_cast<int>(chunk_size));
    offset += chunk_size;
  }
}

}  // namespace gps_idf
}  // namespace esphome