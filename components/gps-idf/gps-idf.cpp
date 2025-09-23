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

GPSIDFComponent::~GPSIDFComponent() {
  if (gps_task_handle_) {
    vTaskDelete(gps_task_handle_);
    gps_task_handle_ = nullptr;
  }
  if (udp_queue_mutex_) {
    vSemaphoreDelete(udp_queue_mutex_);
    udp_queue_mutex_ = nullptr;
  }
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
  // Network state monitoring
  static bool last_network_state = false;
  bool current_network_state = esphome::network::is_connected();
  
  if (last_network_state && !current_network_state) {
    // Network disconnected, clear UDP queue to prevent buildup
    ESP_LOGW(TAG, "Network disconnected, clearing UDP queue");
    if (udp_queue_mutex_ && xSemaphoreTake(udp_queue_mutex_, pdMS_TO_TICKS(100))) {
      udp_queue_.clear();
      xSemaphoreGive(udp_queue_mutex_);
    } else {
      ESP_LOGW(TAG, "Failed to acquire mutex for queue clearing");
    }
  }
  last_network_state = current_network_state;
  
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
  
  static const size_t MAX_SENTENCE_LENGTH = 256;

  while (true) {
    while (self->available()) {
      char c;
      self->read_byte(reinterpret_cast<uint8_t *>(&c));

      if (c == '\n') {
        // End of NMEA sentence
        if (!sentence.empty() && sentence.length() <= MAX_SENTENCE_LENGTH) {
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
        } else if (sentence.length() > MAX_SENTENCE_LENGTH) {
          ESP_LOGW(TAG, "NMEA sentence too long (%d chars), dropping", sentence.length());
        }
        sentence.clear();
      } else if (c != '\r') {
        sentence.push_back(c);
        
        // Prevent sentence from growing too large
        if (sentence.length() > MAX_SENTENCE_LENGTH) {
          ESP_LOGW(TAG, "NMEA sentence exceeded max length, resetting");
          sentence.clear();
        }
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
  if (fields.size() < 15) {  // GGA has at least 15 fields
    ESP_LOGV(TAG, "GGA sentence too short: %d fields", fields.size());
    return;
  }

  // Validate and parse coordinates
  float lat = parse_coord(fields[2], fields[3]);
  float lon = parse_coord(fields[4], fields[5]);
  
  // Validate coordinates are within reasonable bounds
  if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
    ESP_LOGV(TAG, "Invalid GPS coordinates: lat=%.6f, lon=%.6f", lat, lon);
    return;
  }
  
  int sats = 0;
  if (!fields[7].empty()) {
    sats = atoi(fields[7].c_str());
    if (sats < 0 || sats > 50) {  // Reasonable satellite count bounds
      ESP_LOGV(TAG, "Invalid satellite count: %d", sats);
      sats = 0;
    }
  }
  
  float hdop = 0;
  if (!fields[8].empty()) {
    hdop = atof(fields[8].c_str());
    if (hdop < 0 || hdop > 99.9) {  // HDOP should be reasonable
      ESP_LOGV(TAG, "Invalid HDOP: %.2f", hdop);
      hdop = 0;
    }
  }
  
  float altitude = 0;
  if (!fields[9].empty()) {
    altitude = atof(fields[9].c_str());
    if (altitude < -1000 || altitude > 20000) {  // Reasonable altitude bounds
      ESP_LOGV(TAG, "Invalid altitude: %.1f", altitude);
      altitude = 0;
    }
  }

  if (latitude_sensor_ != nullptr) latitude_sensor_->publish_state(lat);
  if (longitude_sensor_ != nullptr) longitude_sensor_->publish_state(lon);
  if (satellites_sensor_ != nullptr) satellites_sensor_->publish_state(sats);
  if (hdop_sensor_ != nullptr) hdop_sensor_->publish_state(hdop);
  if (altitude_sensor_ != nullptr) altitude_sensor_->publish_state(altitude);
}

void GPSIDFComponent::parse_rmc(const std::string &sentence) {
  auto fields = split(sentence, ',');
  if (fields.size() < 12) {  // RMC has at least 12 fields
    ESP_LOGV(TAG, "RMC sentence too short: %d fields", fields.size());
    return;
  }

  float speed_knots = 0;
  if (!fields[7].empty()) {
    speed_knots = atof(fields[7].c_str());
    if (speed_knots < 0 || speed_knots > 1000) {  // Reasonable speed bounds
      ESP_LOGV(TAG, "Invalid speed: %.1f knots", speed_knots);
      speed_knots = 0;
    }
  }
  
  float speed_kmh = speed_knots * 1.852f;
  
  float course = 0;
  if (!fields[8].empty()) {
    course = atof(fields[8].c_str());
    if (course < 0 || course >= 360) {  // Course should be 0-359.9 degrees
      ESP_LOGV(TAG, "Invalid course: %.1f degrees", course);
      course = 0;
    }
  }

  if (speed_sensor_ != nullptr) speed_sensor_->publish_state(speed_kmh);
  if (course_sensor_ != nullptr) course_sensor_->publish_state(course);

  if (fix_status_sensor_ != nullptr) {
    std::string status = (fields[2] == "A") ? "Fix" : "No Fix";
    fix_status_sensor_->publish_state(status);
  }

  if (datetime_sensor_ != nullptr) {
    // Validate date and time fields exist and are properly formatted
    if (!fields[9].empty() && !fields[1].empty() && 
        fields[9].length() == 6 && fields[1].length() >= 6) {
      std::string datetime = fields[9] + " " + fields[1];
      datetime_sensor_->publish_state(datetime);
    } else {
      ESP_LOGV(TAG, "Invalid datetime fields in RMC");
    }
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
  
  // Validate direction is one of the expected values
  if (direction != "N" && direction != "S" && direction != "E" && direction != "W") {
    ESP_LOGV(TAG, "Invalid coordinate direction: %s", direction.c_str());
    return 0.0f;
  }

  double raw = atof(value.c_str());
  if (raw < 0) {
    ESP_LOGV(TAG, "Invalid negative raw coordinate: %.6f", raw);
    return 0.0f;
  }
  
  int degrees = static_cast<int>(raw / 100);
  double minutes = raw - (degrees * 100);
  
  // Validate minutes are in valid range (0-59.999...)
  if (minutes < 0 || minutes >= 60.0) {
    ESP_LOGV(TAG, "Invalid minutes in coordinate: %.6f", minutes);
    return 0.0f;
  }
  
  double coord = degrees + minutes / 60.0;

  if (direction == "S" || direction == "W") coord = -coord;
  return static_cast<float>(coord);
}

void GPSIDFComponent::queue_udp_sentence(const std::string &sentence) {
  std::string full_sentence = sentence + "\r\n";

  if (udp_queue_mutex_ && xSemaphoreTake(udp_queue_mutex_, pdMS_TO_TICKS(100))) {
    if (udp_queue_.size() >= MAX_UDP_QUEUE_SIZE) {
      ESP_LOGV(TAG, "UDP queue full (%d items). Dropping oldest sentence to make space.", udp_queue_.size());
      udp_queue_.pop_front();
    }
    udp_queue_.push_back(full_sentence);
    xSemaphoreGive(udp_queue_mutex_);
  } else {
    ESP_LOGW(TAG, "queue_udp_sentence: Mutex timeout, possible contention - dropping sentence");
  }
}

void GPSIDFComponent::flush_udp_broadcast() {
  // This function is called from loop() and must be fast and non-blocking.
  if (!udp_ || udp_queue_.empty() || !esphome::network::is_connected()) {
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
  size_t queue_size = 0;
  if (udp_queue_mutex_ && xSemaphoreTake(udp_queue_mutex_, pdMS_TO_TICKS(100))) {
    queue_size = udp_queue_.size();
    payload.reserve(std::min(queue_size * 85, static_cast<size_t>(MAX_UDP_PAYLOAD * 8))); // Conservative estimate
    for (const auto &sentence : udp_queue_) {
      payload.append(sentence);
    }
    udp_queue_.clear();
    xSemaphoreGive(udp_queue_mutex_);
  } else {
    ESP_LOGW(TAG, "flush_udp_broadcast: Mutex timeout in flush - possible deadlock");
    return;
  }

  if (payload.empty()) {
    return;
  }

  // Add bounds checking for payload size
  static const size_t MAX_TOTAL_PAYLOAD = 32768; // 32KB reasonable limit
  if (payload.size() > MAX_TOTAL_PAYLOAD) {
    ESP_LOGW(TAG, "Payload too large (%d bytes), truncating to %d bytes", payload.size(), MAX_TOTAL_PAYLOAD);
    payload.resize(MAX_TOTAL_PAYLOAD);
  }

  ESP_LOGI(TAG, "Flushing %d bytes (%d sentences) to UDP broadcast", static_cast<int>(payload.size()), queue_size);

  // Send payload in chunks with error handling
  size_t offset = 0;
  size_t chunks_sent = 0;
  static const size_t MAX_CHUNKS_PER_FLUSH = 50; // Prevent excessive processing time
  
  while (offset < payload.size() && chunks_sent < MAX_CHUNKS_PER_FLUSH) {
    size_t chunk_size = std::min(MAX_UDP_PAYLOAD, payload.size() - offset);
    
    // Simple error checking - if UDP component becomes null or network disconnects, abort
    if (!udp_ || !esphome::network::is_connected()) {
      ESP_LOGW(TAG, "UDP component or network unavailable during send, aborting");
      break;
    }
    
    // Send the packet - no exception handling available in ESP-IDF
    udp_->send_packet(reinterpret_cast<const uint8_t *>(payload.c_str() + offset), chunk_size);
    ESP_LOGV(TAG, "flush_udp_broadcast: sent chunk %d (%d bytes)", chunks_sent + 1, static_cast<int>(chunk_size));
    
    offset += chunk_size;
    chunks_sent++;
    
    // Yield periodically for long payloads to prevent watchdog
    if (chunks_sent % 10 == 0) {
      vTaskDelay(1);
    }
  }
  
  if (offset < payload.size()) {
    ESP_LOGW(TAG, "Did not send complete payload: %d/%d bytes sent", offset, payload.size());
  }
}

}  // namespace gps_idf
}  // namespace esphome