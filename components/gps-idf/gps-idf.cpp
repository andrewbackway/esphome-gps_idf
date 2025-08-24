#include "gps-idf.h"

#include "esphome/core/log.h"

#include <errno.h>
#include <cstring>

namespace esphome {
namespace gps_idf {

static const char *const TAG = "gps_idf";
static const size_t MAX_UDP_QUEUE_SIZE = 20;
static const size_t MAX_UDP_PAYLOAD = 1400; // safe typical UDP payload

void GPSIDFComponent::setup() {
  ESP_LOGI(TAG, "Setting up GPSIDFComponent...");
  
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
    ESP_LOGCONFIG(TAG, "  Broadcast Address: %s",
                  udp_broadcast_address_.c_str());
    ESP_LOGCONFIG(TAG, "  Port: %d", udp_broadcast_port_);
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
  if (udp_broadcast_enabled_ && udp_socket_ >= 0) {
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

          // ESP_LOGI(TAG, "Processed NMEA sentence: %s", sentence.c_str());
          if (esphome::network::is_connected()) {
            if (self->udp_broadcast_enabled_) {
              if (self->udp_socket_ < 0) {
                self->setup_udp_broadcast();
              } else {
                ESP_LOGD(TAG, "Checking filter");
                // Only queue if no filter configured (empty) OR sentence matches one of the filters
                // Example filter logic using instance member correctly:
                bool passes_filter = self->udp_broadcast_sentence_filter_.empty();
                if ( !passes_filter)
                  ESP_LOGD(TAG, "EMPTY FILTER");

                if (!passes_filter) {
                  for (const auto &f : self->udp_broadcast_sentence_filter_) {
                    if (sentence.find(f) != std::string::npos) {
                      passes_filter = true;
                      break;
                    }
                  }
                }

                if (passes_filter) {
                  ESP_LOGD(TAG, "Sentence passed filter queuing UDP sentence: %s", sentence.c_str());
                  // Use the instance method via self
                  self->queue_udp_sentence(sentence);
                }
              }
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

bool GPSIDFComponent::setup_udp_broadcast() {
  ESP_LOGI(TAG, "Setting up UDP broadcast");

  udp_socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (udp_socket_ < 0) {
    ESP_LOGE(TAG, "Failed to create UDP socket");
    return false;
  }

  // Enable the SO_BROADCAST option for the socket
  int broadcast_enable = 1;
  int ret = setsockopt(udp_socket_, SOL_SOCKET, SO_BROADCAST, &broadcast_enable,
                       sizeof(broadcast_enable));
  if (ret < 0) {
    ESP_LOGE(TAG, "Failed to set SO_BROADCAST option, errno: %d", errno);
    close(udp_socket_);  // Clean up the socket
    udp_socket_ = -1;
    return false;
  }
  ESP_LOGD(TAG, "SO_BROADCAST option set successfully");

  // Set the destination address
  udp_dest_addr_.sin_family = AF_INET;
  udp_dest_addr_.sin_port = htons(udp_broadcast_port_);

  // Convert the string address to a network address
  if (inet_aton(udp_broadcast_address_.c_str(), &udp_dest_addr_.sin_addr) ==
      0) {
    ESP_LOGE(TAG, "Failed to convert broadcast address: %s",
             udp_broadcast_address_.c_str());
    close(udp_socket_);
    udp_socket_ = -1;
    return false;
  }

  ESP_LOGD(TAG, "UDP broadcast setup complete.");

  return true;
}

void GPSIDFComponent::queue_udp_sentence(const std::string &sentence) {
  std::string full_sentence = sentence + "\r\n";
  ESP_LOGD(TAG, "Queueing UDP sentence: %s", full_sentence.c_str());

  if (udp_queue_mutex_ && xSemaphoreTake(udp_queue_mutex_, pdMS_TO_TICKS(100))) {
    if (udp_queue_.size() >= MAX_UDP_QUEUE_SIZE) {
      ESP_LOGV(TAG, "UDP queue full. Dropping oldest sentence to make space.");
      udp_queue_.pop_front();  // O(1) with deque
    }
    udp_queue_.push_back(full_sentence);
    xSemaphoreGive(udp_queue_mutex_);
  } else {
    // If can't take mutex, drop the sentence (keep non-blocking)
    ESP_LOGW(TAG, "queue_udp_sentence: could not take udp_queue_mutex, dropping sentence");
  }
}

void GPSIDFComponent::flush_udp_broadcast() {
  // This function is called from loop() and must be fast and non-blocking.
  if (udp_socket_ < 0 || udp_queue_.empty()) {
    return;
  }

  TickType_t now = xTaskGetTickCount();
  TickType_t interval_ticks = pdMS_TO_TICKS(udp_broadcast_interval_ms_);

  // Immediately return if it's not yet time to send. This is the common case.
  if ((now - last_broadcast_ticks_) < interval_ticks) {
    return;
  }
  last_broadcast_ticks_ = now;

  ESP_LOGD(TAG, "Flushing UDP broadcast");

  // Combine all queued sentences into a single payload.
  // This is far more efficient and avoids overwhelming the network buffers.
  // Copy or swap the queue contents locally while holding mutex, then release.
  std::string payload;
  if (udp_queue_mutex_ && xSemaphoreTake(udp_queue_mutex_, pdMS_TO_TICKS(100))) {
    payload.reserve(udp_queue_.size() * 85);
    for (const auto &sentence : udp_queue_) payload.append(sentence);
    udp_queue_.clear();
    xSemaphoreGive(udp_queue_mutex_);
  } else {
    // couldn't obtain mutex — skip this flush to avoid race
    ESP_LOGW(TAG, "flush_udp_broadcast: could not take udp_queue_mutex");
    return;
  }

  ESP_LOGD(TAG, "Flushing %d bytes to UDP broadcast.",  static_cast<int>(payload.size()));

  size_t offset = 0;
  while (offset < payload.size()) {
    size_t chunk_size = std::min(MAX_UDP_PAYLOAD, payload.size() - offset);
    int bytes_sent = sendto(udp_socket_,
                            payload.c_str() + offset,
                            chunk_size,
                            0,
                            (struct sockaddr *)&udp_dest_addr_,
                            sizeof(udp_dest_addr_));
    if (bytes_sent < 0) {
      int err = errno;
      ESP_LOGW(TAG, "flush_udp_broadcast: sendto failed: %d %s", err, strerror(err));
      break;
    } else {
      ESP_LOGI(TAG, "flush_udp_broadcast: sent %d bytes", bytes_sent);
    }
    offset += chunk_size;
  }
}

}  // namespace gps_idf
}  // namespace esphome
