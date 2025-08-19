// my_components/gps_tiny/gps_tiny.cpp
#include "gps-idf.h"
#include "esphome/core/log.h"

namespace esphome
{
    namespace gps_idf
    {

        static const char *const TAG = "gps_tiny";

        void GPSIDFComponent::loop()
        {
            // Drain UART and feed TinyGPS++ byte-by-byte
            while (this->available())
            {
                uint8_t c;
                if (this->read_byte(&c))
                {
                    this->gps_.encode(c);
                }
                else
                {
                    break;
                }
            }
        }

        void GPSIDFComponent::update()
        {
            // Determine fix validity
            const bool has_fix = this->gps_.location.isValid() && !this->gps_.location.isUpdated() ? this->gps_.location.age() < 2000 : this->gps_.location.isValid();

            if (this->fix_ != nullptr)
                this->fix_->publish_state(has_fix);

            if (this->publish_only_on_fix_ && !has_fix)
            {
                ESP_LOGV(TAG, "No valid fix yet; skipping publish");
                return;
            }
            if (this->latitude_ != nullptr && this->gps_.location.isValid())
                this->latitude_->publish_state(this->gps_.location.lat());

            if (this->longitude_ != nullptr && this->gps_.location.isValid())
                this->longitude_->publish_state(this->gps_.location.lng());

            if (this->altitude_ != nullptr && this->gps_.altitude.isValid())
                this->altitude_->publish_state(this->gps_.altitude.meters());

            if (this->speed_ != nullptr && this->gps_.speed.isValid())
                this->speed_->publish_state(this->gps_.speed.kmph());

            if (this->course_ != nullptr && this->gps_.course.isValid())
                this->course_->publish_state(this->gps_.course.deg());

            if (this->hdop_ != nullptr && this->gps_.hdop.isValid())
                this->hdop_->publish_state(this->gps_.hdop.value() / 100.0f); // TinyGPS++ returns hundredths

            if (this->satellites_ != nullptr && this->gps_.satellites.isValid())
                this->satellites_->publish_state(this->gps_.satellites.value());

            if (this->date_ != nullptr && this->gps_.date.isValid())
            {
                char buf[16];
                snprintf(buf, sizeof(buf), "%04d-%02d-%02d",
                         this->gps_.date.year(), this->gps_.date.month(), this->gps_.date.day());
                this->date_->publish_state(buf);
            }

            if (this->time_ != nullptr && this->gps_.time.isValid())
            {
                char buf[16];
                snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
                         this->gps_.time.hour(), this->gps_.time.minute(), this->gps_.time.second());
                this->time_->publish_state(buf);
            }

            if (this->datetime_ != nullptr && this->gps_.date.isValid() && this->gps_.time.isValid())
            {
                char buf[24];
                snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                         this->gps_.date.year(), this->gps_.date.month(), this->gps_.date.day(),
                         this->gps_.time.hour(), this->gps_.time.minute(), this->gps_.time.second());
                this->datetime_->publish_state(buf);
            }
        }

        void GPSIDFComponent::dump_config()
        {
            ESP_LOGCONFIG(TAG, "GPS Tiny (TinyGPS++)");
            LOG_UART_DEVICE(this);
            ESP_LOGCONFIG(TAG, "publish_only_on_fix: %s", YESNO(this->publish_only_on_fix_));
        }

    } // namespace gps_tiny
} // namespace esphome