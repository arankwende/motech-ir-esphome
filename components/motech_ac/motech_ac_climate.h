#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/remote_transmitter/remote_transmitter.h"
#include "esphome/components/remote_base/remote_base.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace motech_ac {

// ─────────────────────────────────────────────────────────────────────
//  Protocol constants (decoded from captured signals)
//
//  32-bit NEC-style frame @ 38 kHz:
//    [Byte1: Mode] [Byte2: Temp] [Byte3: Fan] [Byte4: 0x08]
//
//  Byte 1 – Mode
//    0x10 = Cool  |  0xD0 = Heat  |  0x50 = Fan Only  |  0x40 = Off
//
//  Byte 2 – Temperature  (upper nibble = LSB-reversed offset from 15°C)
//    offset = target_temp - 15,  nibble = bit-reverse of offset (4 bits)
//    byte   = (nibble << 4) | 0x08
//
//  Byte 3 – Fan speed
//    0xC0 = Auto  |  0x80 = High  |  0x40 = Medium  |  0x00 = Low
//
//  Byte 4 – Always 0x08  (device constant)
// ─────────────────────────────────────────────────────────────────────

static const uint8_t MODE_COOL     = 0x10;
static const uint8_t MODE_HEAT     = 0xD0;
static const uint8_t MODE_FAN_ONLY = 0x50;
static const uint8_t MODE_OFF      = 0x40;

static const uint8_t FAN_AUTO   = 0xC0;
static const uint8_t FAN_HIGH   = 0x80;
static const uint8_t FAN_MEDIUM = 0x40;
static const uint8_t FAN_LOW    = 0x00;

static const uint8_t DEVICE_CONST = 0x08;

// NEC timing (microseconds)
static const uint16_t NEC_HDR_MARK   = 9200;
static const uint16_t NEC_HDR_SPACE  = 4500;
static const uint16_t NEC_BIT_MARK   =  560;
static const uint16_t NEC_ONE_SPACE  = 1690;
static const uint16_t NEC_ZERO_SPACE =  560;


class MotechACClimate : public climate::Climate,
                        public Component,
                        public remote_base::RemoteReceiverListener {
 public:
  // ── Setters called from climate.py ─────────────────────────────────
  void set_transmitter(remote_transmitter::RemoteTransmitterComponent *tx) {
    transmitter_ = tx;
  }
  void set_sensor(sensor::Sensor *sensor) {
    sensor_ = sensor;
  }

  // ── Component setup ─────────────────────────────────────────────────
  void setup() override {
    auto restore = this->restore_state_();
    if (restore.has_value()) {
      restore->apply(this);
    } else {
      this->mode               = climate::CLIMATE_MODE_OFF;
      this->target_temperature = 22;
      this->fan_mode           = climate::CLIMATE_FAN_AUTO;
    }

    // If a sensor is wired up, track its value as current temperature
    if (sensor_ != nullptr) {
      sensor_->add_on_state_callback([this](float state) {
        this->current_temperature = state;
        this->publish_state();
      });
      this->current_temperature = sensor_->state;
    }
  }

  // ── Climate traits ──────────────────────────────────────────────────
  climate::ClimateTraits traits() override {
    auto traits = climate::ClimateTraits();

    if (sensor_ != nullptr)
      traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);

    traits.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_FAN_ONLY,
    });

    traits.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
    });

    // visual: min/max/step from YAML override these at the ESPHome layer
    traits.set_visual_min_temperature(16);
    traits.set_visual_max_temperature(30);
    traits.set_visual_temperature_step(1.0f);

    return traits;
  }

  // ── Control (called when HA / user changes state) ───────────────────
  void control(const climate::ClimateCall &call) override {
    if (call.get_mode().has_value())
      this->mode = *call.get_mode();
    if (call.get_target_temperature().has_value())
      this->target_temperature = *call.get_target_temperature();
    if (call.get_fan_mode().has_value())
      this->fan_mode = *call.get_fan_mode();

    transmit_state_();
    this->publish_state();
  }

  // ── IR Receiver listener ────────────────────────────────────────────
  bool on_receive(remote_base::RemoteReceiveData data) override {
    // Validate NEC header
    if (!data.expect_item(NEC_HDR_MARK, NEC_HDR_SPACE))
      return false;

    // Decode 32 bits MSB-first
    uint32_t frame = 0;
    for (int i = 31; i >= 0; i--) {
      if (data.expect_item(NEC_BIT_MARK, NEC_ONE_SPACE)) {
        frame |= (1u << i);
      } else if (data.expect_item(NEC_BIT_MARK, NEC_ZERO_SPACE)) {
        // bit stays 0
      } else {
        return false;  // not our signal
      }
    }

    // Verify device constant in byte 4
    if ((frame & 0xFF) != DEVICE_CONST)
      return false;

    ESP_LOGD("motech_ac", "Received frame: 0x%08X", frame);
    parse_frame_(frame);
    return true;
  }

 protected:
  remote_transmitter::RemoteTransmitterComponent *transmitter_{nullptr};
  sensor::Sensor *sensor_{nullptr};

  // ── Decode temperature from byte 2 ─────────────────────────────────
  float decode_temperature_(uint8_t byte2) {
    uint8_t nibble = (byte2 >> 4) & 0x0F;
    // Bit-reverse the 4-bit nibble
    uint8_t rev = 0;
    for (int i = 0; i < 4; i++) {
      if (nibble & (1 << i))
        rev |= (1 << (3 - i));
    }
    return (float)(rev + 15);
  }

  // ── Encode temperature into byte 2 ─────────────────────────────────
  uint8_t encode_temperature_(float temp_f) {
    uint8_t temp = (uint8_t) std::max(16.0f, std::min(30.0f, roundf(temp_f)));
    uint8_t offset = temp - 15;
    uint8_t rev = 0;
    for (int i = 0; i < 4; i++) {
      if (offset & (1 << i))
        rev |= (1 << (3 - i));
    }
    return (rev << 4) | 0x08;
  }

  // ── Parse a received frame and update climate state ─────────────────
  void parse_frame_(uint32_t frame) {
    uint8_t b1 = (frame >> 24) & 0xFF;
    uint8_t b2 = (frame >> 16) & 0xFF;
    uint8_t b3 = (frame >>  8) & 0xFF;

    // Mode
    switch (b1) {
      case MODE_COOL:     this->mode = climate::CLIMATE_MODE_COOL;     break;
      case MODE_HEAT:     this->mode = climate::CLIMATE_MODE_HEAT;     break;
      case MODE_FAN_ONLY: this->mode = climate::CLIMATE_MODE_FAN_ONLY; break;
      case MODE_OFF:      this->mode = climate::CLIMATE_MODE_OFF;      break;
      default:
        ESP_LOGW("motech_ac", "Unknown mode byte: 0x%02X", b1);
        return;
    }

    // Temperature
    this->target_temperature = decode_temperature_(b2);

    // Fan speed
    switch (b3) {
      case FAN_HIGH:   this->fan_mode = climate::CLIMATE_FAN_HIGH;   break;
      case FAN_MEDIUM: this->fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
      case FAN_LOW:    this->fan_mode = climate::CLIMATE_FAN_LOW;    break;
      default:         this->fan_mode = climate::CLIMATE_FAN_AUTO;   break;
    }

    this->publish_state();
  }

  // ── Build the 32-bit frame ──────────────────────────────────────────
  uint32_t build_frame_() {
    uint8_t b1;
    switch (this->mode) {
      case climate::CLIMATE_MODE_COOL:     b1 = MODE_COOL;     break;
      case climate::CLIMATE_MODE_HEAT:     b1 = MODE_HEAT;     break;
      case climate::CLIMATE_MODE_FAN_ONLY: b1 = MODE_FAN_ONLY; break;
      default:                             b1 = MODE_OFF;       break;
    }

    uint8_t b2 = encode_temperature_(this->target_temperature);

    uint8_t b3;
    switch (this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO)) {
      case climate::CLIMATE_FAN_HIGH:   b3 = FAN_HIGH;   break;
      case climate::CLIMATE_FAN_MEDIUM: b3 = FAN_MEDIUM; break;
      case climate::CLIMATE_FAN_LOW:    b3 = FAN_LOW;    break;
      default:                          b3 = FAN_AUTO;   break;
    }

    return ((uint32_t)b1 << 24) |
           ((uint32_t)b2 << 16) |
           ((uint32_t)b3 <<  8) |
           DEVICE_CONST;
  }

  // ── Transmit via NEC @ 38kHz ────────────────────────────────────────
  void transmit_state_() {
    if (this->transmitter_ == nullptr) {
      ESP_LOGE("motech_ac", "Transmitter not set!");
      return;
    }

    uint32_t frame = build_frame_();

    ESP_LOGI("motech_ac", "TX frame: 0x%08X  mode=%d temp=%.0f fan=%d",
             frame, (int)this->mode, this->target_temperature,
             (int)this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO));

    auto transmit = this->transmitter_->transmit();
    auto *d = transmit.get_data();
    d->set_carrier_frequency(38000);

    // NEC header
    d->mark(NEC_HDR_MARK);
    d->space(NEC_HDR_SPACE);

    // 32 bits MSB first
    for (int i = 31; i >= 0; i--) {
      d->mark(NEC_BIT_MARK);
      if ((frame >> i) & 1u) {
        d->space(NEC_ONE_SPACE);
      } else {
        d->space(NEC_ZERO_SPACE);
      }
    }

    // Stop bit
    d->mark(NEC_BIT_MARK);

    ESP_LOGI("motech_ac", "TX buffer size: %d items", d->get_data().size());
    transmit.perform();
  }
};

}  // namespace motech_ac
}  // namespace esphome
