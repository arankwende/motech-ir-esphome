#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/remote_transmitter/remote_transmitter.h"

namespace esphome {
namespace motech_ac {

// ─────────────────────────────────────────────
//  Protocol constants (decoded from captured signals)
//
//  32-bit NEC-style frame @ 38 kHz:
//    [Byte1: Mode] [Byte2: Temp] [Byte3: Fan] [Byte4: 0x08]
//
//  Byte 1 – Mode
//    0x10 = Cool
//    0xD0 = Heat
//    0x50 = Fan Only
//    0x40 = Off
//
//  Byte 2 – Temperature (upper nibble = LSB-reversed offset from 15°C)
//    offset  = target_temp - 15
//    nibble  = bit-reverse of offset (4 bits)
//    byte    = (nibble << 4) | 0x08
//    e.g.  18°C → offset 3 → 0011 → reversed 1100 → 0xC8
//
//  Byte 3 – Fan speed
//    0xC0 = Auto
//    0x80 = High
//    0x40 = Medium
//    0x00 = Low
//
//  Byte 4 – Always 0x08 (device constant)
// ─────────────────────────────────────────────

static const uint8_t MODE_COOL     = 0x10;
static const uint8_t MODE_HEAT     = 0xD0;
static const uint8_t MODE_FAN_ONLY = 0x50;
static const uint8_t MODE_OFF      = 0x40;

static const uint8_t FAN_AUTO   = 0xC0;
static const uint8_t FAN_HIGH   = 0x80;
static const uint8_t FAN_MEDIUM = 0x40;
static const uint8_t FAN_LOW    = 0x00;

static const uint8_t  DEVICE_CONST   = 0x08;
static const uint8_t  TEMP_MIN       = 16;
static const uint8_t  TEMP_MAX       = 30;

// NEC timing (microseconds)
static const uint16_t NEC_HDR_MARK   = 9200;
static const uint16_t NEC_HDR_SPACE  = 4500;
static const uint16_t NEC_BIT_MARK   =  560;
static const uint16_t NEC_ONE_SPACE  = 1690;
static const uint16_t NEC_ZERO_SPACE =  560;


class MotechACClimate : public climate::Climate, public Component {
 public:
  void set_transmitter(remote_transmitter::RemoteTransmitterComponent *tx) {
    this->transmitter_ = tx;
  }

  void setup() override {
    // Restore previous state if available
    auto restore = this->restore_state_();
    if (restore.has_value()) {
      restore->apply(*this);
    } else {
      this->mode             = climate::CLIMATE_MODE_OFF;
      this->target_temperature = 22;
      this->fan_mode         = climate::CLIMATE_FAN_AUTO;
    }
  }

  climate::ClimateTraits traits() override {
    auto traits = climate::ClimateTraits();

    traits.set_supports_current_temperature(false);
    traits.set_supports_two_point_target_temperature(false);

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

    traits.set_visual_min_temperature(TEMP_MIN);
    traits.set_visual_max_temperature(TEMP_MAX);
    traits.set_visual_temperature_step(1.0f);

    return traits;
  }

  void control(const climate::ClimateCall &call) override {
    if (call.get_mode().has_value())
      this->mode = *call.get_mode();

    if (call.get_target_temperature().has_value())
      this->target_temperature = *call.get_target_temperature();

    if (call.get_fan_mode().has_value())
      this->fan_mode = *call.get_fan_mode();

    this->transmit_state_();
    this->publish_state();
  }

 protected:
  remote_transmitter::RemoteTransmitterComponent *transmitter_{nullptr};

  // ── Encode temperature ──────────────────────────────────────────────
  // Clamp, compute offset from 15, bit-reverse the lower 4 bits,
  // then pack as upper nibble with 0x8 in the lower nibble.
  uint8_t encode_temperature_(float temp_f) {
    uint8_t temp = (uint8_t) std::max((float)TEMP_MIN,
                              std::min((float)TEMP_MAX, roundf(temp_f)));
    uint8_t offset = temp - 15;
    uint8_t rev = 0;
    for (int i = 0; i < 4; i++) {
      if (offset & (1 << i))
        rev |= (1 << (3 - i));
    }
    return (rev << 4) | 0x08;
  }

  // ── Build the 32-bit frame ───────────────────────────────────────────
  uint32_t build_frame_() {
    // Byte 1: Mode
    uint8_t b1;
    switch (this->mode) {
      case climate::CLIMATE_MODE_COOL:     b1 = MODE_COOL;     break;
      case climate::CLIMATE_MODE_HEAT:     b1 = MODE_HEAT;     break;
      case climate::CLIMATE_MODE_FAN_ONLY: b1 = MODE_FAN_ONLY; break;
      default:                             b1 = MODE_OFF;       break;
    }

    // Byte 2: Temperature (irrelevant when OFF, but send last known)
    uint8_t b2 = encode_temperature_(this->target_temperature);

    // Byte 3: Fan speed
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

  // ── Transmit via NEC protocol ────────────────────────────────────────
  void transmit_state_() {
    uint32_t frame = build_frame_();

    ESP_LOGD("motech_ac", "Transmitting frame: 0x%08X  (mode=%d temp=%.0f fan=%d)",
             frame, (int)this->mode, this->target_temperature,
             (int)this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO));

    auto transmit = this->transmitter_->transmit();
    auto *data = transmit.get_data();
    data->set_carrier_frequency(38000);

    // NEC header
    data->mark(NEC_HDR_MARK);
    data->space(NEC_HDR_SPACE);

    // 32 bits, MSB first
    for (int i = 31; i >= 0; i--) {
      data->mark(NEC_BIT_MARK);
      data->space((frame >> i) & 1u ? NEC_ONE_SPACE : NEC_ZERO_SPACE);
    }

    // Stop bit
    data->mark(NEC_BIT_MARK);

    transmit.perform();
  }
};

}  // namespace motech_ac
}  // namespace esphome
