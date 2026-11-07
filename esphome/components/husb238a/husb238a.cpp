#include "husb238a.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cstring>
#include <map>

namespace esphome {
namespace husb238a {

static const char *const TAG = "husb238a";

static const std::map<std::string, PdoSelect> SELECT_VOLTAGE{
    {"5V", PdoSelect::SRC_PDO_5V},
    {"9V", PdoSelect::SRC_PDO_9V},
    {"12V", PdoSelect::SRC_PDO_12V},
    {"15V", PdoSelect::SRC_PDO_15V},
    {"20V", PdoSelect::SRC_PDO_20V},
};

static float pdo_select_to_voltage(uint8_t pdo_select) {
  switch (static_cast<PdoSelect>(pdo_select)) {
    case PdoSelect::SRC_PDO_5V:
      return 5.0f;
    case PdoSelect::SRC_PDO_9V:
      return 9.0f;
    case PdoSelect::SRC_PDO_12V:
      return 12.0f;
    case PdoSelect::SRC_PDO_15V:
      return 15.0f;
    case PdoSelect::SRC_PDO_20V:
      return 20.0f;
    default:
      return 0.0f;
  }
}

// Mirrors pdo_select_to_voltage(), but as the option string the select platform publishes/expects.
static const char *pdo_select_to_string(uint8_t pdo_select) {
  switch (static_cast<PdoSelect>(pdo_select)) {
    case PdoSelect::SRC_PDO_5V:
      return "5V";
    case PdoSelect::SRC_PDO_9V:
      return "9V";
    case PdoSelect::SRC_PDO_12V:
      return "12V";
    case PdoSelect::SRC_PDO_15V:
      return "15V";
    case PdoSelect::SRC_PDO_20V:
      return "20V";
    default:
      return nullptr;
  }
}

static float pd_contract_to_voltage(uint8_t pd_contract) {
  switch (static_cast<PdContract>(pd_contract)) {
    case PdContract::TYPE_C_5V:
      return 5.0f;
    case PdContract::SRC_PDO_5V:
      return 5.0f;
    case PdContract::SRC_PDO_9V:
      return 9.0f;
    case PdContract::SRC_PDO_12V:
      return 12.0f;
    case PdContract::SRC_PDO_15V:
      return 15.0f;
    case PdContract::SRC_PDO_20V:
      return 20.0f;
    default:
      return 0.0f;
  }
}

// CONTRACT_STATUS1 encodes the negotiated operating current for a fixed PDO contract: 20 mA/LSB up to
// 0x7D, 40 mA/LSB above that, offset by 500 mA. Source: Pythonic-Rainbow/HUSB238A Conversion.hpp
// (from_contract_current_fpdo), not documented in the public Hynetek datasheet.
static float contract_current_to_amps(uint8_t contract_status1) {
  int step = contract_status1 <= 0x7D ? 20 : 40;
  return (500 + contract_status1 * step) / 1000.0f;
}

// SRC_PDO_xV.current is 100 mA/LSB.
static float src_pdo_current_to_amps(uint8_t current_field) { return current_field * 0.1f; }

void Husb238aComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HUSB238A...");

  this->voltage_pref_ = global_preferences->make_preference<uint8_t>(fnv1_hash("husb238a_voltage") ^
                                                                       this->get_i2c_address());

  // HUSB238A gates its whole PD policy engine behind CONTROL1.ENABLE: with the bit clear the chip
  // only answers I2C (datasheet, "Enable Control"), presents Rd and lets the board run on the
  // Type-C 5 V default. Register reads/writes are ACKed either way, so a missing ENABLE write is
  // silent: commands appear to succeed but never reach the policy engine.
  RegControl1 control1;
  if (!this->read_byte(static_cast<uint8_t>(CommandRegister::CONTROL1), &control1.raw)) {
    ESP_LOGE(TAG, "Failed to read CONTROL1");
    this->mark_failed();
    return;
  }

  if (control1.enable) {
    // Warm reboot (OTA, reset button, watchdog): VBUS never dropped, the chip kept running and its
    // explicit contract is still in place. Nothing to negotiate, just re-apply the saved voltage.
    ESP_LOGD(TAG, "Policy engine already enabled, keeping the current contract");
    this->pd_state_ = PdState::RUNNING;
    this->ready_ = true;
    this->set_timeout(DEFER_UPDATE_DELAY_MS, [this]() {
      this->restore_requested_voltage_();
      this->defer_update();
    });
    return;
  }

  // Cold start of the policy engine. Setting ENABLE makes the chip attach as a PD sink and wait
  // for Source_Capabilities. A source only advertises them for a few seconds after the plug goes
  // in; if we enable later than that (e.g. after flashing, or on a reset while plugged in), the
  // sink hears nothing and -- as the PD spec requires -- issues a Hard Reset about 500 ms after
  // attach. The source then removes VBUS for up to a second, the ESP32 brown-outs, and because
  // the chip is powered from VBUS too it comes back with ENABLE clear, so a naive driver repeats
  // this forever. Verified on the Louder-ESP32 Mini 55 with a Mac as source (FSM 0x02 -> 0x2A,
  // then "E BOD: Brownout detector was triggered").
  //
  // So: enable, watch the chip at high rate, and if the source hasn't talked to us within
  // PROBE_WINDOW_MS, switch the engine off again before its Hard Reset timer expires. The board
  // then simply stays on 5 V until the next real power cycle. See loop()/finish_enable_().
  control1.enable = true;
  if (!this->write_byte(static_cast<uint8_t>(CommandRegister::CONTROL1), control1.raw)) {
    ESP_LOGE(TAG, "Failed to enable HUSB238A");
    this->mark_failed();
    return;
  }
  this->enable_time_ = millis();
  this->pd_state_ = PdState::PROBING;
  this->high_freq_.start();
}

void Husb238aComponent::loop() {
  if (this->pd_state_ != PdState::PROBING) {
    return;
  }
  if (millis() - this->enable_time_ < PROBE_WINDOW_MS) {
    return;
  }
  this->high_freq_.stop();
  this->finish_enable_(this->source_is_talking_());
}

// True once the source has sent us anything: a capability list (SRC_PDO_xV.detect), an explicit
// contract, or just PD traffic (STATUS1.PD_COMM).
bool Husb238aComponent::source_is_talking_() {
  bool is_changed{false};
  if (!this->read_status_block_(is_changed)) {
    return false;
  }
  const RegSrcPdoXxv *pdos[] = {&this->registers_.src_pdo_5v, &this->registers_.src_pdo_9v,
                                 &this->registers_.src_pdo_12v, &this->registers_.src_pdo_15v,
                                 &this->registers_.src_pdo_20v};
  for (auto *pdo : pdos) {
    if (pdo->detect) {
      return true;
    }
  }
  return this->registers_.status1.pd_comm || this->registers_.contract_status0.pd_contract != 0;
}

void Husb238aComponent::finish_enable_(bool source_is_talking) {
  if (source_is_talking) {
    ESP_LOGI(TAG, "USB-PD source answered within %u ms, policy engine enabled", (unsigned) PROBE_WINDOW_MS);
    this->pd_state_ = PdState::RUNNING;
    this->ready_ = true;
    // The chip is negotiating its own first contract right now; wait for that AMS to settle
    // before asking for the saved voltage, then again before the first status poll.
    this->set_timeout(DEFER_UPDATE_DELAY_MS, [this]() {
      this->restore_requested_voltage_();
      this->defer_update();
    });
    return;
  }

  RegControl1 control1;
  if (this->read_byte(static_cast<uint8_t>(CommandRegister::CONTROL1), &control1.raw)) {
    control1.enable = false;
    this->write_byte(static_cast<uint8_t>(CommandRegister::CONTROL1), control1.raw);
  }
  this->pd_state_ = PdState::DISABLED;
  ESP_LOGW(TAG,
           "No USB-PD source capabilities within %u ms of enabling the chip: the source stopped advertising "
           "(board reset while plugged in, or a non-PD supply). Policy engine disabled again to avoid a PD Hard "
           "Reset that would cut VBUS and brown out the board. Staying on the Type-C 5 V default; unplug for 30 s "
           "and plug back in to negotiate USB-PD.",
           (unsigned) PROBE_WINDOW_MS);
  this->status_set_warning(LOG_STR("USB-PD disabled, power cycle needed"));

#ifdef USE_BINARY_SENSOR
  if (this->attached_binary_sensor_ != nullptr) {
    this->attached_binary_sensor_->publish_state(false);
  }
#endif
#ifdef USE_TEXT_SENSOR
  if (this->status_text_sensor_ != nullptr) {
    this->status_text_sensor_->publish_state("Disabled (power cycle needed)");
  }
  if (this->capabilities_text_sensor_ != nullptr) {
    this->capabilities_text_sensor_->publish_state("Unknown (USB-PD disabled)");
  }
#endif
}

bool Husb238aComponent::pdo_offered_(PdoSelect voltage) {
  switch (voltage) {
    case PdoSelect::SRC_PDO_5V:
      return this->registers_.src_pdo_5v.detect;
    case PdoSelect::SRC_PDO_9V:
      return this->registers_.src_pdo_9v.detect;
    case PdoSelect::SRC_PDO_12V:
      return this->registers_.src_pdo_12v.detect;
    case PdoSelect::SRC_PDO_15V:
      return this->registers_.src_pdo_15v.detect;
    case PdoSelect::SRC_PDO_20V:
      return this->registers_.src_pdo_20v.detect;
    default:
      return false;
  }
}

void Husb238aComponent::restore_requested_voltage_() {
  // The chip renegotiates its Type-C default (5V) on every power-up / hard reset, forgetting
  // whatever fixed voltage was last requested over I2C. Re-request it here so a reboot doesn't
  // silently drop the board back to 5V. Falls back to 5V itself when nothing was saved yet.
  uint8_t stored = static_cast<uint8_t>(PdoSelect::SRC_PDO_5V);
  this->voltage_pref_.load(&stored);
  const auto want = static_cast<PdoSelect>(stored);
  const uint32_t started = millis();

  // GO_COMMAND is ignored while the chip is in the middle of an AMS (its own start-up
  // negotiation, typically). Poll until it is idle, then request -- but only a PDO the source
  // actually offers, and only if that isn't already the contract.
  this->set_interval("husb238a_restore", 100, [this, want, started]() {
    bool is_changed{false};
    if (!this->read_status_block_(is_changed)) {
      return;
    }
    if (this->registers_.status.ams_process && millis() - started < RESTORE_TIMEOUT_MS) {
      return;
    }
    this->cancel_interval("husb238a_restore");

    if (this->registers_.contract_status0.pd_contract == static_cast<uint8_t>(want)) {
      ESP_LOGD(TAG, "Saved voltage %.0f V is already the active contract", pdo_select_to_voltage(static_cast<uint8_t>(want)));
      return;
    }
    if (!this->pdo_offered_(want)) {
      ESP_LOGW(TAG, "Saved voltage %.0f V is not offered by this source (%s), keeping the current contract",
               pdo_select_to_voltage(static_cast<uint8_t>(want)), this->get_capabilities_().c_str());
      return;
    }
    if (!this->command_request_pdo(want)) {
      ESP_LOGW(TAG, "Failed to restore last requested PD voltage");
    }
    this->defer_update();
  });
}

void Husb238aComponent::update() {
  if (!this->ready_) {
    return;
  }

  bool is_changed{false};
  if (!this->read_status_block_(is_changed)) {
    is_changed = !this->status_has_error();
    this->status_set_error(LOG_STR("Unable to communicate with HUSB238A chip"));
    memset(&this->registers_, 0, sizeof(this->registers_));
  } else {
    this->status_clear_error();
  }

  if (!is_changed) {
    return;
  }

#ifdef USE_BINARY_SENSOR
  if (this->attached_binary_sensor_ != nullptr) {
    this->attached_binary_sensor_->publish_state(this->registers_.status.attach);
  }
#endif

#ifdef USE_SENSOR
  if (this->voltage_sensor_ != nullptr) {
    float voltage = this->registers_.status.attach
                        ? pd_contract_to_voltage(this->registers_.contract_status0.pd_contract)
                        : 0.0f;
    this->voltage_sensor_->publish_state(voltage);
  }

  if (this->current_sensor_ != nullptr) {
    float current =
        this->registers_.status.attach ? contract_current_to_amps(this->registers_.contract_status1) : 0.0f;
    this->current_sensor_->publish_state(current);
  }

  if (this->selected_voltage_sensor_ != nullptr) {
    RegSrcPdoSelect src_pdo{};
    if (this->read_byte(static_cast<uint8_t>(CommandRegister::SRC_PDO), &src_pdo.raw)) {
      this->selected_voltage_sensor_->publish_state(pdo_select_to_voltage(src_pdo.pdo_select));
    }
  }
#endif

#ifdef USE_SELECT
  if (this->voltage_select_ != nullptr) {
    // Keep the select in sync with what the chip actually has selected -- it otherwise never
    // gets an initial state (shows "Unknown") until the user changes it themselves.
    RegSrcPdoSelect src_pdo{};
    if (this->read_byte(static_cast<uint8_t>(CommandRegister::SRC_PDO), &src_pdo.raw)) {
      if (const char *option = pdo_select_to_string(src_pdo.pdo_select)) {
        this->voltage_select_->publish_state(option);
      }
    }
  }
#endif

#ifdef USE_TEXT_SENSOR
  if (this->status_text_sensor_ != nullptr) {
    this->status_text_sensor_->publish_state(this->registers_.status1.ams_succ ? "Success" : "Unknown");
  }

  if (this->capabilities_text_sensor_ != nullptr) {
    this->capabilities_text_sensor_->publish_state(this->get_capabilities_());
  }
#endif
}

void Husb238aComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "HUSB238A:");
  const char *state = "idle";
  switch (this->pd_state_) {
    case PdState::PROBING:
      state = "probing source";
      break;
    case PdState::RUNNING:
      state = "running";
      break;
    case PdState::DISABLED:
      state = "disabled (source silent at boot, power cycle needed)";
      break;
    default:
      break;
  }
  ESP_LOGCONFIG(TAG, "  Policy engine: %s", state);

#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "PD Attached", this->attached_binary_sensor_);
#endif

#ifdef USE_SENSOR
  LOG_SENSOR("  ", "Source Voltage", this->voltage_sensor_);
  LOG_SENSOR("  ", "Source Current", this->current_sensor_);
  LOG_SENSOR("  ", "Selected Voltage", this->selected_voltage_sensor_);
#endif

#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "Last Request Status", this->status_text_sensor_);
  LOG_TEXT_SENSOR("  ", "Capabilities", this->capabilities_text_sensor_);
#endif

#ifdef USE_SELECT
  LOG_SELECT("  ", "Voltage", this->voltage_select_);
#endif
}

bool Husb238aComponent::command_request_voltage(int volt) {
  PdoSelect voltage;
  switch (volt) {
    case 5:
      voltage = PdoSelect::SRC_PDO_5V;
      break;
    case 9:
      voltage = PdoSelect::SRC_PDO_9V;
      break;
    case 12:
      voltage = PdoSelect::SRC_PDO_12V;
      break;
    case 15:
      voltage = PdoSelect::SRC_PDO_15V;
      break;
    case 20:
      voltage = PdoSelect::SRC_PDO_20V;
      break;
    default:
      ESP_LOGE(TAG, "Invalid voltage");
      return false;
  }
  return this->command_request_pdo(voltage);
}

bool Husb238aComponent::command_request_voltage(const std::string &select_state) {
  auto volt = SELECT_VOLTAGE.find(select_state);
  if (volt == SELECT_VOLTAGE.end()) {
    ESP_LOGE(TAG, "Invalid voltage");
    return false;
  }
  return this->command_request_pdo(volt->second);
}

bool Husb238aComponent::command_request_pdo(PdoSelect voltage) {
  if (!this->ready_) {
    if (this->pd_state_ == PdState::DISABLED) {
      ESP_LOGE(TAG, "USB-PD is disabled (source was silent at boot); power-cycle the board to negotiate");
    } else {
      ESP_LOGE(TAG, "Component not ready");
    }
    return false;
  }
  if (!this->pdo_offered_(voltage)) {
    ESP_LOGW(TAG, "%.0f V is not offered by this source (%s), not requesting it",
             pdo_select_to_voltage(static_cast<uint8_t>(voltage)), this->get_capabilities_().c_str());
    return false;
  }

  if (!this->select_pdo_(voltage)) {
    ESP_LOGV(TAG, "Select PDO voltage failed");
    return false;
  }
  delay(5);

  if (!this->send_command_(GoCommandFunction::SELECT_PDO)) {
    ESP_LOGV(TAG, "Send SELECT_PDO failed");
    return false;
  }

  uint8_t stored = static_cast<uint8_t>(voltage);
  this->voltage_pref_.save(&stored);

  return true;
}

bool Husb238aComponent::is_attached() {
  if (!this->ready_) {
    return false;
  }
  return this->registers_.status.attach;
}

bool Husb238aComponent::read_status_block_(bool &is_changed) {
  uint16_t old_crc = crc16(reinterpret_cast<uint8_t *>(&this->registers_), sizeof(this->registers_));

  auto ok = this->read_bytes(STATUS_BLOCK_START, reinterpret_cast<uint8_t *>(&this->registers_), STATUS_BLOCK_NUM);
  if (!ok) {
    ESP_LOGE(TAG, "Error reading HUSB238A status block");
  }
  is_changed = old_crc != crc16(reinterpret_cast<uint8_t *>(&this->registers_), sizeof(this->registers_));

  return ok;
}

bool Husb238aComponent::send_command_(GoCommandFunction function) {
  ESP_LOGV(TAG, "Sending command %u", static_cast<uint8_t>(function));

  if (!this->ready_) {
    ESP_LOGE(TAG, "Component not ready");
    return false;
  }
  RegGoCommand go_command{};
  go_command.function = static_cast<uint8_t>(function);
  auto ok = this->write_byte(static_cast<uint8_t>(CommandRegister::GO_COMMAND), go_command.raw);
  if (!ok) {
    ESP_LOGE(TAG, "Error sending command to HUSB238A");
  }
  return ok;
}

bool Husb238aComponent::select_pdo_(PdoSelect voltage) {
  ESP_LOGV(TAG, "Setting PDO select to %.0f", pdo_select_to_voltage(static_cast<uint8_t>(voltage)));
  if (!this->ready_) {
    ESP_LOGE(TAG, "Component not ready");
    return false;
  }

  RegSrcPdoSelect reg_data{};
  reg_data.pdo_select = static_cast<uint8_t>(voltage);
  auto ok = this->write_byte(static_cast<uint8_t>(CommandRegister::SRC_PDO), reg_data.raw);
  if (!ok) {
    ESP_LOGE(TAG, "Error setting PDO select");
  }
  return ok;
}

std::string Husb238aComponent::get_capabilities_() {
  const RegSrcPdoXxv *pdos[] = {&this->registers_.src_pdo_5v, &this->registers_.src_pdo_9v,
                                 &this->registers_.src_pdo_12v, &this->registers_.src_pdo_15v,
                                 &this->registers_.src_pdo_20v};
  const uint8_t voltages[] = {5, 9, 12, 15, 20};

  bool nothing_detected = true;
  for (auto *pdo : pdos) {
    if (pdo->detect) {
      nothing_detected = false;
      break;
    }
  }
  if (nothing_detected) {
    return "No capabilities detected";
  }

  std::string capabilities;
  capabilities.reserve(96);
  char buffer[32];
  for (size_t i = 0; i < 5; i++) {
    if (pdos[i]->detect) {
      snprintf(buffer, sizeof(buffer), "%dV: %.1fA", voltages[i], src_pdo_current_to_amps(pdos[i]->current));
      if (!capabilities.empty()) {
        capabilities += ", ";
      }
      capabilities += buffer;
    }
  }
  return capabilities;
}

}  // namespace husb238a
}  // namespace esphome
